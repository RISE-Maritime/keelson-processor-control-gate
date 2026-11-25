#!/usr/bin/env python3
"""keelson-processor-control-gate — the onboard motion gate (COMMAND-ARCHITECTURE.md §2.1, §4).

Its own process from day one, even on the same machine as the connector: responsibility isolation,
not performance. It serves `control_gate/v1` to the authority manager (and `abort_automation` to the
assignee when the manager is unhealthy), and it hears:
- `command_authority` -- the conn lease and the assignment sessions, stamped on local receipt;
- `command_assignment` -- its own projection, for the direct abort;
- `safety_state` and `maintenance_state` -- ACTUATION_ALLOWED. A gate that must read the second and
  cannot (an SDK without MaintenanceState, keelson#377) refuses to start rather than inhibit forever.

It hears the candidates -- `control_candidate/**` (a station's helm) and `guidance_candidate/**` (the
ANS, checked for DECISION and EXECUTION, I15) -- and the navigation authority document, and
publishes `resolved_control/{vessel}` every cycle at --rate-hz: the rolling acknowledgement and the only
thing downstream consumes. The actuator itself is still a LOGGING one (README).
"""

import argparse
import logging
import pathlib
import sys
import time

import keelson
import zenoh
from keelson.interfaces.ControlGate_pb2 import AbortAutomationRequest, GetGateStateRequest
from google.protobuf.timestamp_pb2 import Timestamp
from keelson.payloads.CommandAuthority_pb2 import CommandAssignment, CommandAuthority
from keelson.payloads.ControlCandidate_pb2 import ControlCandidate
from keelson.payloads.GuidanceCandidate_pb2 import GuidanceCandidate
from keelson.payloads.ControlMapping_pb2 import TransferCommit, TransferPrepare
from keelson.payloads.NavigationState_pb2 import NavigationState
from keelson.payloads.MotionConstraint_pb2 import MotionConstraint
from keelson.payloads.PropulsionConstraint_pb2 import PropulsionConstraint
from keelson.payloads.SafetyState_pb2 import SafetyState
from keelson.scaffolding import add_common_arguments, create_zenoh_config, setup_logging
from keelson.scaffolding.liveliness import declare_liveliness
from keelson.scaffolding.rpc import RpcOp, serve_rpc
from keelson.scaffolding.signals import GracefulShutdown

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from control_gate import codec
from control_gate.actuation import LoggingActuator
from control_gate.policy import load
from control_gate.service import Gate
from control_gate.store import Store

log = logging.getLogger("control-gate")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="control-gate2keelson", description=__doc__.split("\n")[0])
    add_common_arguments(p)
    p.add_argument("-r", "--realm", required=True, help="Keelson realm, e.g. rise")
    p.add_argument("-e", "--entity-id", required=True, help="The vessel this gate is onboard (the vessel_id)")
    p.add_argument("-s", "--source-id", default="control-gate", help="Source id and RPC responder id")
    p.add_argument("--state-dir", type=pathlib.Path, required=True, help="Directory of the gate's store (a persistent volume)")
    p.add_argument("--policy", type=pathlib.Path, required=True, help="The control matrix and safe_state_policy timings (JSON)")
    p.add_argument("--rate-hz", type=float, default=10.0, help="Resolution rate (default 10 Hz)")
    return p


def _decode(sample, cls):
    msg = cls()
    msg.ParseFromString(keelson.uncover(sample.payload.to_bytes())[2])
    return msg


def _decode_timed(sample, cls):
    """(message, received wall ns, enclosed wall ns): the gate's receipt (t3) and the source's
    publish (t2), for the provenance ResolvedControl carries (crowsnest-dev#98). Never judged."""
    received_ns, enclosed_ns, payload = keelson.uncover(sample.payload.to_bytes())
    msg = cls()
    msg.ParseFromString(payload)
    return msg, received_ns, enclosed_ns


def _maintenance_pb():
    """keelson.MaintenanceState, or None on an SDK that predates it (keelson#377 is not in the pin)."""
    try:
        from keelson.payloads.MaintenanceState_pb2 import MaintenanceState  # pylint: disable=import-outside-toplevel
    except ImportError:
        return None
    return MaintenanceState


def main() -> int:
    args = build_parser().parse_args()
    setup_logging(level=args.log_level)
    zenoh.init_log_from_env_or(logging.getLevelName(args.log_level))
    realm, vessel = args.realm, args.entity_id
    base = f"{realm}/@v0/{vessel}/pubsub"
    policy = load(args.policy)
    if not policy.require_safety_state:
        log.warning("require_safety_state is OFF: a rig with no supervisor. Never on a vessel.")
    maintenance_pb = _maintenance_pb()
    if policy.require_maintenance_state and maintenance_pb is None:
        log.error("require_maintenance_state is on, and this keelson SDK has no MaintenanceState (keelson#377): "
                  "the gate could never allow actuation. Install an SDK that carries it, or set "
                  "require_maintenance_state false in a rig with no manager.")
        return 2
    if not policy.require_maintenance_state:
        log.warning("require_maintenance_state is OFF: a rig with no maintenance_state. Never on a vessel.")
    attested = "attestation" in AbortAutomationRequest.DESCRIPTOR.fields_by_name
    if policy.attestation_required and not attested:
        log.error("attestation is required on the direct abort, and this keelson SDK has no "
                  "AbortAutomationRequest.attestation (crowsnest-dev#316): every direct abort would be refused. "
                  "Install an SDK that carries it.")
        return 2
    # Counts and ids only: a public key is not a secret, but nothing here needs to print one.
    log.info("direct abort: attestation %s, %d trusted key(s)%s",
             "REQUIRED" if policy.attestation_required else "optional (verified when present)",
             len(policy.attestation_keys), f" {sorted(policy.attestation_keys)}" if policy.attestation_keys else "")

    conf = create_zenoh_config(args.mode, args.connect, args.listen, zenoh_config=args.zenoh_config)
    with zenoh.open(conf) as session:
        gate = Gate(store=Store(args.state_dir / "gate.sqlite"), policy=policy, vessel_id=vessel, actuator=LoggingActuator())
        gate.boot()

        def on_authority(sample):
            path = str(sample.key_expr).split("/command_authority/", 1)[1]
            rec = _decode(sample, CommandAuthority)
            ttl = int(rec.lease_ttl_seconds) * 1000
            if path == vessel:
                gate.observe_lease("conn", rec.controller_id, rec.token, ttl, rec.released)
            elif path == f"{vessel}/engineer":
                # A new ENGINEER token is a new plant_generation, minted here (crowsnest-dev#270).
                gate.observe_lease("engineer", rec.controller_id, rec.token, ttl, rec.released, rec.controller_site)
            elif path == f"{vessel}/navigation" and rec.HasField("navigation_authority"):
                gate.observe_navigation(*codec.navigation_from_record(rec))
            elif path in (f"{vessel}/overall/session", f"{vessel}/navigational/session"):
                role = path.split("/")[1]
                gate.observe_session(role, rec.controller_id, rec.token, rec.assignment.assignment_id if rec.HasField("assignment") else "",
                                     ttl, rec.released)

        def on_assignment(sample):
            a = _decode(sample, CommandAssignment)
            gate.observe_assignments(codec.assignment_seen(a.overall) if a.HasField("overall") else None,
                                     codec.assignment_seen(a.navigational) if a.HasField("navigational") else None,
                                     a.issuer_id, int(a.feed_ttl_seconds) * 1000)

        def on_safety(sample):
            s = _decode(sample, SafetyState)
            gate.observe_safety(int(s.state), bool(s.local_lockout), int(s.start_warning.state) if s.HasField("start_warning") else None)

        def on_maintenance(sample):
            gate.observe_maintenance(int(_decode(sample, maintenance_pb).state))

        def on_constraint(sample):
            gate.observe_constraint(codec.constraint_from_proto(_decode(sample, PropulsionConstraint)))

        def on_motion(sample):
            pb = _decode(sample, MotionConstraint)
            if pb.vessel_id and pb.vessel_id != vessel:
                log.warning("motion_constraint for vessel %r ignored", pb.vessel_id)
                return
            gate.observe_motion(codec.motion_from_proto(pb))

        def guarded(fn):
            def run(sample):
                try:
                    fn(sample)
                except Exception:  # pylint: disable=broad-except
                    log.exception("could not handle %s", sample.key_expr)
            return run

        def mono_ms():
            return int(time.monotonic() * 1000)

        def on_control_candidate(sample):
            recv_ms = mono_ms()
            msg, received_ns, enclosed_ns = _decode_timed(sample, ControlCandidate)
            gate.sample(codec.sample_from_control_candidate(msg, recv_ms, received_ns, enclosed_ns))

        def on_guidance_candidate(sample):
            recv_ms = mono_ms()
            msg, received_ns, enclosed_ns = _decode_timed(sample, GuidanceCandidate)
            gate.guidance(codec.guidance_from_proto(msg, recv_ms, received_ns, enclosed_ns))

        def on_navigation_state(sample):
            # The producer is the key's remainder after the subject: what a HUMAN_ORDER's source spells.
            producer = str(sample.key_expr).split("/navigation_state/", 1)[1]
            ns = _decode(sample, NavigationState)
            gate.observe_navigation_state(producer, ns.order_id if ns.HasField("order_id") else None)

        subs = [
            session.declare_subscriber(f"{base}/control_candidate/**", guarded(on_control_candidate)),
            session.declare_subscriber(f"{base}/guidance_candidate/**", guarded(on_guidance_candidate)),
            session.declare_subscriber(f"{base}/navigation_state/**", guarded(on_navigation_state)),
            session.declare_subscriber(f"{base}/command_authority/**", guarded(on_authority)),
            session.declare_subscriber(f"{base}/command_assignment/{vessel}", guarded(on_assignment)),
            session.declare_subscriber(f"{base}/safety_state/{vessel}", guarded(on_safety)),
            # Live only, never read from storage at boot: a stored INACTIVE says nothing about a live
            # manager, and its heartbeat (5 s) arrives inside the lease grace anyway.
            *([session.declare_subscriber(f"{base}/maintenance_state/{vessel}", guarded(on_maintenance))]
              if maintenance_pb is not None else []),
            session.declare_subscriber(f"{base}/propulsion_constraint/{vessel}", guarded(on_constraint)),
            # One slot per payload (keelson MotionConstraint): the interim's rules, crowsnest-dev#85.
            session.declare_subscriber(f"{base}/motion_constraint/{vessel}/*", guarded(on_motion)),
        ]
        # Stored: a constraint written while this gate was down is still the one in force. On a new
        # term it is restored and rebased, and actuation waits for the ENGINEER (I16, crowsnest-dev#345).
        stored, unreadable = [], []
        try:
            replies = list(session.get(f"{base}/propulsion_constraint/{vessel}", timeout=2.0))
        except Exception as e:  # pylint: disable=broad-except
            # Not "none": a read that failed says nothing about what is in force, so it holds like an
            # invalid record -- and, like one, the ENGINEER's record under the new term lifts it.
            replies, unreadable = [], [f"the storage read failed: {e}"]
        for reply in replies:
            if reply.ok is None:
                continue
            try:
                pb = _decode(reply.ok, PropulsionConstraint)
            except Exception as e:  # pylint: disable=broad-except
                unreadable.append(f"undecodable: {e}")
                continue
            c = codec.constraint_from_proto(pb)
            if pb.vessel_id and pb.vessel_id != vessel:
                unreadable.append(f"written for vessel {pb.vessel_id!r}")
            elif c is None:
                unreadable.append("it carries no fence")
            else:
                stored.append(c)
        gate.restore_constraints(stored, unreadable)

        # The same for motion_constraint (I16): every payload's slot, read once. A latched slot written
        # while this gate was down is still in force -- the sonar is still in the water.
        motion, bad_motion = [], []
        try:
            motion_replies = list(session.get(f"{base}/motion_constraint/{vessel}/*", timeout=2.0))
        except Exception as e:  # pylint: disable=broad-except
            motion_replies, bad_motion = [], [("?", f"the storage read failed: {e}")]
        for reply in motion_replies:
            if reply.ok is None:
                continue
            payload = str(reply.ok.key_expr).rsplit("/", 1)[-1] or "?"
            try:
                pb = _decode(reply.ok, MotionConstraint)
            except Exception as e:  # pylint: disable=broad-except
                bad_motion.append((payload, f"undecodable: {e}"))
                continue
            m = codec.motion_from_proto(pb)
            if pb.vessel_id and pb.vessel_id != vessel:
                bad_motion.append((payload, f"written for vessel {pb.vessel_id!r}"))
            elif m is None:
                bad_motion.append((payload, "it carries no fence"))
            else:
                motion.append(m)
        gate.restore_motion(motion, bad_motion)

        def parse(cls, op):
            msg = cls()
            msg.ParseFromString(op.request_bytes)
            return msg

        def prepare(op: RpcOp):
            p = parse(TransferPrepare, op)
            if p.vessel_id and p.vessel_id != vessel:
                op.reply_err(f"this gate is {vessel}'s, not {p.vessel_id}'s")
                return
            op.reply_ok(codec.ack_to_proto(p.transfer_id, gate.prepare(codec.proposal_from_proto(p))))

        def commit(op: RpcOp):
            c = parse(TransferCommit, op)
            applied, reason, fence = gate.commit(c.transfer_id, codec.fence_from_proto(c.fence))
            op.reply_ok(codec.commit_response(c.transfer_id, applied, reason, fence))

        def get_gate_state(op: RpcOp):
            parse(GetGateStateRequest, op)
            op.reply_ok(codec.gate_state(gate.exposed()))

        def abort_automation(op: RpcOp):
            ok, detail, fence = gate.abort_direct(codec.direct_abort_from_proto(parse(AbortAutomationRequest, op)))
            op.reply_ok(codec.abort_response(ok, detail, fence))

        serve_rpc(session, base_path=realm, entity_id=vessel, responder_id=args.source_id, interface="control_gate", version="v1",
                  handlers={"prepare": prepare, "commit": commit, "get_gate_state": get_gate_state,
                            "abort_automation": abort_automation}, log=log)

        period = 1.0 / args.rate_hz
        with declare_liveliness(session, realm, vessel, args.source_id), GracefulShutdown() as shutdown:
            while not shutdown.is_requested():
                t0 = time.monotonic()
                try:
                    gate.cycle()
                    if gate.last_cycle is not None:
                        ts = Timestamp()
                        ts.GetCurrentTime()
                        rc = codec.resolved_control(gate.last_cycle, vessel, args.source_id, ts)
                        session.put(f"{base}/resolved_control/{vessel}", keelson.enclose(rc.SerializeToString()))
                    for c in gate.take_publications():
                        # The inherited constraint, rebased at revision 1 under the new plant_generation (I16).
                        ts = Timestamp()
                        ts.GetCurrentTime()
                        session.put(f"{base}/propulsion_constraint/{vessel}",
                                    keelson.enclose(codec.constraint_to_proto(c, vessel, ts).SerializeToString()))
                except Exception:  # pylint: disable=broad-except
                    log.exception("cycle failed")
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        for s in subs:
            s.undeclare()
    gate.store.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
