#!/usr/bin/env python3
"""keelson-processor-control-gate — the onboard motion gate (COMMAND-ARCHITECTURE.md §2.1, §4).

Its own process from day one, even on the same machine as the connector: responsibility isolation,
not performance. It serves `control_gate/v1` to the authority manager (and `abort_automation` to the
assignee when the manager is unhealthy), and it hears:
- `command_authority` -- the conn lease and the assignment sessions, stamped on local receipt;
- `command_assignment` -- its own projection, for the direct abort;
- `safety_state` -- ACTUATION_ALLOWED.

It resolves every DOF at --rate-hz. This cut emits to a LOGGING actuator only: no candidate subject
exists yet (no sample reaches it over the bus) and nothing consumes ResolvedControl (README).
"""

import argparse
import logging
import pathlib
import sys
import time

import keelson
import zenoh
from keelson.interfaces.ControlGate_pb2 import AbortAutomationRequest, GetGateStateRequest
from keelson.payloads.CommandAuthority_pb2 import CommandAssignment, CommandAuthority
from keelson.payloads.ControlMapping_pb2 import TransferCommit, TransferPrepare
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


def main() -> int:
    args = build_parser().parse_args()
    setup_logging(level=args.log_level)
    zenoh.init_log_from_env_or(logging.getLevelName(args.log_level))
    realm, vessel = args.realm, args.entity_id
    base = f"{realm}/@v0/{vessel}/pubsub"
    policy = load(args.policy)
    if not policy.require_safety_state:
        log.warning("require_safety_state is OFF: a rig with no supervisor. Never on a vessel.")

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
            gate.observe_safety(int(s.state), bool(s.local_lockout))

        def guarded(fn):
            def run(sample):
                try:
                    fn(sample)
                except Exception:  # pylint: disable=broad-except
                    log.exception("could not handle %s", sample.key_expr)
            return run

        subs = [
            session.declare_subscriber(f"{base}/command_authority/**", guarded(on_authority)),
            session.declare_subscriber(f"{base}/command_assignment/{vessel}", guarded(on_assignment)),
            session.declare_subscriber(f"{base}/safety_state/{vessel}", guarded(on_safety)),
        ]

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
                except Exception:  # pylint: disable=broad-except
                    log.exception("cycle failed")
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        for s in subs:
            s.undeclare()
    gate.store.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
