"""The gate CLI as a real process -- alone, and with the authority manager's CLI -- on an ISOLATED
loopback Zenoh pair: multicast and gossip scouting off everywhere, so nothing here can reach a shared
network or a vessel."""

import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import keelson
import pytest
import zenoh
from keelson.interfaces.ControlGate_pb2 import AbortAutomationRequest, AbortAutomationResponse, CommitResponse, GateState
from keelson.interfaces.VehicleCommon_pb2 import CommandResult
from keelson.payloads.CommandAuthority_pb2 import Assignment, CommandAssignment, CommandAuthority, RecordKind, Role
from keelson.payloads.ControlMapping_pb2 import ControlMapping, TransferAck, TransferCommit, TransferKind, TransferPrepare
from keelson.payloads.SafetyState_pb2 import SafetyState

pytestmark = pytest.mark.e2e

ROOT = Path(__file__).resolve().parent.parent
REALM, V = "e2e", "testvessel"
BASE = f"{REALM}/@v0/{V}/pubsub"
DOFS = ["rudder", "throttle"]


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def isolated(conf: zenoh.Config, **kv):
    conf.insert_json5("scouting/multicast/enabled", "false")
    conf.insert_json5("scouting/gossip/enabled", "false")
    for k, v in kv.items():
        conf.insert_json5(k, json.dumps(v))
    return conf


class Harness:
    def __init__(self, tmp: Path):
        self.tmp = tmp
        self.port = free_port()
        self.procs = {}
        # A ROUTER on loopback: two CLIs both connected to it as peers reach each other only through a
        # router (a peer does not forward, and gossip is off). Still loopback-only and scouting-free.
        self.session = zenoh.open(isolated(zenoh.Config(), mode="router", **{"listen/endpoints": [f"tcp/127.0.0.1:{self.port}"],
                                                                           "connect/endpoints": []}))
        self.zcfg = tmp / "zenoh.json5"
        self.zcfg.write_text(json.dumps({"mode": "client", "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
                                         "connect": {"endpoints": [f"tcp/127.0.0.1:{self.port}"]}, "listen": {"endpoints": []}}))

    def put(self, path, msg):
        self.session.put(f"{BASE}/{path}", keelson.enclose(msg.SerializeToString()))

    def clear(self):
        from keelson.payloads.MaintenanceState_pb2 import MaintenanceState

        self.put(f"safety_state/{V}", SafetyState(vessel_id=V, state=SafetyState.State.SAFETY_STATE_CLEAR))
        self.put(f"maintenance_state/{V}", MaintenanceState(vessel_id=V, state=MaintenanceState.State.STATE_INACTIVE))

    def start_gate(self, **extra):
        policy = self.tmp / "gate-policy.json"
        policy.write_text(json.dumps({"dofs": DOFS, "automation_sources": ["navigation/0"], **extra}))
        self._spawn("gate", [str(ROOT / "bin/control-gate2keelson.py"), "--state-dir", str(self.tmp / "gate-state"),
                             "--policy", str(policy)], "control-gate")

    def start_manager(self, bin_path):
        policy = self.tmp / "manager-policy.json"
        policy.write_text(json.dumps({"dofs": DOFS}))
        self._spawn("manager", [str(bin_path), "--state-dir", str(self.tmp / "manager-state"), "--policy", str(policy),
                                "--heartbeat-s", "1", "--gate-timeout-s", "2"], "authority-manager")

    def _spawn(self, name, argv, source):
        self.procs[name] = subprocess.Popen([sys.executable, *argv, "-r", REALM, "-e", V, "--zenoh-config", str(self.zcfg)],
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        token = keelson.construct_source_liveliness_key(REALM, V, source)
        self.wait(lambda: any(True for _ in self.session.liveliness().get(token, timeout=0.5)), what=f"{name}'s liveliness")

    def stop(self, name):
        p = self.procs.pop(name)
        p.terminate()
        return p.communicate(timeout=10)[0]

    def wait(self, pred, timeout=20.0, what="condition"):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for name, p in self.procs.items():
                if p.poll() is not None:
                    raise AssertionError(f"{name} exited:\n{p.stdout.read()}")
            if pred():
                return
            time.sleep(0.1)
        raise AssertionError(f"timed out waiting for {what}")

    def call(self, interface, responder, procedure, request, reply_cls):
        key = keelson.construct_rpc_key(REALM, V, interface, "v1", procedure, responder)
        for reply in self.session.get(key, payload=request.SerializeToString(), timeout=10):
            assert reply.ok is not None, reply.err.payload.to_bytes()
            out = reply_cls()
            out.ParseFromString(reply.ok.payload.to_bytes())
            return out
        raise AssertionError(f"no answer to {procedure}")

    def gate(self, procedure, request, reply_cls):
        return self.call("control_gate", "control-gate", procedure, request, reply_cls)

    def close(self):
        for name in list(self.procs):
            self.stop(name)
        self.session.close()


@pytest.fixture
def h(tmp_path):
    harness = Harness(tmp_path)
    yield harness
    harness.close()


def _state(h):
    from keelson.interfaces.ControlGate_pb2 import GetGateStateRequest

    return h.gate("get_gate_state", GetGateStateRequest(), GateState)


def test_an_abort_through_prepare_and_commit_and_the_fence_survives_a_restart(h):
    h.start_gate()
    st = _state(h)
    assert st.fence.generation == 1 and st.in_transition and len(st.fence.term) == 32
    ack = h.gate("prepare", TransferPrepare(transfer_id="a1", vessel_id=V, kind=TransferKind.TRANSFER_KIND_ABORT_AUTOMATION,
                                            prepare_ttl_ms=2000, requested_by="ted@ROC-1"), TransferAck)
    assert ack.accepted and ack.proposed_fence.generation == 2
    assert _state(h).reserved_transfer_id == "a1" and _state(h).fence.generation == 1  # an ACK changes nothing
    done = h.gate("commit", TransferCommit(transfer_id="a1", fence=ack.proposed_fence), CommitResponse)
    assert done.applied and done.fence.generation == 2
    term = done.fence.term
    h.stop("gate")
    h.start_gate()
    st = _state(h)
    assert (st.fence.term, st.fence.generation) == (term, 2)


def test_a_proposal_with_rows_needs_a_fresh_neutral_sample_from_each_source(h):
    h.start_gate()
    rows = [ControlMapping(dof=d, mapping_id=d, authority_holder="ted@ROC-1", authority_lease_id="t",
                           execution_source=f"ROC-1/helm-1/{d}") for d in DOFS]
    ack = h.gate("prepare", TransferPrepare(transfer_id="g1", vessel_id=V, kind=TransferKind.TRANSFER_KIND_GRANT, mappings=rows,
                                            active_control_station="ROC-1", prepare_ttl_ms=2000), TransferAck)
    assert not ack.accepted and set(ack.rejected_mapping_ids) == set(DOFS) and "no fresh sample" in ack.reason


def test_the_direct_abort_is_verified_on_the_gates_own_projection(h):
    h.start_gate()
    h.put(f"command_assignment/{V}", CommandAssignment(vessel_id=V, issuer_id="authority-manager/0", feed_ttl_seconds=15,
                                                       navigational=Assignment(assignee="anna@ROC-2", assignment_id="as-7", revision=4)))
    session = CommandAuthority(vessel_id=V, controller_id="anna@ROC-2", token="sess-a", lease_ttl_seconds=30,
                               role=Role.ROLE_NAVIGATIONAL_COMMAND, kind=RecordKind.RECORD_KIND_SESSION,
                               assignment=Assignment(assignee="anna@ROC-2", assignment_id="as-7", revision=4))
    h.put(f"command_authority/{V}/navigational/session", session)
    time.sleep(0.5)
    stale = h.gate("abort_automation", AbortAutomationRequest(requester_id="anna@ROC-2", assignment_id="as-7", assignment_revision=3,
                                                             session_token="sess-a"), AbortAutomationResponse)
    assert stale.result == CommandResult.COMMAND_RESULT_DENIED and "stale" in stale.detail
    ok = h.gate("abort_automation", AbortAutomationRequest(requester_id="anna@ROC-2", assignment_id="as-7", assignment_revision=4,
                                                          session_token="sess-a", reason="e2e"), AbortAutomationResponse)
    assert ok.result == CommandResult.COMMAND_RESULT_ACCEPTED and ok.fence.generation == 2


def test_the_manager_and_the_gate_as_two_processes(h):
    ca = pytest.importorskip("control_authority")
    manager_bin = Path(ca.__file__).resolve().parent.parent / "bin/authority-manager2keelson.py"
    if not manager_bin.exists():
        pytest.skip("the manager's CLI is not beside its package")
    from keelson.interfaces import AuthorityManager_pb2 as am

    h.start_gate()
    h.clear()
    h.start_manager(manager_bin)

    def manager_state():
        return h.call("authority_manager", "authority-manager", "get", am.GetAuthorityRequest(), am.AuthorityState)

    try:
        h.wait(lambda: manager_state().reconciled, what="the manager to reconcile with the real gate")
    except AssertionError:
        print(h.stop("manager"))
        raise
    gate = _state(h)
    st = manager_state()
    assert (st.fence.term, st.fence.generation) == (gate.fence.term, gate.fence.generation)
    # A PREPARE over the wire, judged by the real gate: no station has spoken, so nothing moves.
    rows = [ControlMapping(dof=d, execution_source=f"ROC-1/helm-1/{d}") for d in DOFS]
    r = h.call("authority_manager", "authority-manager", "request",
               am.RequestRoleRequest(caller=am.Caller(controller_id="ted@ROC-1", controller_site="ROC-1"), role=Role.ROLE_CONN,
                                     mappings=rows), am.AuthorityReply)
    assert (r.result, r.refusal) == (CommandResult.COMMAND_RESULT_DENIED, am.Refusal.REFUSAL_GATE)
    assert "no fresh sample" in r.detail and _state(h).fence.generation == gate.fence.generation


def test_a_helm_on_the_bus_steers_and_resolved_control_shows_it(h):
    """#272: a ControlCandidate on the bus reaches the gate; the gate's ResolvedControl is its answer."""
    from keelson.payloads.ControlCandidate_pb2 import ControlCandidate
    from keelson.payloads.GuidanceCandidate_pb2 import CandidateStream
    from keelson.payloads.ResolvedControl_pb2 import ResolvedControl

    resolved = []
    sub = h.session.declare_subscriber(f"{BASE}/resolved_control/{V}", lambda s: resolved.append(_dec(s, ResolvedControl)))
    try:
        h.start_gate()
        h.clear()
        h.put(f"command_authority/{V}", CommandAuthority(vessel_id=V, controller_id="ted@ROC-1", token="tok", lease_ttl_seconds=60,
                                                         role=Role.ROLE_CONN))
        seq = {"n": 0}

        def helm(dof, value):
            seq["n"] += 1
            h.put(f"control_candidate/helm-1/{dof}", ControlCandidate(source=f"ROC-1/helm-1/{dof}", dof=dof, value_pct=value,
                                                                      stream=CandidateStream(stream_id="s1", sequence=seq["n"])))

        for d in DOFS:
            helm(d, 0)
        time.sleep(0.3)
        rows = [ControlMapping(dof=d, mapping_id=d, authority_holder="ted@ROC-1", authority_lease_id="tok",
                               execution_source=f"ROC-1/helm-1/{d}") for d in DOFS]
        ack = h.gate("prepare", TransferPrepare(transfer_id="g1", vessel_id=V, kind=TransferKind.TRANSFER_KIND_GRANT, mappings=rows,
                                                active_control_station="ROC-1", prepare_ttl_ms=2000), TransferAck)
        assert ack.accepted, ack.reason
        assert h.gate("commit", TransferCommit(transfer_id="g1", fence=ack.proposed_fence), CommitResponse).applied
        helm("rudder", 15)

        def steering():
            if not resolved:
                return False
            d = {r.dof: r for r in resolved[-1].dofs}
            return d["rudder"].HasField("accepted_pct") and d["rudder"].accepted_pct == 15 and d["rudder"].control_state == 1
        h.wait(steering, what="ResolvedControl to show the helm's rudder")
        last = resolved[-1]
        assert last.actuation_allowed and last.motion_epoch.generation == ack.proposed_fence.generation and last.gate_id == "control-gate"
    finally:
        sub.undeclare()


def _dec(sample, cls):
    msg = cls()
    msg.ParseFromString(keelson.uncover(sample.payload.to_bytes())[2])
    return msg


def test_i16_an_engineer_handover_on_the_bus_never_lets_the_derate_slip(h):
    """crowsnest-dev#270: the conn drives the starboard throttle at 80 %, the engineer derates it to 40 %,
    and a new ENGINEER lease on the bus mints the next plant_generation here; the gate publishes the
    inherited constraint at revision 1, and no ResolvedControl after the derate shows more than 40 %."""
    import threading

    from keelson.payloads.ControlCandidate_pb2 import ControlCandidate
    from keelson.payloads.Fence_pb2 import Fence as PbFence
    from keelson.payloads.GuidanceCandidate_pb2 import CandidateStream
    from keelson.payloads.PropulsionConstraint_pb2 import PropulsionConstraint
    from keelson.payloads.ResolvedControl_pb2 import ResolvedControl

    resolved, constraints = [], []
    subs = [h.session.declare_subscriber(f"{BASE}/resolved_control/{V}", lambda s: resolved.append(_dec(s, ResolvedControl))),
            h.session.declare_subscriber(f"{BASE}/propulsion_constraint/{V}",
                                         lambda s: constraints.append(_dec(s, PropulsionConstraint)))]
    stop = threading.Event()
    seq = {"n": 0}

    def helm(dof, value):
        seq["n"] += 1
        h.put(f"control_candidate/helm-1/{dof}", ControlCandidate(source=f"ROC-1/helm-1/{dof}", dof=dof, value_pct=value,
                                                                  stream=CandidateStream(stream_id="s1", sequence=seq["n"])))

    def driving():
        while not stop.is_set():
            helm("throttle", 80)
            time.sleep(0.05)

    def engineer(who, token):
        h.put(f"command_authority/{V}/engineer", CommandAuthority(vessel_id=V, controller_id=who, controller_site="ECR", token=token,
                                                                  lease_ttl_seconds=60, role=Role.ROLE_ENGINEER))

    def throttle(rc):
        return next(d for d in rc.dofs if d.dof == "throttle")

    try:
        h.start_gate(propulsion={"throttle": "stbd"})
        h.clear()
        h.put(f"command_authority/{V}", CommandAuthority(vessel_id=V, controller_id="ted@ROC-1", token="tok", lease_ttl_seconds=60,
                                                         role=Role.ROLE_CONN))
        for d in DOFS:
            helm(d, 0)
        time.sleep(0.3)
        rows = [ControlMapping(dof=d, mapping_id=d, authority_holder="ted@ROC-1", authority_lease_id="tok",
                               execution_source=f"ROC-1/helm-1/{d}") for d in DOFS]
        ack = h.gate("prepare", TransferPrepare(transfer_id="g1", vessel_id=V, kind=TransferKind.TRANSFER_KIND_GRANT, mappings=rows,
                                                active_control_station="ROC-1", prepare_ttl_ms=2000), TransferAck)
        assert ack.accepted, ack.reason
        assert h.gate("commit", TransferCommit(transfer_id="g1", fence=ack.proposed_fence), CommitResponse).applied
        threading.Thread(target=driving, daemon=True).start()
        h.wait(lambda: resolved and throttle(resolved[-1]).accepted_pct == 80, what="the throttle at 80 %")

        engineer("anna@ECR", "eng-anna")  # the gate mints plant_generation 2 for her
        term = ack.proposed_fence.term
        time.sleep(0.5)
        h.put(f"propulsion_constraint/{V}", PropulsionConstraint(
            vessel_id=V, controller_id="anna@ECR", controller_site="ECR", fence=PbFence(term=term, generation=2), revision=1,
            devices=[PropulsionConstraint.DeviceConstraint(device_id="stbd", max_power_pct=40, reason="bearing temperature")]))
        h.wait(lambda: throttle(resolved[-1]).accepted_pct == 40, what="the derate to clamp the throttle")
        derated_from = len(resolved) - 1

        engineer("bo@ECR", "eng-bo")  # the handover: a token the gate has not seen
        h.wait(lambda: any(c.controller_id == "bo@ECR" for c in constraints), what="the inherited constraint at revision 1")
        c = next(c for c in constraints if c.controller_id == "bo@ECR")
        assert (c.fence.term, c.fence.generation, c.revision) == (term, 3, 1)
        assert [(d.device_id, d.max_power_pct) for d in c.devices] == [("stbd", 40)]
        # Anna, after the handover, under her generation: dropped.
        h.put(f"propulsion_constraint/{V}", PropulsionConstraint(
            vessel_id=V, controller_id="anna@ECR", fence=PbFence(term=term, generation=2), revision=2))
        time.sleep(1.0)
        after = [throttle(rc).accepted_pct for rc in resolved[derated_from:] if throttle(rc).HasField("accepted_pct")]
        assert after and max(after) <= 40
    finally:
        stop.set()
        for s in subs:
            s.undeclare()



def test_i16_a_reinitialised_gate_restores_the_stored_derate_and_waits_for_the_engineer(h):
    """crowsnest-dev#345: the gate's store is gone, and storage (the harness, answering the gate's read)
    holds a 40 % derate under the old term. The gate rebases it under its new term, publishes it, and
    holds ACTUATION_ALLOWED false until the ENGINEER writes under the new plant fence; no
    ResolvedControl ever shows the throttle above 40 %."""
    import threading

    from keelson.payloads.ControlCandidate_pb2 import ControlCandidate
    from keelson.payloads.Fence_pb2 import Fence as PbFence
    from keelson.payloads.GuidanceCandidate_pb2 import CandidateStream
    from keelson.payloads.PropulsionConstraint_pb2 import PropulsionConstraint
    from keelson.payloads.ResolvedControl_pb2 import ResolvedControl

    old = PropulsionConstraint(vessel_id=V, controller_id="anna@ECR", controller_site="ECR",
                               fence=PbFence(term="0123456789abcdef" * 2, generation=4), revision=3,
                               devices=[PropulsionConstraint.DeviceConstraint(device_id="stbd", max_power_pct=40,
                                                                              reason="bearing temperature")])
    storage = h.session.declare_queryable(f"{BASE}/propulsion_constraint/{V}",
                                          lambda q: q.reply(f"{BASE}/propulsion_constraint/{V}",
                                                            keelson.enclose(old.SerializeToString())))
    resolved, constraints = [], []
    subs = [h.session.declare_subscriber(f"{BASE}/resolved_control/{V}", lambda s: resolved.append(_dec(s, ResolvedControl))),
            h.session.declare_subscriber(f"{BASE}/propulsion_constraint/{V}",
                                         lambda s: constraints.append(_dec(s, PropulsionConstraint)))]
    stop = threading.Event()
    seq = {"n": 0}

    def helm(dof, value):
        seq["n"] += 1
        h.put(f"control_candidate/helm-1/{dof}", ControlCandidate(source=f"ROC-1/helm-1/{dof}", dof=dof, value_pct=value,
                                                                  stream=CandidateStream(stream_id="s1", sequence=seq["n"])))

    def driving():
        while not stop.is_set():
            helm("throttle", 80)
            time.sleep(0.05)

    def throttle(rc):
        return next(d for d in rc.dofs if d.dof == "throttle")

    try:
        h.start_gate(propulsion={"throttle": "stbd"})
        term = _state(h).fence.term
        h.wait(lambda: constraints, what="the restored constraint, published under the new term")
        c = constraints[0]
        assert (c.fence.term, c.fence.generation, c.revision) == (term, 1, 1) and term != old.fence.term
        assert [(d.device_id, d.max_power_pct) for d in c.devices] == [("stbd", 40)]
        h.clear()
        h.put(f"command_authority/{V}", CommandAuthority(vessel_id=V, controller_id="ted@ROC-1", token="tok", lease_ttl_seconds=60,
                                                         role=Role.ROLE_CONN))
        for d in DOFS:
            helm(d, 0)
        time.sleep(0.3)
        rows = [ControlMapping(dof=d, mapping_id=d, authority_holder="ted@ROC-1", authority_lease_id="tok",
                               execution_source=f"ROC-1/helm-1/{d}") for d in DOFS]
        ack = h.gate("prepare", TransferPrepare(transfer_id="g1", vessel_id=V, kind=TransferKind.TRANSFER_KIND_GRANT, mappings=rows,
                                                active_control_station="ROC-1", prepare_ttl_ms=2000), TransferAck)
        assert ack.accepted, ack.reason
        assert h.gate("commit", TransferCommit(transfer_id="g1", fence=ack.proposed_fence), CommitResponse).applied
        threading.Thread(target=driving, daemon=True).start()
        time.sleep(1.0)
        last = resolved[-1]
        assert not last.actuation_allowed and "awaiting the ENGINEER" in last.actuation_reason
        assert not throttle(last).HasField("accepted_pct")

        h.put(f"command_authority/{V}/engineer", CommandAuthority(vessel_id=V, controller_id="anna@ECR", controller_site="ECR",
                                                                  token="eng-anna", lease_ttl_seconds=60, role=Role.ROLE_ENGINEER))
        h.wait(lambda: any(c.fence.generation == 2 for c in constraints), what="the restore rebased onto her generation")
        h.put(f"propulsion_constraint/{V}", PropulsionConstraint(
            vessel_id=V, controller_id="anna@ECR", controller_site="ECR", fence=PbFence(term=term, generation=2), revision=2,
            devices=[PropulsionConstraint.DeviceConstraint(device_id="stbd", max_power_pct=40, reason="confirmed")]))
        h.wait(lambda: resolved[-1].actuation_allowed and throttle(resolved[-1]).accepted_pct == 40,
               what="her confirmation to lift the hold, clamped at 40 %")
        seen = [throttle(rc).accepted_pct for rc in resolved if throttle(rc).HasField("accepted_pct")]
        assert seen and max(seen) <= 40
    finally:
        stop.set()
        for s in subs:
            s.undeclare()
        storage.undeclare()

def test_maintenance_on_the_bus_inhibits_every_dof_and_inactive_releases_it(h):
    """#302: the manager's maintenance_state reaches the gate; anything but INACTIVE inhibits, and says so."""
    from keelson.payloads.MaintenanceState_pb2 import MaintenanceState
    from keelson.payloads.ResolvedControl_pb2 import ResolvedControl

    resolved = []
    sub = h.session.declare_subscriber(f"{BASE}/resolved_control/{V}", lambda s: resolved.append(_dec(s, ResolvedControl)))
    try:
        h.start_gate()
        h.put(f"safety_state/{V}", SafetyState(vessel_id=V, state=SafetyState.State.SAFETY_STATE_CLEAR))
        h.wait(lambda: resolved and "no maintenance_state heard" in resolved[-1].actuation_reason, what="the unheard record named")
        h.put(f"maintenance_state/{V}", MaintenanceState(vessel_id=V, state=MaintenanceState.State.STATE_MAINTENANCE))
        h.wait(lambda: "maintenance_state is MAINTENANCE" in resolved[-1].actuation_reason, what="MAINTENANCE named")
        assert not resolved[-1].actuation_allowed
        h.put(f"maintenance_state/{V}", MaintenanceState(vessel_id=V, state=MaintenanceState.State.STATE_INACTIVE))
        h.wait(lambda: resolved[-1].actuation_allowed, what="INACTIVE to allow actuation")
    finally:
        sub.undeclare()


# A keelson SDK without MaintenanceState, as the pin (0.6.0.dev21) is: the one module made unimportable.
_NO_MAINTENANCE = '''
import importlib.abc, sys
class _Block(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name == "keelson.payloads.MaintenanceState_pb2":
            raise ImportError("this SDK predates MaintenanceState")
sys.meta_path.insert(0, _Block())
'''


@pytest.mark.parametrize("required", [True, False])
def test_on_an_sdk_without_maintenance_state_the_gate_refuses_to_start_unless_told_not_to(h, tmp_path, required):
    import os

    site = tmp_path / "site"
    site.mkdir()
    (site / "sitecustomize.py").write_text(_NO_MAINTENANCE)
    policy = tmp_path / "p.json"
    policy.write_text(json.dumps({"dofs": DOFS, "require_maintenance_state": required}))
    env = {**os.environ, "PYTHONPATH": str(site)}
    p = subprocess.Popen([sys.executable, str(ROOT / "bin/control-gate2keelson.py"), "--state-dir", str(tmp_path / "s"),
                          "--policy", str(policy), "-r", REALM, "-e", V, "--zenoh-config", str(h.zcfg)],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        if required:
            out = p.communicate(timeout=20)[0]
            assert p.returncode == 2 and "no MaintenanceState" in out
        else:
            token = keelson.construct_source_liveliness_key(REALM, V, "control-gate")
            h.wait(lambda: p.poll() is None and any(True for _ in h.session.liveliness().get(token, timeout=0.5)),
                   what="the gate to start with the requirement off")
    finally:
        if p.poll() is None:
            p.terminate()
            p.communicate(timeout=10)
