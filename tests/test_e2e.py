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
        self.put(f"safety_state/{V}", SafetyState(vessel_id=V, state=SafetyState.State.SAFETY_STATE_CLEAR))

    def start_gate(self):
        policy = self.tmp / "gate-policy.json"
        policy.write_text(json.dumps({"dofs": DOFS, "automation_sources": ["navigation/0"]}))
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
