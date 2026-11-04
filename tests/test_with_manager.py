"""The real authority manager (keelson-processor-control-authority) against this gate, in one process:
the two halves of the transfer agree on the contract, not only each with its own fake. Skipped when
the manager is not installed in this venv (README, "Developing before the build")."""

import pytest

from control_gate import model as g
from control_gate.actuation import LoggingActuator
from control_gate.model import Sample
from control_gate.policy import parse
from control_gate.service import Gate
from control_gate.store import Store

ca = pytest.importorskip("control_authority")
from control_authority import model as m  # noqa: E402
from control_authority.policy import parse as manager_policy  # noqa: E402
from control_authority.service import GateAck, Manager  # noqa: E402
from control_authority.store import Store as ManagerStore  # noqa: E402

V = "usv-alpha"
DOFS = ["rudder", "throttle"]


class Clock:
    def __init__(self):
        self.t = 50_000

    def __call__(self):
        return self.t


class InProcess:
    """The manager's GatePort, onto this Gate: the same conversions bin/ does over control_gate/v1."""

    def __init__(self, gate):
        self.gate = gate

    def prepare(self, t, ttl):
        rows = tuple(g.Mapping(r.dof, r.authority_holder, r.authority_lease_id, r.execution_source, r.envelope, r.group_id,
                               r.mapping_id) for r in t.mappings)
        ack = self.gate.prepare(g.Proposal(t.transfer_id, t.kind, rows, t.station, ttl, t.requested_by))
        fence = None if ack.fence is None else m.Fence(ack.fence.term, ack.fence.generation)
        return GateAck(ack.accepted, fence, ack.reason, ack.rejected_mapping_ids)

    def commit(self, tid, fence):
        applied, reason, _ = self.gate.commit(tid, g.Fence(fence.term, fence.generation))
        return applied, reason

    def exposed(self):
        st = self.gate.exposed()
        rows = tuple(m.Mapping(r.dof, r.authority_holder, r.authority_lease_id, r.execution_source, r.envelope, r.group_id,
                               r.mapping_id) for r in st.mappings)
        return m.Fence(st.term, st.generation), rows, st.station


class Bus:
    """What the manager publishes, delivered to the gate the way bin/ subscribes it."""

    def __init__(self):
        self.gate = None

    def __call__(self, effect, st):
        if isinstance(effect, m.PublishLease) and effect.slot == m.CONN and self.gate is not None:
            lease = effect.lease if effect.released else st.leases.get(m.CONN)
            self.gate.observe_lease("conn", lease.holder, lease.token, lease.ttl_ms, effect.released)


@pytest.fixture
def pair(tmp_path):
    clock = Clock()
    gate = Gate(store=Store(tmp_path / "gate.sqlite"), policy=parse({"dofs": DOFS, "automation_sources": ["navigation/0"]}),
                vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    gate.boot()
    gate.observe_safety(g.SAFETY_CLEAR, False)
    bus = Bus()
    bus.gate = gate
    mgr = Manager(store=ManagerStore(tmp_path / "authority.sqlite"), policy=manager_policy({"dofs": DOFS}), vessel_id=V,
                  gate=InProcess(gate), publish=bus, now_ms=clock)
    mgr.boot()
    return mgr, gate, clock


def helm(gate, clock, station, value=0.0, seq=1):
    for d in DOFS:
        gate.sample(Sample(f"{station}/helm-1/{d}", d, value, "s1", seq, clock()))


def rows(station):
    return tuple(m.Mapping(d, "", "", f"{station}/helm-1/{d}") for d in DOFS)


def test_the_conn_is_taken_and_the_helm_steers_under_the_manager_s_lease(pair):
    mgr, gate, clock = pair
    helm(gate, clock, "ROC-1")
    d = mgr.transfer(m.TransferRequest(m.GRANT, "ted@ROC-1", "ROC-1", mappings=rows("ROC-1")))
    assert d.outcome == m.ACCEPTED, d.detail
    assert mgr.state.in_force.fence == m.Fence(gate.state.term, gate.state.generation)  # one fence, the gate's
    ok, why = gate.sample(Sample("ROC-1/helm-1/rudder", "rudder", 15, "s1", 2, clock()))
    assert ok, why  # the lease id on the mapping IS the token the manager published
    assert {r.dof: r.value for r in gate.cycle()}["rudder"] == 15


def test_an_unmatched_lever_naks_through_the_manager(pair):
    mgr, gate, clock = pair
    helm(gate, clock, "ROC-1", value=70)  # a new stream at 70 % is not even taken...
    d = mgr.transfer(m.TransferRequest(m.GRANT, "ted@ROC-1", "ROC-1", mappings=rows("ROC-1")))
    assert d.outcome == m.REFUSED_BY_GATE and gate.state.mappings == () and mgr.state.leases.get(m.CONN) is None


def test_the_manager_restarts_and_reconciles_with_a_gate_that_fenced_alone(pair, tmp_path):
    mgr, gate, clock = pair
    helm(gate, clock, "ROC-1")
    mgr.transfer(m.TransferRequest(m.GRANT, "ted@ROC-1", "ROC-1", mappings=rows("ROC-1")))
    mgr.store.close()
    clock.t += 2001  # the manager is dead: nobody renews the conn
    gate.observe_safety(g.SAFETY_CLEAR, False)
    gate.cycle()
    assert gate.state.in_transition
    mgr2 = Manager(store=ManagerStore(tmp_path / "authority.sqlite"), policy=manager_policy({"dofs": DOFS}), vessel_id=V,
                   gate=InProcess(gate), publish=Bus(), now_ms=clock)
    mgr2.boot()
    assert mgr2.state.in_force.fence == m.Fence(gate.state.term, gate.state.generation) and mgr2.state.in_force.mappings == ()
