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
        c = st.last_cause
        cause = None if c is None else m.GateCause(c.kind, m.Fence(c.fence.term, c.fence.generation), c.requested_by,
                                                   c.assignment_id, c.assignment_revision, c.reason)
        return m.Fence(st.term, st.generation), rows, st.station, cause


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


# ---------------------------------------------------------------------------- crowsnest-dev#270: the plant and the payload


class AllLeases(Bus):
    """Every lease the manager publishes, to the gate as bin/ delivers it (conn, engineer, payload)."""

    def __call__(self, effect, st):
        if isinstance(effect, m.PublishLease) and self.gate is not None:
            lease = effect.lease if effect.released else st.leases.get(effect.slot)
            self.gate.observe_lease(effect.slot, lease.holder, lease.token, lease.ttl_ms, effect.released, lease.site)


def _heard(c):
    """The gate's rebased constraint, as the manager hears it on propulsion_constraint."""
    return m.HeardConstraint(m.Fence(c.fence.term, c.fence.generation), c.revision, c.controller_id,
                             tuple((d.device_id, d.max_power_pct, d.inhibit, d.reason) for d in c.devices))


@pytest.fixture
def plant(tmp_path):
    clock = Clock()
    gate = Gate(store=Store(tmp_path / "gate.sqlite"), policy=parse({"dofs": DOFS, "automation_sources": ["navigation/0"],
                                                                     "propulsion": {"throttle": "stbd"}}),
                vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    gate.boot()
    gate.observe_safety(g.SAFETY_CLEAR, False)
    bus = AllLeases()
    bus.gate = gate
    mgr = Manager(store=ManagerStore(tmp_path / "authority.sqlite"), policy=manager_policy({"dofs": DOFS}), vessel_id=V,
                  gate=InProcess(gate), publish=bus, now_ms=clock)
    mgr.boot()
    helm(gate, clock, "ROC-1")
    assert mgr.transfer(m.TransferRequest(m.GRANT, "ted@ROC-1", "ROC-1", mappings=rows("ROC-1"))).outcome == m.ACCEPTED
    return mgr, gate, clock


def test_i16_an_engineer_handover_through_the_manager_inherits_the_derate_and_power_stays_under_it(plant):
    mgr, gate, clock = plant
    assert mgr.take(m.ENGINEER, "anna@ECR", "ECR").outcome == m.ACCEPTED
    n = gate.state.plant_generation
    assert n == 2  # the gate minted for Anna's token
    ok, why = gate.observe_constraint(g.Constraint(gate.state.plant_fence, 1, "anna@ECR", "ECR",
                                                   (g.DeviceLimit("stbd", 40.0, reason="bearing temperature"),)))
    assert ok, why
    mgr.observe_constraint(_heard(gate.state.constraint))
    assert mgr.state.leases[m.slot_of(m.ENGINEER)].fence == m.Fence(gate.state.term, n)
    gate.sample(Sample("ROC-1/helm-1/throttle", "throttle", 80, "s1", 2, clock()))
    out = [{r.dof: r.value for r in gate.cycle()}["throttle"]]

    anna = mgr.state.leases[m.slot_of(m.ENGINEER)]
    assert mgr.offer(m.ENGINEER, "anna@ECR", "", anna.token, "bo@ECR", "ECR").outcome == m.ACCEPTED
    out.append({r.dof: r.value for r in gate.cycle()}["throttle"])
    assert gate.state.plant_generation == n  # an offer moves nothing
    d = mgr.take(m.ENGINEER, "bo@ECR", "ECR")
    assert d.outcome == m.ACCEPTED and "inherits stbd 40 %" in d.detail
    assert gate.state.plant_generation == n + 1  # the new token, observed
    out.append({r.dof: r.value for r in gate.cycle()}["throttle"])
    (rebased,) = gate.take_publications()
    assert (rebased.revision, rebased.controller_id, rebased.devices) == (1, "bo@ECR", gate.state.constraint.devices)
    mgr.observe_constraint(_heard(rebased))
    assert mgr.state.leases[m.slot_of(m.ENGINEER)].fence == m.Fence(gate.state.term, n + 1)
    assert out == [40, 40, 40]
    assert gate.state.fence.generation == mgr.state.in_force.fence.generation  # the helm's fence never moved


def test_a_payload_change_of_hands_fences_the_payload_and_not_the_helm(plant):
    mgr, gate, clock = plant
    motion, mapped = gate.state.fence, gate.state.mappings
    mgr.take(m.PAYLOAD, "cy@ROC-1", "ROC-1", "gimbal")
    cy = mgr.state.leases[m.slot_of(m.PAYLOAD, "gimbal")]
    mgr.offer(m.PAYLOAD, "cy@ROC-1", "gimbal", cy.token, "bo@ECR", "ECR")
    assert mgr.take(m.PAYLOAD, "bo@ECR", "ECR", "gimbal").outcome == m.ACCEPTED
    gate.cycle()
    assert gate.state.fence == motion and gate.state.mappings == mapped and not gate.state.in_transition
    assert gate.state.plant_generation == 1


# ---------------------------------------------------------------------------- crowsnest-dev#290


def test_an_abort_the_gate_ran_alone_is_completed_by_the_manager_at_reconcile(tmp_path):
    """The real gate's direct abort, with the manager down; a fresh manager boots against it and takes the
    navigator's functions back to the human who asked, at method 2, once."""
    clock = Clock()
    gate = Gate(store=Store(tmp_path / "gate.sqlite"), policy=parse({"dofs": DOFS, "automation_sources": ["navigation/0"]}),
                vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    gate.boot()
    gate.observe_safety(g.SAFETY_CLEAR, False)
    bus = AllLeases()
    bus.gate = gate
    pol = manager_policy({"dofs": DOFS, "automation_sources": ["navigation/0"], "processes": ["navigation/0@usv-alpha"]})
    mgr = Manager(store=ManagerStore(tmp_path / "authority.sqlite"), policy=pol, vessel_id=V, gate=InProcess(gate),
                  publish=bus, now_ms=clock)
    mgr.boot()
    mgr.assign(m.OVERALL, "ted@ROC-1", "procedure")
    mgr.session(m.OVERALL, "ted@ROC-1", "ROC-1", "sess-1", mgr.state.overall.assignment_id)
    mgr.assign(m.NAVIGATIONAL, "anna@ROC-2", "ted@ROC-1", mgr.state.overall.revision, "sess-1")
    nav = mgr.state.navigational
    mgr.session(m.NAVIGATIONAL, "anna@ROC-2", "ROC-2", "sess-2", nav.assignment_id)
    helm(gate, clock, "ROC-1")
    assert mgr.transfer(m.TransferRequest(m.GRANT, "ted@ROC-1", "ROC-1", mappings=rows("ROC-1"))).outcome == m.ACCEPTED
    for f, method in (("collision_avoidance", 3), ("track_keeping", 4)):
        assert mgr.set_navigation_function(f, m.NavFunction("navigation/0@usv-alpha", "nav-live", method), "anna@ROC-2",
                                           nav.revision, "sess-2").outcome == m.ACCEPTED
    rev = mgr.state.navigation_revision

    # The manager dies. Anna aborts automation at the gate directly; the gate verifies her on its own projection.
    mgr.store.close()
    gate.observe_assignments(None, g.AssignmentSeen("anna@ROC-2", nav.assignment_id, nav.revision), "authority-manager/0", 15000)
    gate.observe_session(g.NAVIGATIONAL, "anna@ROC-2", "sess-2", nav.assignment_id, 30000, False)
    ok, why, fence = gate.abort_direct(g.DirectAbort("anna@ROC-2", nav.assignment_id, nav.revision, "sess-2", "manager down"))
    assert ok, why

    mgr2 = Manager(store=ManagerStore(tmp_path / "authority.sqlite"), policy=pol, vessel_id=V, gate=InProcess(gate),
                   publish=AllLeases(), now_ms=clock)
    mgr2.boot()
    assert mgr2.state.in_force.fence == m.Fence(fence.term, fence.generation)
    for f in ("collision_avoidance", "track_keeping"):
        fn = mgr2.state.navigation[f]
        assert (fn.holder, fn.method, fn.holder_lease_id) == ("anna@ROC-2", 2, "sess-2")
    assert mgr2.state.navigation_revision == rev + 1
