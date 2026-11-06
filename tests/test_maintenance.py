"""ACTUATION_ALLOWED needs maintenance_state INACTIVE, heard fresh (crowsnest-dev#302, keelson#377).

Every other state of the procedure, a stale record and an unheard one inhibit every DOF, with a reason
that names why. Off only by policy, in a rig where nothing publishes it. No network."""

import pytest

from control_gate import model
from control_gate.policy import parse
from test_gate import DOFS, POL, Clock, Helm, lease, make, ready


@pytest.fixture
def clock():
    return Clock()


def _inhibited(g):
    out = {r.dof: r for r in g.cycle()}
    assert set(out) == set(DOFS)
    return out


def test_inactive_and_fresh_lets_her_move(tmp_path, clock):
    g = ready(tmp_path, clock)
    assert Helm(g, clock).put("rudder", 10)[0]
    out = {r.dof: r for r in g.cycle()}
    assert out["rudder"].control_state == model.ACTIVE and out["rudder"].value == 10
    assert g.last_cycle.actuation_allowed


@pytest.mark.parametrize("state, name", [(0, "UNSPECIFIED"), (2, "REQUESTED"), (3, "PREPARING"), (4, "MAINTENANCE"),
                                         (5, "READY_FOR_CHECK"), (9, "state 9")])
def test_every_other_state_inhibits_every_dof_and_names_itself(tmp_path, clock, state, name):
    g = ready(tmp_path, clock)
    assert Helm(g, clock).put("rudder", 10)[0]
    g.observe_maintenance(state)
    ok, why = Helm(g, clock).put("rudder", 12)
    assert not ok and name in why
    for r in _inhibited(g).values():
        assert r.control_state == model.INHIBITED and r.value is None and f"maintenance_state is {name}" in r.reason
    assert not g.last_cycle.actuation_allowed and name in g.last_cycle.actuation_reason


def test_a_record_heard_too_long_ago_is_not_inactive(tmp_path, clock):
    g = ready(tmp_path, clock)
    clock.t += POL.maintenance_ttl_ms  # exactly the window: still fresh
    g.observe_safety(model.SAFETY_CLEAR, False)
    assert model.actuation_allowed(g.state, POL, clock())[0]
    clock.t += 1
    g.observe_safety(model.SAFETY_CLEAR, False)  # the supervisor is alive; the manager went quiet
    lease(g)
    for r in _inhibited(g).values():
        assert r.control_state == model.INHIBITED and r.reason == "maintenance_state is stale"
    g.observe_maintenance(model.MAINTENANCE_INACTIVE)  # its heartbeat again
    assert model.actuation_allowed(g.state, POL, clock())[0]


def test_an_unheard_record_is_not_inactive(tmp_path, clock):
    g = make(tmp_path, clock)
    g.observe_safety(model.SAFETY_CLEAR, False)
    for r in _inhibited(g).values():
        assert r.control_state == model.INHIBITED and r.reason == "no maintenance_state heard"


def test_the_safety_state_still_speaks_first(tmp_path, clock):
    """A STOPPED vessel says so, whatever the procedure: the supervisor's reason is the one an operator needs."""
    g = make(tmp_path, clock)
    g.observe_safety(2, False)
    assert model.actuation_allowed(g.state, POL, clock()) == (False, "the vessel is not CLEAR")


def test_it_binds_in_a_rig_with_no_supervisor(tmp_path, clock):
    """`require_safety_state` false does not switch the maintenance requirement off with it."""
    pol = parse({"dofs": DOFS, "require_safety_state": False})
    g = make(tmp_path, clock, policy=pol)
    assert model.actuation_allowed(g.state, pol, clock()) == (False, "no maintenance_state heard")
    g.observe_maintenance(model.MAINTENANCE_INACTIVE)
    assert model.actuation_allowed(g.state, pol, clock()) == (True, "")


def test_off_by_policy_it_is_not_asked_for_and_the_lockout_still_binds(tmp_path, clock):
    pol = parse({"dofs": DOFS, "require_maintenance_state": False})
    g = make(tmp_path, clock, policy=pol)
    g.observe_safety(model.SAFETY_CLEAR, False)
    assert model.actuation_allowed(g.state, pol, clock()) == (True, "")
    g.observe_safety(model.SAFETY_CLEAR, True)
    assert not model.actuation_allowed(g.state, pol, clock())[0]


def test_the_policy_defaults_to_required_with_a_fifteen_second_window():
    pol = parse({"dofs": DOFS})
    assert pol.require_maintenance_state is True and pol.maintenance_ttl_ms == 15000
    assert parse({"dofs": DOFS, "maintenance_ttl_ms": 4000}).maintenance_ttl_ms == 4000


def test_a_restarted_gate_waits_to_hear_it_again(tmp_path, clock):
    """Volatile like safety_state: an INACTIVE heard before a restart proves nothing about the manager now."""
    g = ready(tmp_path, clock)
    g2 = make(tmp_path, clock)
    g2.observe_safety(model.SAFETY_CLEAR, False)
    assert g.state.maintenance is not None and g2.state.maintenance is None
    assert model.actuation_allowed(g2.state, POL, clock()) == (False, "no maintenance_state heard")
