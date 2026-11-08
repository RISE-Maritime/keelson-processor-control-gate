"""I16 on a new term (crowsnest-dev#345, COMMAND-ARCHITECTURE §1.1): a reinitialised store restores the
plant constraint from storage, validated and rebased at revision 1 under the new term, clamps by it from
the first cycle, and holds ACTUATION_ALLOWED false until the live ENGINEER writes under the new plant
fence. No record is no constraint; an invalid one holds the same way; the hold is durable. No network."""

import math
from dataclasses import replace

import pytest

from control_gate import model
from control_gate.actuation import LoggingActuator
from control_gate.model import Constraint, DeviceLimit, Fence
from control_gate.service import Gate
from control_gate.store import Store
from test_gate import Clock, Helm, V, clear, lease, take
from test_plant import ANNA, BO, POL, derate, engineer, throttle

OLD = Fence("0123456789abcdef" * 2, 4)  # the previous life's term: gone with its store
DERATE = Constraint(OLD, 3, ANNA, "ECR", (DeviceLimit("stbd", 40.0, reason="bearing temperature"),))


@pytest.fixture
def clock():
    return Clock()


def reborn(tmp_path, clock, pol=POL):
    """A gate whose store was lost: it boots on a new term and has not yet read storage."""
    g = Gate(store=Store(tmp_path / "gate.sqlite"), policy=pol, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    g.boot()
    g.restore_motion([], [])  # no motion_constraint stored: this file is about the plant (#85 has its own)
    return g


def driving(g, clock, value=80):
    """Cleared, the conn's lease and map in force, the helm at `value` on the starboard throttle."""
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    assert take(g, clock)[1]
    h = Helm(g, clock)
    h.put("throttle", value)
    return h


def held(g, clock):
    return model.actuation_allowed(g.state, g.policy, clock())


def test_a_new_term_holds_actuation_until_storage_is_read(tmp_path, clock):
    g = reborn(tmp_path, clock)
    clear(g)
    assert held(g, clock) == (False, model.HOLD_UNREAD)
    # The engineer's record cannot be judged before storage is: it is not taken as a confirmation.
    engineer(g)
    assert derate(g, 90.0) == (False, "the stored plant constraint has not been read yet")
    assert g.state.plant_hold == model.HOLD_UNREAD


def test_i16_a_reinitialised_store_restores_the_derate_and_waits_for_the_engineer(tmp_path, clock):
    g = reborn(tmp_path, clock)
    assert g.restore_constraints([DERATE], []) == "restored"
    c = g.state.constraint
    assert c.fence == g.state.plant_fence and c.fence.term != OLD.term and c.revision == 1
    assert c.devices == DERATE.devices and c.controller_id == ANNA
    assert g.take_publications() == [c]  # bin/ publishes it, so storage holds it under the new term
    h = driving(g, clock)
    seen = [throttle(g)]
    assert seen[0].control_state == model.INHIBITED and seen[0].reason == model.HOLD_RESTORED
    # The ENGINEER is heard: the next plant_generation, the restored record rebased to her -- still held.
    engineer(g)
    seen.append(throttle(g))
    assert g.state.constraint.controller_id == ANNA and g.state.constraint.revision == 1
    assert seen[-1].control_state == model.INHIBITED
    # A record from anyone else, or under the old term, is no confirmation.
    assert not derate(g, 40.0, revision=2, holder=BO)[0]
    assert not g.observe_constraint(replace(DERATE, revision=9))[0]
    assert g.state.plant_hold == model.HOLD_RESTORED
    # Her own revision under the new plant fence is.
    assert derate(g, 40.0, revision=2) == (True, "")
    assert g.state.plant_hold == ""
    h.put("throttle", 90)
    seen.append(throttle(g))
    assert seen[-1].control_state == model.ACTIVE and seen[-1].value == 40
    assert all(r.value is None or abs(r.value) <= 40 for r in seen)


def test_the_restored_derate_clamps_from_the_first_cycle(tmp_path, clock):
    pol = replace(POL, transition_values={"throttle": 60.0})
    g = reborn(tmp_path, clock, pol)
    g.restore_constraints([DERATE], [])
    clear(g)
    first = {r.dof: r for r in g.cycle()}["throttle"]
    # Inhibited, so the DOF sits at its transition value -- and that is clamped, 60 → 40.
    assert first.control_state == model.INHIBITED and g.state.emitting["throttle"] == 40


def test_a_relaxing_record_the_old_gate_refused_still_waits(tmp_path, clock):
    """Storage holds the last record anybody PUT: here the outgoing engineer's 'no derate', which the
    old gate dropped. Restoring it alone would lift the derate; the hold is what stops that."""
    lifted = Constraint(OLD, 4, BO, "ECR", ())
    g = reborn(tmp_path, clock)
    assert g.restore_constraints([lifted], []) == "restored"
    driving(g, clock)
    assert throttle(g).control_state == model.INHIBITED
    assert held(g, clock) == (False, model.HOLD_RESTORED)


def test_no_stored_record_is_no_constraint(tmp_path, clock):
    g = reborn(tmp_path, clock)
    assert g.restore_constraints([], []) == "none"
    assert g.state.constraint is None and g.state.plant_hold == ""
    driving(g, clock)
    r = throttle(g)
    assert r.control_state == model.ACTIVE and r.value == 80
    assert g.take_publications() == []


@pytest.mark.parametrize("bad, why", [
    (replace(DERATE, fence=Fence("not-hex" * 4 + "abcd", 4)), "16 bytes of hex"),
    (replace(DERATE, fence=Fence("ab" * 8, 4)), "16 bytes of hex"),
    (replace(DERATE, revision=0), "revision 0"),
    (replace(DERATE, devices=(DeviceLimit("", 40.0),)), "no id"),
    (replace(DERATE, devices=(DeviceLimit("stbd", 40.0), DeviceLimit("stbd", 90.0))), "named twice"),
    (replace(DERATE, devices=(DeviceLimit("stbd", 150.0),)), "max_power_pct 150"),
    (replace(DERATE, devices=(DeviceLimit("stbd", -1.0),)), "max_power_pct -1"),
    (replace(DERATE, devices=(DeviceLimit("stbd", math.nan),)), "max_power_pct nan"),
])
def test_an_invalid_record_restores_nothing_and_holds(tmp_path, clock, bad, why):
    g = reborn(tmp_path, clock)
    assert g.restore_constraints([bad], []) == "invalid"
    assert g.state.constraint is None and why in g.state.plant_hold
    assert g.take_publications() == []
    h = driving(g, clock)
    assert throttle(g).control_state == model.INHIBITED
    # The engineer's own record lifts it, at any revision: there is none in force to be newer than.
    engineer(g)
    assert derate(g, 40.0, revision=1) == (True, "")
    h.put("throttle", 80)  # a sample refused under the hold is not remembered: the helm sends again
    assert throttle(g).value == 40 and g.state.plant_hold == ""


def test_an_unreadable_reply_holds_even_beside_a_good_one(tmp_path, clock):
    g = reborn(tmp_path, clock)
    assert g.restore_constraints([DERATE], ["undecodable: truncated"]) == "invalid"
    assert g.state.constraint is None and "undecodable: truncated" in g.state.plant_hold


def test_several_replies_restore_the_tightest(tmp_path, clock):
    other = Constraint(OLD, 2, BO, "ECR", (DeviceLimit("stbd", 30.0, reason="vibration"), DeviceLimit("port", None, True)))
    loose = Constraint(OLD, 5, BO, "ECR", (DeviceLimit("stbd", 90.0), DeviceLimit("port", 80.0)))
    g = reborn(tmp_path, clock)
    g.restore_constraints([DERATE, other, loose], [])
    devs = {d.device_id: d for d in g.state.constraint.devices}
    assert devs["stbd"].max_power_pct == 30 and not devs["stbd"].inhibit
    assert devs["stbd"].reason == "bearing temperature; vibration"
    assert devs["port"].inhibit and devs["port"].max_power_pct == 80  # one inhibit is enough


def test_the_writer_is_the_live_engineer_when_she_is_heard_first(tmp_path, clock):
    g = reborn(tmp_path, clock)
    engineer(g, BO, "eng-bo")
    g.restore_constraints([DERATE], [])
    c = g.state.constraint
    assert (c.controller_id, c.controller_site, c.fence) == (BO, "ECR", g.state.plant_fence)
    # And her next revision under it is the confirmation.
    assert derate(g, 40.0, revision=2, holder=BO)[0] and g.state.plant_hold == ""


def test_the_hold_survives_a_restart(tmp_path, clock):
    g = reborn(tmp_path, clock)
    g.restore_constraints([DERATE], [])
    want = (g.state.term, g.state.constraint, g.state.plant_hold)
    g.store.close()
    again = reborn(tmp_path, clock)
    assert (again.state.term, again.state.constraint, again.state.plant_hold) == want
    # The store survived, so this is not a new term: what storage holds is an ordinary record.
    assert again.restore_constraints([DERATE], []) == ""
    assert again.state.constraint == want[1] and held(again, clock)[0] is False
    engineer(again)
    assert derate(again, 40.0, revision=2)[0] and again.state.plant_hold == ""


def test_a_surviving_store_keeps_its_own_constraint_and_holds_nothing(tmp_path, clock):
    g = reborn(tmp_path, clock)
    g.restore_constraints([], [])
    engineer(g)
    assert derate(g, 40.0)[0]
    mine = g.state.constraint
    g.store.close()
    again = reborn(tmp_path, clock)
    # Storage's copy under the old life is refused as ever; the store's own is the one in force.
    assert again.restore_constraints([DERATE], []) == ""
    assert again.state.constraint == mine and again.state.plant_hold == ""
