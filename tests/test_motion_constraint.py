"""motion_constraint, the interim (crowsnest-dev#85; keelson#421, #422): one slot per payload; I11 under
one fence; a latch that outlives its writer and lifts only by a release matching what clears it (I5);
nothing relaxed under a payload_epoch this gate cannot judge current; a latched slot with a limit
refuses ACTUATION_ALLOWED until the clamp exists; and I16 -- storage read once on a new term, an invalid
record holding until a valid one for that payload. No network."""

import math

from control_gate import model
from control_gate.actuation import LoggingActuator
from control_gate.model import Fence, MotionLimit
from control_gate.service import Gate
from control_gate.store import Store
from test_gate import POL, V, Clock, clear, make

EPOCH = Fence("a" * 32, 3)  # the sonar's payload gate's payload_epoch
OTHER = Fence("b" * 32, 1)  # a payload_epoch this gate cannot judge current (keelson#421)
SONAR = "towed-sonar"
STOWED = ("sensor", "rise/@v0/usv-alpha/pubsub/winch_stowed/winch")
RECOVERY = ("procedure", "proc_recover_sonar")


def sonar(rev=1, fence=EPOCH, speed=4.0, rot=1.0, band=None, latched=True, clears=STOWED, release=None, pid=SONAR):
    return MotionLimit(pid, fence, rev, "payload-gate/sonar", band, speed, rot, "towed sonar deployed", latched,
                       clears, release)


def refusal(g):
    return model.motion_refusal(g.state)


# --------------------------------------------------------------------------- I5: the latch outlives its writer


def test_a_latched_constraint_refuses_actuation_and_says_why(tmp_path):
    clock = Clock()
    g = make(tmp_path, clock)
    clear(g)
    assert model.actuation_allowed(g.state, POL, clock()) == (True, "")  # nothing else is refusing
    assert g.observe_motion(sonar()) == (True, "")
    ok, why = model.actuation_allowed(g.state, POL, clock())
    assert not ok and "latched motion_constraint towed-sonar" in why and "keelson#422" in why


def test_silence_and_time_lift_nothing(tmp_path):
    clock = Clock()
    g = make(tmp_path, clock)
    g.observe_motion(sonar())
    clock.t += 6 * 3600 * 1000  # six hours of a dead payload gate
    g.cycle()
    assert "latched motion_constraint" in refusal(g)


def test_a_non_latched_limit_is_reported_not_enforced(tmp_path):
    g = make(tmp_path, Clock())
    assert g.observe_motion(sonar(latched=False, clears=None)) == (True, "")
    assert refusal(g) == ""


def test_a_latched_record_with_no_limit_refuses_nothing(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_motion(sonar(speed=None, rot=None))
    assert refusal(g) == ""


# --------------------------------------------------------------------------- I11 under one fence


def test_revisions_5_7_6_apply_5_then_7_and_drop_6(tmp_path):
    g = make(tmp_path, Clock())
    assert g.observe_motion(sonar(5, speed=4.0))[0]
    assert g.observe_motion(sonar(7, speed=3.0))[0]
    ok, why = g.observe_motion(sonar(6, speed=3.5))
    assert not ok and "6 is not newer than 7" in why
    assert not g.observe_motion(sonar(7, speed=2.0))[0]  # the same revision again is not newer either
    assert g.state.motion[SONAR].max_speed_knots == 3.0


# --------------------------------------------------------------------------- lifting a latch


def test_a_latch_is_not_lifted_without_evidence(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_motion(sonar(1))
    ok, why = g.observe_motion(sonar(2, latched=False, speed=None, rot=None))
    assert not ok and "release matching" in why
    ok, _ = g.observe_motion(sonar(3, speed=8.0))  # still latched, but looser: a relaxation, also refused
    assert not ok


def test_a_release_of_the_wrong_kind_or_procedure_does_not_lift_it(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_motion(sonar(1, clears=RECOVERY))
    assert not g.observe_motion(sonar(2, latched=False, release=("sensor", "true")))[0]
    assert not g.observe_motion(sonar(3, latched=False, release=("procedure", "proc_something_else")))[0]
    assert "latched" in refusal(g)


def test_the_matching_release_lifts_it(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_motion(sonar(1))
    assert g.observe_motion(sonar(2, latched=False, speed=None, rot=None, release=("sensor", "true"))) == (True, "")
    assert refusal(g) == ""
    g2 = make(tmp_path / "b", Clock())
    g2.observe_motion(sonar(1, clears=RECOVERY))
    assert g2.observe_motion(sonar(2, latched=False, speed=None, rot=None, release=RECOVERY))[0]


# --------------------------------------------------------------------------- a fence this gate cannot judge (keelson#421)


def test_under_another_payload_epoch_a_tightening_applies(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_motion(sonar(5))
    assert g.observe_motion(sonar(1, fence=OTHER, speed=2.0)) == (True, "")
    assert g.state.motion[SONAR].fence == OTHER


def test_under_another_payload_epoch_nothing_is_relaxed(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_motion(sonar(5))
    ok, why = g.observe_motion(sonar(9, fence=OTHER, speed=6.0))
    assert not ok and "keelson#421" in why
    # Not even a matching release: a dead incarnation's release must not lift today's latch.
    assert not g.observe_motion(sonar(9, fence=OTHER, latched=False, release=("sensor", "true")))[0]
    assert g.state.motion[SONAR].fence == EPOCH


def test_a_heading_band_is_relaxed_when_widened_even_through_north(tmp_path):
    cur = sonar(band=(350.0, 20.0))
    assert model.motion_relaxes(cur, sonar(band=(355.0, 10.0))) == ""
    assert model.motion_relaxes(cur, sonar(band=(10.0, 15.0))) == ""  # inside, past north
    assert model.motion_relaxes(cur, sonar(band=(340.0, 20.0))) == "the heading band widened"
    assert model.motion_relaxes(cur, sonar(band=None)) == "the heading band widened"


def test_an_invalid_record_is_dropped(tmp_path):
    g = make(tmp_path, Clock())
    assert "invalid" in g.observe_motion(sonar(speed=math.nan))[1]
    assert "invalid" in g.observe_motion(sonar(fence=Fence("xyz", 1)))[1]
    assert "invalid" in g.observe_motion(sonar(latched=True, clears=None))[1]
    assert g.state.motion == {}


# --------------------------------------------------------------------------- I16 on a new term


def reborn(tmp_path, clock):
    g = Gate(store=Store(tmp_path / "gate.sqlite"), policy=POL, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    g.boot()
    g.restore_constraints([], [])
    return g


def test_a_new_term_holds_until_storage_is_read(tmp_path):
    g = reborn(tmp_path, Clock())
    assert refusal(g) == model.MOTION_UNREAD
    assert not g.observe_motion(sonar())[0]  # cannot be judged against what it may relax
    assert g.restore_motion([], []) == "none"
    assert refusal(g) == ""


def test_a_stored_latch_is_restored_and_keeps_refusing(tmp_path):
    g = reborn(tmp_path, Clock())
    assert g.restore_motion([sonar(4)], []) == "restored"
    assert "latched motion_constraint" in refusal(g)
    assert g.state.motion[SONAR].fence == EPOCH  # the payload gate's fence, not rebased into this term


def test_an_invalid_stored_record_holds_until_a_valid_one_for_it(tmp_path):
    g = reborn(tmp_path, Clock())
    assert g.restore_motion([], [(SONAR, "undecodable: truncated")]) == "invalid"
    assert "awaiting a valid record" in refusal(g)
    g.observe_motion(sonar(1, pid="a-frame", latched=False, clears=None))  # another payload lifts nothing
    assert "awaiting a valid record" in refusal(g)
    assert g.observe_motion(sonar(1, latched=False, clears=None))[0]
    assert refusal(g) == ""


def test_an_unattributable_stored_record_is_lifted_by_any_valid_one(tmp_path):
    g = reborn(tmp_path, Clock())
    g.restore_motion([], [("?", "the storage read failed: timeout")])
    assert refusal(g)
    g.observe_motion(sonar(1, latched=False, clears=None))
    assert refusal(g) == ""


def test_a_latched_slot_survives_the_gates_restart(tmp_path):
    clock = Clock()
    g = make(tmp_path, clock)
    g.observe_motion(sonar(3))
    g.store.close()
    again = Gate(store=Store(tmp_path / "gate.sqlite"), policy=POL, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    again.boot()
    assert again.state.motion[SONAR].latched and again.state.motion[SONAR].revision == 3
    assert "latched motion_constraint" in refusal(again)
    assert again.restore_motion([sonar(2)], []) == ""  # a surviving store: storage's older record is judged, and dropped
    assert again.state.motion[SONAR].revision == 3
