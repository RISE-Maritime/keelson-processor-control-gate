"""The plant domain (crowsnest-dev#270, COMMAND-ARCHITECTURE §1.1, I16): a new ENGINEER holder's token
mints the next plant_generation here, the constraint in force is rebased onto it unchanged at revision
1, a constraint record applies only under the current fence at a newer revision from the live holder,
and every resolved propulsion value is clamped by it. No network."""

from dataclasses import replace

import pytest

from control_gate import model
from control_gate.actuation import LoggingActuator
from control_gate.model import Constraint, DeviceLimit
from control_gate.policy import parse
from control_gate.service import Gate
from control_gate.store import Store
from test_gate import DOFS, Clock, Helm, V, clear, lease, take

POL = parse({"dofs": DOFS, "automation_sources": ["navigation/0"], "transition": {"values": {"throttle": 0}},
             "propulsion": {"throttle": "stbd"}})
ANNA, BO = "anna@ECR", "bo@ECR"


@pytest.fixture
def clock():
    return Clock()


def make(tmp_path, clock):
    g = Gate(store=Store(tmp_path / "gate.sqlite"), policy=POL, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    g.boot()
    g.restore_constraints([], [])  # storage holds none (#345)
    g.restore_motion([], [])  # nor any motion_constraint (#85)
    return g


def engineer(g, holder=ANNA, token="eng-anna", released=False, ttl=60_000):
    g.observe_lease("engineer", holder, token, ttl, released, "ECR")


def derate(g, pct=40.0, revision=1, holder=ANNA, fence=None, **kw):
    lim = DeviceLimit("stbd", pct, reason="bearing temperature", **kw)
    return g.observe_constraint(Constraint(fence or g.state.plant_fence, revision, holder, "ECR", (lim,)))


def driving(tmp_path, clock, value=80):
    """The conn drives the starboard throttle at `value` while the engineer derates it to 40 %."""
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    assert take(g, clock)[1]
    engineer(g)
    assert derate(g) == (True, "")
    h = Helm(g, clock)
    h.put("throttle", value)
    return g, h


def throttle(g):
    return {r.dof: r for r in g.cycle()}["throttle"]


def test_a_derate_clamps_what_the_conn_orders(tmp_path, clock):
    g, _ = driving(tmp_path, clock)
    r = throttle(g)
    assert r.value == 40 and r.requested == 80 and r.control_state == model.ACTIVE
    assert "stbd 40 %" in r.reason and "bearing temperature" in r.reason


def test_astern_is_clamped_by_magnitude_and_an_inhibit_sits_at_zero(tmp_path, clock):
    g, h = driving(tmp_path, clock)
    h.put("throttle", -70)
    assert throttle(g).value == -40
    assert derate(g, None, revision=2, inhibit=True)[0]
    assert throttle(g).value == 0 and "inhibited" in throttle(g).reason


def test_the_transition_value_is_clamped_too(tmp_path, clock):
    pol = replace(POL, transition_values={"throttle": 60.0})
    g = Gate(store=Store(tmp_path / "gate.sqlite"), policy=pol, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    g.boot()
    g.restore_constraints([], [])  # storage holds none (#345)
    g.restore_motion([], [])  # nor any motion_constraint (#85)
    clear(g)
    engineer(g)
    derate(g)
    rows = {r.dof: r for r in g.cycle()}
    assert rows["throttle"].control_state == model.TRANSITION and rows["throttle"].value == 40


def test_i16_an_engineer_handover_mints_and_inherits_and_power_never_exceeds_the_derate(tmp_path, clock):
    g, h = driving(tmp_path, clock)
    n = g.state.plant_generation
    seen = [throttle(g).value]
    # The manager commits Bo's lease: a token the gate has not seen.
    engineer(g, BO, "eng-bo")
    seen.append(throttle(g).value)
    assert g.state.plant_generation == n + 1
    c = g.state.constraint
    assert c.fence == g.state.plant_fence and c.revision == 1 and c.controller_id == BO
    assert c.devices == (DeviceLimit("stbd", 40.0, reason="bearing temperature"),)
    assert g.take_publications() == [c]  # bin/ publishes the inherited record
    # Anna, too late, lifts it under the old generation: dropped.
    ok, why = derate(g, None, revision=9, holder=ANNA, fence=model.Fence(g.state.term, n))
    assert not ok and "plant_generation" in why
    h.put("throttle", 90)
    seen.append(throttle(g).value)
    # Bo cannot write revision 1 again; his own revision 2 is his to make.
    assert not derate(g, 30.0, revision=1, holder=BO)[0]
    assert derate(g, 30.0, revision=2, holder=BO)[0]
    seen.append(throttle(g).value)
    assert seen == [40, 40, 40, 30]
    # Only a newer revision lifts it.
    assert g.observe_constraint(Constraint(g.state.plant_fence, 3, BO, "ECR", ()))[0]
    assert throttle(g).value == 90


def test_the_motion_fence_does_not_move_with_the_plant(tmp_path, clock):
    g, _ = driving(tmp_path, clock)
    motion = g.state.fence
    engineer(g, BO, "eng-bo")
    g.observe_lease("payload/gimbal", "cy@ROC-1", "pay-cy", 5000, False)
    assert g.state.fence == motion and g.state.mappings


def test_only_a_new_engineer_token_mints(tmp_path, clock):
    g, _ = driving(tmp_path, clock)
    n = g.state.plant_generation
    engineer(g)  # a renewal: the same token
    lease(g)  # the conn renewed
    g.observe_lease("payload/gimbal", "cy@ROC-1", "pay-cy", 5000, False)
    engineer(g, released=True)  # a release lifts nothing and mints nothing
    assert g.state.plant_generation == n and throttle(g).value == 40


def test_silence_never_lifts_it(tmp_path, clock):
    g, _ = driving(tmp_path, clock)
    engineer(g, ttl=1000)
    clock.t += 5_000  # the engineer's lease lapses; the conn and the supervisor are still heard
    lease(g)
    clear(g)
    Helm(g, clock).put("throttle", 80)
    assert throttle(g).value == 40
    # ...and with no live ENGINEER holder nobody can write a newer revision.
    assert not derate(g, 90.0, revision=2)[0]


@pytest.mark.parametrize("change, why", [
    (dict(holder="mallory@ECR"), "does not hold a live ENGINEER lease"),
    (dict(revision=1), "not newer"),
    (dict(fence=model.Fence("f" * 32, 2)), "plant_generation"),
])
def test_a_constraint_applies_only_under_the_fence_newer_and_from_the_holder(tmp_path, clock, change, why):
    g, _ = driving(tmp_path, clock)
    ok, reason = derate(g, 90.0, **{"revision": 2, **change})
    assert not ok and why in reason and throttle(g).value == 40


def test_the_generation_and_the_constraint_survive_a_restart(tmp_path, clock):
    g, _ = driving(tmp_path, clock)
    engineer(g, BO, "eng-bo")
    want = (g.state.plant_generation, g.state.engineer_token, g.state.constraint)
    g.store.close()
    again = make(tmp_path, clock)
    assert (again.state.plant_generation, again.state.engineer_token, again.state.constraint) == want
    engineer(again, BO, "eng-bo")  # the same holder heard again after the restart: nothing new
    assert again.state.plant_generation == want[0]


def test_a_store_written_before_the_plant_domain_loads_without_a_new_term(tmp_path, clock):
    import json
    import sqlite3

    from control_gate.store import _canonical, _digest

    g = make(tmp_path, clock)
    term = g.state.term
    g.store.close()
    db = sqlite3.connect(tmp_path / "gate.sqlite")
    d = json.loads(db.execute("SELECT v FROM meta WHERE k='state'").fetchone()[0])
    for k in ("plant_generation", "engineer_token", "constraint"):
        d.pop(k)
    body = _canonical(d)
    db.executemany("UPDATE meta SET v=? WHERE k=?", [(body, "state"), (_digest(body), "state_hash")])
    db.commit()
    db.close()
    again = make(tmp_path, clock)
    assert again.state.term == term and again.state.plant_generation == 1 and again.state.constraint is None


def test_propulsion_names_only_the_vessels_dofs():
    with pytest.raises(ValueError, match="propulsion"):
        parse({"dofs": DOFS, "propulsion": {"bow_thruster": "bow-1"}})


def test_the_codec_round_trips_a_constraint_and_keeps_no_cap_apart_from_zero():
    from google.protobuf.timestamp_pb2 import Timestamp

    from control_gate import codec

    c = Constraint(model.Fence("a" * 32, 3), 2, BO, "ECR", (DeviceLimit("stbd", 0.0, reason="r"), DeviceLimit("port", None, True)))
    pb = codec.constraint_to_proto(c, V, Timestamp())
    assert pb.devices[0].HasField("max_power_pct") and not pb.devices[1].HasField("max_power_pct")
    assert codec.constraint_from_proto(pb) == c
