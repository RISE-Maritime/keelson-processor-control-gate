"""Stored control-plane state is applied only under the current fence and at a newer revision (I11,
crowsnest-dev#85 slice 6; COMMAND-ARCHITECTURE §0). propulsion_constraint has done so since #270
(test_plant.py); this pins the navigation authority document, which has NO motion fence: a revision is
judged within its incarnation (`assignment_id`), and the floor survives a restart. No network."""

from control_gate.service import Gate
from control_gate.store import Store
from control_gate.actuation import LoggingActuator
from test_gate import POL, V, Clock, make

NAV = "navigation/0@usv-alpha"


def doc(method):
    return {"track_keeping": (NAV, method)}


def test_revisions_5_7_6_apply_5_then_7_and_drop_6(tmp_path):
    g = make(tmp_path, Clock())
    assert g.observe_navigation(5, doc(3), "a-1") == (True, "")
    assert g.observe_navigation(7, doc(4), "a-1") == (True, "")
    ok, why = g.observe_navigation(6, doc(1), "a-1")
    assert not ok and "6 is not newer than 7" in why
    assert g.state.navigation.revision == 7 and g.state.navigation.functions == doc(4)


def test_the_same_revision_again_is_not_newer(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_navigation(7, doc(4), "a-1")
    assert g.observe_navigation(7, doc(1), "a-1")[0] is False
    assert g.state.navigation.functions == doc(4)


def test_a_reissued_document_is_a_new_incarnation_with_its_own_revisions(tmp_path):
    g = make(tmp_path, Clock())
    g.observe_navigation(9, doc(4), "a-1")
    assert g.observe_navigation(1, doc(2), "a-2") == (True, "")
    assert g.state.navigation.assignment_id == "a-2" and g.state.navigation.revision == 1
    # A late document from the earlier incarnation is judged against ITS floor, not the new one's.
    assert g.observe_navigation(8, doc(5), "a-1")[0] is False


def test_the_floor_survives_a_restart(tmp_path):
    clock = Clock()
    g = make(tmp_path, clock)
    g.observe_navigation(7, doc(4), "a-1")
    g.store.close()
    again = Gate(store=Store(tmp_path / "gate.sqlite"), policy=POL, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    again.boot()
    assert again.state.nav_floors == {"a-1": 7}
    assert again.observe_navigation(6, doc(1), "a-1")[0] is False  # storage replaying an older document
    assert again.observe_navigation(8, doc(3), "a-1") == (True, "")


def test_a_lost_store_is_a_new_term_and_takes_the_document_afresh(tmp_path):
    clock = Clock()
    g = make(tmp_path, clock)
    g.observe_navigation(7, doc(4), "a-1")
    g.store.close()
    (tmp_path / "gate.sqlite").unlink()
    for side in ("gate.sqlite-wal", "gate.sqlite-shm"):
        (tmp_path / side).unlink(missing_ok=True)
    again = make(tmp_path, clock)
    assert again.state.nav_floors == {}
    assert again.observe_navigation(7, doc(4), "a-1") == (True, "")
