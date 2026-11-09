"""The on-board start warning at the gate (crowsnest-dev#377, keelson #428): a start waits for it, and a
vessel already under way is never interrupted by it."""

from dataclasses import replace

import pytest

from control_gate import model
from control_gate.model import Proposal
from tests.test_gate import POL, TED, Helm, clear, clock, lease, make, ready, rows, take  # noqa: F401

UNSPECIFIED, NOT_FITTED, IDLE, SOUNDING, SATISFIED, FAULT = 0, 1, 2, 3, 4, 5


def prepared(tmp_path, clock, warning):  # noqa: F811
    g = make(tmp_path, clock)
    clear(g)
    g.observe_safety(model.SAFETY_CLEAR, False, warning)
    lease(g)
    Helm(g, clock).all(0)
    return g, g.prepare(Proposal("t1", model.GRANT, rows(), "ROC-1", 2000, TED))


@pytest.mark.parametrize("warning,name", [(UNSPECIFIED, "UNSPECIFIED"), (IDLE, "IDLE"), (SOUNDING, "SOUNDING"), (FAULT, "FAULT")])
def test_a_start_is_held_until_the_warning_has_run(tmp_path, clock, warning, name):  # noqa: F811
    g, ack = prepared(tmp_path, clock, warning)
    assert not ack.accepted and ack.reason == f"start held: the on-board warning is {name}"
    assert g.state.reservation is None


@pytest.mark.parametrize("warning", [SATISFIED, NOT_FITTED, None])
def test_a_completed_or_impossible_warning_lets_it_start(tmp_path, clock, warning):  # noqa: F811
    _, ack = prepared(tmp_path, clock, warning)
    assert ack.accepted, ack.reason


def test_a_held_start_goes_through_once_the_warning_has_run(tmp_path, clock):  # noqa: F811
    g, ack = prepared(tmp_path, clock, IDLE)
    assert not ack.accepted
    g.observe_safety(model.SAFETY_CLEAR, False, SATISFIED)
    ack, applied = take(g, clock)
    assert ack.accepted and applied


def test_a_warning_nobody_has_heard_holds(tmp_path, clock):  # noqa: F811
    g = make(tmp_path, clock)
    g.observe_maintenance(model.MAINTENANCE_INACTIVE)
    lease(g)
    Helm(g, clock).all(0)
    ack = g.prepare(Proposal("t1", model.GRANT, rows(), "ROC-1", 2000, TED))
    assert not ack.accepted and "no safety_state heard" in ack.reason


def test_a_stale_warning_holds_even_if_it_said_satisfied(tmp_path, clock):  # noqa: F811
    g = make(tmp_path, clock)
    clear(g)
    g.observe_safety(model.SAFETY_CLEAR, False, SATISFIED)
    clock.t += POL.safety_ttl_ms + 1
    assert "stale" in model.start_refusal(g.state, POL, clock())


def test_at_the_ttl_it_is_still_fresh(tmp_path, clock):  # noqa: F811
    g = make(tmp_path, clock)
    g.observe_safety(model.SAFETY_CLEAR, False, SATISFIED)
    clock.t += POL.safety_ttl_ms
    assert model.start_refusal(g.state, POL, clock()) is None


def test_a_rig_with_no_supervisor_is_not_held(tmp_path, clock):  # noqa: F811
    rig = replace(POL, require_safety_state=False)
    g = make(tmp_path, clock, policy=rig)
    assert model.start_refusal(g.state, rig, clock()) is None


def test_a_handover_under_way_is_not_a_start(tmp_path, clock):  # noqa: F811
    g = ready(tmp_path, clock)
    g.observe_safety(model.SAFETY_CLEAR, False, IDLE)
    assert g.state.mappings
    Helm(g, clock).all(0)
    ack = g.prepare(Proposal("t2", model.STATION_HANDOVER, rows(), "ROC-1", 2000, TED))
    assert ack.accepted, ack.reason


def test_it_never_interrupts_a_vessel_already_under_way(tmp_path, clock):  # noqa: F811
    g = ready(tmp_path, clock)
    for warning in (IDLE, SOUNDING, FAULT, UNSPECIFIED):
        g.observe_safety(model.SAFETY_CLEAR, False, warning)
        allowed, why = model.actuation_allowed(g.state, POL, clock())
        assert allowed, why


def test_an_abort_is_never_held(tmp_path, clock):  # noqa: F811
    g = make(tmp_path, clock)
    clear(g)
    g.observe_safety(model.SAFETY_CLEAR, False, FAULT)
    ack = g.prepare(Proposal("t1", model.ABORT_AUTOMATION, (), "ROC-1", 2000, TED))
    assert ack.accepted, ack.reason
