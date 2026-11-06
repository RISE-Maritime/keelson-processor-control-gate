"""Why the gate last fenced alone (GateState.last_fence_cause, crowsnest-dev#290): a direct abort names who
asked, a CONN lapse names nobody, a COMMIT clears it, and it survives a restart. No network."""

import pytest

from control_gate import codec, model
from control_gate.model import DirectAbort
from test_gate import Clock, Helm, clear, lease, make, ready, take
from test_gate import _projection


@pytest.fixture
def clock():
    return Clock()


def test_a_direct_abort_records_who_asked_and_the_fence_it_produced(tmp_path, clock):
    g = ready(tmp_path, clock)
    _projection(g, clock)
    ok, _, fence = g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a", "no answer from the navigator"))
    c = g.state.last_cause
    assert ok and c.kind == model.CAUSE_ABORT and c.fence == fence
    assert (c.requested_by, c.assignment_id, c.assignment_revision) == ("anna@ROC-2", "as-7", 4)
    assert "no answer from the navigator" in c.reason


def test_a_refused_direct_abort_records_nothing(tmp_path, clock):
    g = ready(tmp_path, clock)
    _projection(g, clock)
    assert not g.abort_direct(DirectAbort("bo@ROC-2", "as-7", 4, "sess-a"))[0]
    assert g.state.last_cause is None


def test_a_conn_lapse_records_a_lapse_and_names_nobody(tmp_path, clock):
    g = ready(tmp_path, clock)
    clock.t += 5000  # the conn lease (2 s) lapses
    clear(g)
    g.cycle()
    c = g.state.last_cause
    assert c.kind == model.CAUSE_CONN_LAPSE and c.fence == g.state.fence and c.requested_by == ""


def test_a_commit_clears_it(tmp_path, clock):
    g = ready(tmp_path, clock)
    _projection(g, clock)
    g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a"))
    assert g.state.last_cause is not None
    lease(g)
    Helm(g, clock).all(0)
    assert take(g, clock, tid="t2")[1]
    assert g.state.last_cause is None


def test_it_survives_a_restart(tmp_path, clock):
    g = ready(tmp_path, clock)
    _projection(g, clock)
    g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a"))
    want = g.state.last_cause
    g.store.close()
    again = make(tmp_path, clock)
    assert again.state.last_cause == want and again.state.fence == want.fence


def test_the_codec_carries_it_where_the_sdk_has_the_field(tmp_path, clock):
    g = ready(tmp_path, clock)
    _projection(g, clock)
    g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a"))
    pb = codec.gate_state(g.state)
    if not codec.HAS_FENCE_CAUSE:
        pytest.skip("this keelson build has no GateState.last_fence_cause (crowsnest-dev#290)")
    c = pb.last_fence_cause
    assert (c.kind, c.requested_by, c.fence.generation) == (1, "anna@ROC-2", g.state.generation)


def test_no_cause_before_the_gate_ever_fenced_alone(tmp_path, clock):
    g = make(tmp_path, clock)
    assert g.state.last_cause is None
