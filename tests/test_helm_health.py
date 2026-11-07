"""A helm's own health, reported and never judged (crowsnest-dev#309).

ControlCandidate carries the helm's profile_version and device_health (keelson#382): DEGRADED when its
read watchdog has not read the device within budget. The gate copies both onto the accepted sample's
ResolvedDof and logs the transition, and accepts a DEGRADED helm exactly as it would a NOMINAL one:
validity is receive-side freshness. The codec cases need a keelson build with the fields and skip on one
without.
"""

import logging

import pytest
from keelson.payloads.ControlCandidate_pb2 import ControlCandidate
from keelson.payloads.ResolvedControl_pb2 import ResolvedDof

from control_gate import codec, model
from control_gate.model import Sample
from test_gate import POL, V, Clock, Helm, clear, lease, make, take

DEGRADED, NOMINAL = model.HEALTH_DEGRADED, 4
SRC = "ROC-1/helm-1/rudder"
HAS_CANDIDATE = "device_health" in ControlCandidate.DESCRIPTOR.fields_by_name
HAS_RESOLVED = "source_device_health" in ResolvedDof.DESCRIPTOR.fields_by_name
needs_sdk = pytest.mark.skipif(not (HAS_CANDIDATE and HAS_RESOLVED),
                               reason="keelson build without ControlCandidate.device_health / ResolvedDof.source_device_health")


@pytest.fixture
def clock():
    return Clock()


def _active(tmp_path, clock):
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    take(g, clock)
    return g


def _put(g, clock, value, health, profile="hotas-x/3"):
    seqs = g.__dict__.setdefault("_test_seqs", {})
    seqs[(SRC, "s1")] = seqs.get((SRC, "s1"), 0) + 1
    return g.sample(Sample(SRC, "rudder", value, "s1", seqs[(SRC, "s1")], clock(), profile_version=profile,
                           device_health=health))


def _rudder(rows):
    return next(r for r in rows if r.dof == "rudder")


def test_a_degraded_helm_is_accepted_and_resolves_as_a_nominal_one_would(tmp_path, clock):
    """Row for row the same cycle: the health decides nothing."""
    out = []
    for i, health in enumerate((NOMINAL, DEGRADED)):
        g = _active(tmp_path / str(i), clock)
        ok, why = _put(g, clock, 33.0, health)
        out.append((ok, why, g.cycle()))
    assert out[0] == out[1]
    ok, _, rows = out[1]
    assert ok and _rudder(rows).control_state == model.ACTIVE and _rudder(rows).value == 33.0
    assert _rudder(rows).provenance.device_health == DEGRADED


def test_a_degraded_helm_past_the_dead_man_is_stale_for_its_silence_not_its_health(tmp_path, clock):
    g = _active(tmp_path, clock)
    _put(g, clock, 10.0, DEGRADED)
    clock.t += POL.max_axis_age_ms + 1
    r = _rudder(g.cycle())
    assert r.control_state == model.STALE and "DEGRADED" not in r.reason


def test_the_transition_is_logged_once_each_way(tmp_path, clock, caplog):
    g = _active(tmp_path, clock)
    caplog.set_level(logging.INFO, logger="control_gate.service")
    for health in (NOMINAL, DEGRADED, DEGRADED, DEGRADED, NOMINAL, NOMINAL):
        assert _put(g, clock, 5.0, health)[0]
    said = [(r.levelno, r.getMessage()) for r in caplog.records if "reports itself" in r.getMessage()]
    assert said == [
        (logging.WARNING, f"helm {SRC} reports itself DEGRADED (profile 'hotas-x/3'): still judged on receipt freshness alone"),
        (logging.INFO, f"helm {SRC} reports itself NOMINAL again"),
    ]


def test_a_refused_sample_does_not_move_the_reported_health(tmp_path, clock):
    g = _active(tmp_path, clock)
    _put(g, clock, 5.0, NOMINAL)
    seq = g._test_seqs[(SRC, "s1")]
    ok, _ = g.sample(Sample(SRC, "rudder", 5.0, "s1", seq, clock(), device_health=DEGRADED))  # not rising
    assert not ok and g.helm_health[SRC] == NOMINAL


@needs_sdk
def test_resolved_control_reports_degraded_and_a_nominal_sample_clears_it(tmp_path, clock):
    from keelson.payloads.EntityHealth_pb2 import HealthLevel
    from keelson.payloads.GuidanceCandidate_pb2 import CandidateStream

    g = _active(tmp_path, clock)

    def send(seq, health):
        pb = ControlCandidate(source=SRC, dof="rudder", value_pct=12.0, stream=CandidateStream(stream_id="s1", sequence=seq),
                              profile_version="hotas-x/3", device_health=health)
        ok, why = g.sample(codec.sample_from_control_candidate(pb, clock()))
        assert ok, why
        g.cycle()
        return next(d for d in codec.resolved_control(g.last_cycle, V, "control-gate").dofs if d.dof == "rudder")

    base = g._test_seqs[(SRC, "s1")]
    d = send(base + 1, HealthLevel.HEALTH_DEGRADED)
    assert (d.control_state, d.accepted_pct) == (1, 12.0)  # ACTIVE: accepted, and emitted
    assert (d.source_device_health, d.source_profile_version) == (HealthLevel.HEALTH_DEGRADED, "hotas-x/3")
    d = send(base + 2, HealthLevel.HEALTH_NOMINAL)
    assert d.source_device_health == HealthLevel.HEALTH_NOMINAL


@needs_sdk
def test_a_row_with_no_accepted_sample_reports_no_health(tmp_path, clock):
    from keelson.payloads.EntityHealth_pb2 import HealthLevel

    g = _active(tmp_path, clock)
    _put(g, clock, 5.0, DEGRADED)
    g.observe_safety(2, False)  # STOPPED: INHIBITED, nothing accepted onto the row
    g.cycle()
    for d in codec.resolved_control(g.last_cycle, V, "control-gate").dofs:
        assert (d.source_device_health, d.source_profile_version) == (HealthLevel.HEALTH_UNKNOWN, "")


def test_a_build_without_the_fields_reads_none_and_writes_none(tmp_path, clock):
    """On the pinned build (no keelson#382): an UNKNOWN health and no profile, and the codec does not fail."""
    pb = ControlCandidate(source=SRC, dof="rudder", value_pct=1.0)
    s = codec.sample_from_control_candidate(pb, 0)
    if not HAS_CANDIDATE:
        assert (s.device_health, s.profile_version) == (0, "")
    g = _active(tmp_path, clock)
    _put(g, clock, 5.0, DEGRADED)
    g.cycle()
    codec.resolved_control(g.last_cycle, V, "control-gate")
