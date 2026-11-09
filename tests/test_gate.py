"""The gate's decisions and its sequence: the fence, the reservation, the per-DOF test, the
transition state, the direct abort. No network."""

import sqlite3

import pytest

from control_gate import model
from control_gate.actuation import LoggingActuator
from control_gate.model import DirectAbort, Fence, Mapping, Proposal, Sample
from control_gate.policy import parse
from control_gate.service import Gate
from control_gate.store import Store

V = "usv-alpha"
DOFS = ["rudder", "throttle"]
POL = parse({"dofs": DOFS, "automation_sources": ["navigation/0"], "transition": {"values": {"throttle": 0}}})
TED, TOKEN = "ted@ROC-1", "tok-ted"


class Clock:
    def __init__(self):
        self.t = 10_000

    def __call__(self):
        return self.t


@pytest.fixture
def clock():
    return Clock()


def make(tmp_path, clock, policy=POL):
    g = Gate(store=Store(tmp_path / "gate.sqlite"), policy=policy, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    g.boot()
    g.restore_constraints([], [])  # storage holds none (#345)
    g.restore_motion([], [])  # nor any motion_constraint (#85)
    return g


def rows(station="ROC-1", holder=TED, lease=TOKEN, envelope=()):
    return tuple(Mapping(d, holder, lease, f"{station}/helm-1/{d}", envelope, "", f"m-{d}") for d in DOFS)


class Helm:
    """A station's helm publishing candidate samples, one stream, rising sequence."""

    def __init__(self, gate, clock, station="ROC-1", stream="s1"):
        self.g, self.clock, self.station, self.stream = gate, clock, station, stream
        # One sequence per (gate, stream), so two Helm objects on one stream keep rising.
        self.seqs = gate.__dict__.setdefault("_test_seqs", {})

    def put(self, dof, value, fence=None, seq=None, stream=None, source=None):
        stream = stream or self.stream
        source = source or f"{self.station}/helm-1/{dof}"
        key = (source, stream)
        self.seqs[key] = seq if seq is not None else self.seqs.get(key, 0) + 1
        return self.g.sample(Sample(source, dof, value, stream, self.seqs[key], self.clock(), fence))

    def all(self, value=0.0):
        for d in DOFS:
            self.put(d, value)


def clear(g):
    """Cleared to move: the supervisor CLEAR and no maintenance procedure in force (#302)."""
    g.observe_safety(model.SAFETY_CLEAR, False)
    g.observe_maintenance(model.MAINTENANCE_INACTIVE)


def lease(g, holder=TED, token=TOKEN, ttl=2000):
    g.observe_lease("conn", holder, token, ttl, False)


def take(g, clock, station="ROC-1", kind=model.GRANT, tid="t1", **kw):
    """PREPARE → ACK → COMMIT, as the manager does it."""
    ack = g.prepare(Proposal(tid, kind, rows(station, **kw) if kind != model.ABORT_AUTOMATION else (), station, 2000, TED))
    if not ack.accepted:
        return ack, False
    applied, _, _ = g.commit(tid, ack.fence)
    return ack, applied


def ready(tmp_path, clock):
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    ack, applied = take(g, clock)
    assert ack.accepted and applied
    return g


# ---------------------------------------------------------------------------- the fence (I2, I13)


def test_a_fresh_gate_draws_a_term_and_starts_in_its_transition_state(tmp_path, clock):
    g = make(tmp_path, clock)
    assert len(g.state.term) == 32 and g.state.generation == 1 and g.state.in_transition
    rows_ = g.cycle()
    assert all(r.control_state == model.INHIBITED and r.value is None for r in rows_)  # no safety_state heard yet


def test_the_ack_reserves_n_plus_1_and_only_the_commit_activates_it(tmp_path, clock):
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    ack = g.prepare(Proposal("t1", model.GRANT, rows(), "ROC-1", 2000, TED))
    assert ack.accepted and ack.fence == Fence(g.state.term, 2)
    assert g.state.generation == 1 and g.state.mappings == ()  # nothing changed on the ACK
    applied, _, fence = g.commit("t1", ack.fence)
    assert applied and fence.generation == 2 and g.state.station == "ROC-1"
    assert all(m.fence == fence for m in g.state.mappings)


def test_a_proposal_that_names_a_fence_is_refused(tmp_path, clock):
    g = make(tmp_path, clock)
    Helm(g, clock).all(0)
    forged = tuple(Mapping(m.dof, m.authority_holder, m.authority_lease_id, m.execution_source, fence=Fence("ff" * 16, 9))
                   for m in rows())
    ack = g.prepare(Proposal("t1", model.GRANT, forged, "ROC-1", 2000))
    assert not ack.accepted and "names no fence" in ack.reason


def test_an_orphaned_prepare_lapses_on_the_gates_clock(tmp_path, clock):
    g = make(tmp_path, clock)
    clear(g)
    Helm(g, clock).all(0)
    ack = g.prepare(Proposal("t1", model.GRANT, rows(), "ROC-1", 2000))
    assert not g.prepare(Proposal("t2", model.GRANT, rows(), "ROC-1", 2000)).accepted  # one at a time
    clock.t += 2001
    Helm(g, clock, stream="s1").all(0)
    applied, reason, _ = g.commit("t1", ack.fence)
    assert not applied and "no such reservation" in reason
    assert g.prepare(Proposal("t2", model.GRANT, rows(), "ROC-1", 2000)).accepted


def test_a_commit_after_a_gate_restart_is_refused_and_the_old_authority_stands(tmp_path, clock):
    g = ready(tmp_path, clock)
    before = g.state.fence
    Helm(g, clock, stream="s2").all(0)
    ack = g.prepare(Proposal("t2", model.GRANT, rows(holder="anna@ROC-1", lease="tok-anna"), "ROC-1", 2000))
    assert ack.accepted
    g.store.close()
    g2 = make(tmp_path, clock)  # restarted after its ACK: the reservation is gone
    applied, _, fence = g2.commit("t2", ack.fence)
    assert not applied and fence == before and g2.state.mappings[0].authority_holder == TED


def test_the_fence_survives_a_restart_and_a_wiped_store_is_a_new_term(tmp_path, clock):
    g = ready(tmp_path, clock)
    fence = g.state.fence
    g.store.close()
    g2 = make(tmp_path, clock)
    assert g2.state.fence == fence and not g2.state.in_transition
    g2.store.close()
    db = sqlite3.connect(tmp_path / "gate.sqlite")
    db.execute("UPDATE meta SET v=replace(v, '\"generation\":2', '\"generation\":7') WHERE k='state'")
    db.commit()
    db.close()
    g3 = make(tmp_path, clock)
    assert g3.state.term != fence.term and g3.state.generation == 1 and g3.state.in_transition
    assert any("reinitialised" in e for e in g3.events)


# ---------------------------------------------------------------------------- neutral-or-match (I10)


def test_one_unmatched_lever_naks_the_whole_transfer(tmp_path, clock):
    g = make(tmp_path, clock)
    h = Helm(g, clock)
    h.put("rudder", 0)
    h.put("throttle", 80)  # 80 % ahead while the gate emits 0
    ack = g.prepare(Proposal("t1", model.GRANT, rows(), "ROC-1", 2000))
    assert not ack.accepted and ack.rejected_mapping_ids == ("m-throttle",) and g.state.reservation is None


def test_a_matched_lever_picks_up(tmp_path, clock):
    g = ready(tmp_path, clock)
    h = Helm(g, clock)
    h.put("throttle", 40)
    g.cycle()
    assert g.state.emitting["throttle"] == 40
    # Another station's lever at 42 % against 40 % emitted: within the band, so it may pick up.
    h2 = Helm(g, clock, station="ROC-1", stream="s9")
    h2.put("rudder", 0, source="ROC-1/helm-2/rudder")
    h2.put("throttle", 0, source="ROC-1/helm-2/throttle")  # a new stream starts at neutral (I11)...
    h2.put("throttle", 42, source="ROC-1/helm-2/throttle")  # ...then moves to match
    ack = g.prepare(Proposal("t2", model.GRANT, tuple(Mapping(d, "anna@ROC-1", "tok-anna", f"ROC-1/helm-2/{d}", (), "", d)
                                                     for d in DOFS), "ROC-1", 2000))
    assert ack.accepted, ack.reason


def test_a_silent_incoming_source_is_not_ready(tmp_path, clock):
    g = make(tmp_path, clock)
    ack = g.prepare(Proposal("t1", model.GRANT, rows(), "ROC-1", 2000))
    assert not ack.accepted and "no fresh sample" in ack.reason


def test_a_transfer_moves_every_dof_and_human_sources_sit_at_the_station(tmp_path, clock):
    g = make(tmp_path, clock)
    Helm(g, clock).all(0)
    assert not g.prepare(Proposal("t1", model.GRANT, rows()[:1], "ROC-1", 2000)).accepted
    assert not g.prepare(Proposal("t1", model.GRANT, rows("ROC-1"), "ROC-2", 2000)).accepted


# ---------------------------------------------------------------------------- the per-DOF test


def test_the_mapped_source_steers(tmp_path, clock):
    g = ready(tmp_path, clock)
    assert Helm(g, clock).put("rudder", 20)[0]
    out = {r.dof: r for r in g.cycle()}
    assert out["rudder"].value == 20 and out["rudder"].control_state == model.ACTIVE
    # Its zero from ready() was heard before the commit: a commit forgets every sample accepted under
    # the old authority, so a mapped source that has not spoken since is silent.
    assert out["throttle"].control_state == model.STALE and out["throttle"].value == 0


@pytest.mark.parametrize("case", ["other_source", "stale_fence", "envelope", "lease_lapsed", "not_clear", "lockout"])
def test_every_line_of_the_test_refuses_on_its_own(tmp_path, clock, case):
    g = ready(tmp_path, clock)
    h = Helm(g, clock)
    if case == "other_source":
        ok, why = h.put("rudder", 10, source="ROC-1/helm-2/rudder", stream="x")
    elif case == "stale_fence":
        ok, why = h.put("rudder", 10, fence=Fence(g.state.term, g.state.generation - 1))
    elif case == "envelope":
        g.state = model.replace(g.state, mappings=tuple(model.replace(m, envelope=(-30.0, 30.0, 0.0)) for m in g.state.mappings))
        ok, why = h.put("rudder", 35)
    elif case == "lease_lapsed":
        clock.t += 2001
        ok, why = h.put("rudder", 10)
    elif case == "not_clear":
        g.observe_safety(2, False)
        ok, why = h.put("rudder", 10)
    else:
        g.observe_safety(model.SAFETY_CLEAR, True)
        ok, why = h.put("rudder", 10)
    assert not ok and why


def test_the_rate_limit(tmp_path, clock):
    g = ready(tmp_path, clock)
    g.state = model.replace(g.state, mappings=tuple(model.replace(m, envelope=(-100.0, 100.0, 20.0)) for m in g.state.mappings))
    h = Helm(g, clock)
    assert h.put("rudder", 0)[0]
    clock.t += 100
    assert not h.put("rudder", 10)[0]  # 100 %/s
    clock.t += 400
    assert h.put("rudder", 9)[0]  # 18 %/s over the half second since the last accepted


def test_a_sample_accepted_under_the_old_holder_does_not_drive_under_the_new_one(tmp_path, clock):
    """A grant on the same helm: what ted's lever said is not anna's order, even for the 1 s it is fresh."""
    g = ready(tmp_path, clock)
    h = Helm(g, clock)
    assert h.put("rudder", 20)[0]
    g.cycle()
    h.put("rudder", 20)
    h.put("throttle", 0)
    lease(g, "anna@ROC-1", "tok-anna")
    ack, applied = take(g, clock, tid="t2", holder="anna@ROC-1", lease="tok-anna")
    assert applied
    out = {r.dof: r for r in g.cycle()}
    assert out["rudder"].control_state == model.STALE and out["rudder"].value == 0


def test_ordering_is_per_stream_and_a_fresh_late_sample_is_dropped(tmp_path, clock):
    """I11: seq 102 over the satellite link arrives first; 101 over LTE, though fresh, is dropped."""
    g = ready(tmp_path, clock)
    h = Helm(g, clock)
    assert h.put("rudder", 12, seq=102)[0]
    ok, why = h.put("rudder", 10, seq=101)
    assert not ok and "out of order" in why


def test_a_helm_restarting_at_80_percent_cannot_regain_motion(tmp_path, clock):
    g = ready(tmp_path, clock)
    h = Helm(g, clock)
    h.put("throttle", 0)
    ok, why = h.put("throttle", 80, stream="restarted", seq=1)
    assert not ok and "neutral-reconnect" in why
    assert h.put("throttle", 0, stream="restarted", seq=2)[0]  # back to neutral: the new stream is taken
    assert h.put("throttle", 30, stream="restarted", seq=3)[0]


def test_a_silent_source_resolves_to_the_transition_value_not_its_last_word(tmp_path, clock):
    g = ready(tmp_path, clock)
    Helm(g, clock).put("throttle", 60)
    clock.t += POL.max_axis_age_ms + 1
    lease(g)  # the holder is still here; only the lever went quiet
    out = {r.dof: r for r in g.cycle()}
    assert out["throttle"].value == 0 and out["throttle"].control_state == model.STALE


def test_under_a_stop_or_the_lockout_the_gate_emits_nothing(tmp_path, clock):
    g = ready(tmp_path, clock)
    Helm(g, clock).put("throttle", 30)
    g.observe_safety(2, False)  # STOPPED
    assert all(r.value is None and r.control_state == model.INHIBITED for r in g.cycle())
    g.observe_safety(model.SAFETY_CLEAR, True)
    assert all(r.value is None for r in g.cycle())


def test_a_silent_supervisor_is_not_a_clear_one(tmp_path, clock):
    g = ready(tmp_path, clock)
    clock.t += POL.safety_ttl_ms + 1
    lease(g)
    assert all(r.control_state == model.INHIBITED for r in g.cycle())


# ---------------------------------------------------------------------------- the gate alone (I2, I9)


def test_the_holders_lease_lapses_and_the_gate_fences_alone(tmp_path, clock):
    g = ready(tmp_path, clock)
    n = g.state.generation
    clock.t += 2001
    clear(g)
    out = g.cycle()
    assert g.state.generation == n + 1 and g.state.in_transition and g.state.mappings == ()
    assert all(r.control_state == model.TRANSITION and r.value == 0 for r in out)
    assert any("fenced alone" in e for e in g.events)
    g.store.close()
    assert make(tmp_path, clock).state.generation == n + 1  # persisted before it emitted


def test_navigation_executes_under_a_humans_conn_and_the_humans_lease_is_what_counts(tmp_path, clock):
    """I9: the holder is not the source. navigation/0 holds no lease; when the human's lapses, the DOF
    withdraws though the navigator is perfectly fresh."""
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    nav = [Mapping(d, TED, TOKEN, "navigation/0", (), "", d) for d in DOFS]
    for d in DOFS:
        g.sample(Sample("navigation/0", d, 0, "n1", DOFS.index(d) + 1, clock()))
    ack = g.prepare(Proposal("t1", model.GRANT, tuple(nav), "ROC-1", 2000))
    assert ack.accepted and g.commit("t1", ack.fence)[0]
    assert g.sample(Sample("navigation/0", "rudder", 5, "n1", 10, clock()))[0]
    clock.t += 2001
    clear(g)
    g.sample(Sample("navigation/0", "rudder", 5, "n1", 11, clock()))
    g.cycle()
    assert g.state.in_transition


def test_no_lease_heard_yet_after_a_restart_is_not_a_lapse_within_the_grace(tmp_path, clock):
    g = ready(tmp_path, clock)
    n = g.state.generation
    g.store.close()
    g2 = make(tmp_path, clock)
    clear(g2)
    g2.cycle()
    assert g2.state.generation == n  # within the grace: not fenced, and nothing actuates (no lease heard)
    assert not Helm(g2, clock, stream="s5").put("rudder", 0)[0]
    clock.t += POL.lease_grace_ms + 1
    clear(g2)
    g2.cycle()
    assert g2.state.generation == n + 1


def test_abort_through_the_manager_proposes_no_map_and_enters_the_transition_state(tmp_path, clock):
    g = ready(tmp_path, clock)
    ack, applied = take(g, clock, kind=model.ABORT_AUTOMATION, tid="a1")
    assert applied and g.state.in_transition and g.state.mappings == ()
    assert "abort automation" in g.state.transition_reason


# ---------------------------------------------------------------------------- the direct abort


def _projection(g, clock, feed_ttl=15000):
    g.observe_assignments(None, model.AssignmentSeen("anna@ROC-2", "as-7", 4), "authority-manager/0", feed_ttl)
    g.observe_session(model.NAVIGATIONAL, "anna@ROC-2", "sess-a", "as-7", 30000, False)


def test_the_direct_abort_verifies_on_its_own_projection(tmp_path, clock):
    g = ready(tmp_path, clock)
    _projection(g, clock)
    n = g.state.generation
    assert not g.abort_direct(DirectAbort("bo@ROC-2", "as-7", 4, "sess-a"))[0]
    assert not g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 3, "sess-a"))[0]  # stale revision
    assert not g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "wrong"))[0]
    ok, _, fence = g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a", "no answer from the navigator"))
    assert ok and fence.generation == n + 1 and g.state.in_transition


def test_with_the_manager_alive_a_lapsed_session_is_refused_with_it_dead_the_last_one_counts(tmp_path, clock):
    g = ready(tmp_path, clock)
    _projection(g, clock, feed_ttl=60000)
    clock.t += 30001  # the session lapsed; the feed (60 s) is still live
    assert not g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a"))[0]
    clock.t += 30000  # now the feed is stale too: the manager is gone
    assert g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a"))[0]


# ---------------------------------------------------------------------------- guidance: DECISION ∧ EXECUTION (I15)

NAV = "navigation/0"
NAV_PRINCIPAL = "navigation/0@usv-alpha"


def _carry_out(tmp_path, clock, functions=None, revision=7):
    """A human holds the conn; navigation/0 carries out under it on every DOF (the mapping's source)."""
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    nav_rows = tuple(Mapping(d, TED, TOKEN, NAV, (), "", f"m-{d}") for d in DOFS)
    for i, d in enumerate(DOFS):
        g.sample(Sample(NAV, d, 0, "n1", i + 1, clock()))
    ack = g.prepare(Proposal("t1", model.GRANT, nav_rows, "ROC-1", 2000, TED))
    assert ack.accepted, ack.reason
    g.commit("t1", ack.fence)
    g.observe_navigation(revision, functions if functions is not None else {"track_keeping": (NAV_PRINCIPAL, 4)})
    g._seq = 10
    return g


def _guide(g, clock, **kw):
    g._seq += 1
    base = dict(source=NAV, dof="rudder", value_pct=5.0, stream_id="n1", sequence=g._seq, recv_ms=clock(),
                decision="nav_function", function="track_keeping", nav_revision=7, execution="carry_out", mapping_id="m-rudder")
    base.update(kw)
    return g.guidance(model.Guidance(**base))


def test_i15_a_function_it_does_not_hold_is_refused_though_execution_passes(tmp_path, clock):
    g = _carry_out(tmp_path, clock)
    ok, why = _guide(g, clock, function="collision_avoidance")
    assert not ok and "does not hold collision_avoidance" in why
    assert model.execution_check(g.state, model.Guidance(NAV, "rudder", 5, "n1", 99, clock(), execution="carry_out",
                                                         mapping_id="m-rudder"), clock()) == ""  # EXECUTION alone passes
    ok, why = _guide(g, clock)  # the function it holds, at method 4
    assert ok, why


def test_i15_each_decision_refusal_on_its_own(tmp_path, clock):
    g = _carry_out(tmp_path, clock, functions={"track_keeping": (NAV_PRINCIPAL, 2)})
    assert "below DELEGATED" in _guide(g, clock)[1]
    g = _carry_out(tmp_path / "b", clock)
    assert "revision" in _guide(g, clock, nav_revision=6)[1]
    assert "no navigation_state heard from navigation/0" in _guide(g, clock, decision="human_order", order_id="o-1")[1]
    assert "no decision basis" in _guide(g, clock, decision="")[1]
    g2 = make(tmp_path / "c", clock)
    assert "no navigation authority document" in model.decision_check(g2.state, g2.policy, model.Guidance(
        NAV, "rudder", 0, "n", 1, 0, decision="nav_function", function="track_keeping", nav_revision=1), 0)


def test_i15_each_execution_refusal_on_its_own(tmp_path, clock):
    g = _carry_out(tmp_path, clock)
    assert "not this DOF's" in _guide(g, clock, mapping_id="m-throttle")[1]
    assert "does not hold the conn" in _guide(g, clock, execution="direct")[1]  # a human holds it
    assert "no execution basis" in _guide(g, clock, execution="")[1]
    # A helm source claiming the navigator's mapping is not its execution source.
    ok, why = g.guidance(model.Guidance("ROC-1/helm-1/rudder", "rudder", 5, "h", 1, clock(), decision="nav_function",
                                        function="track_keeping", nav_revision=7, execution="carry_out", mapping_id="m-rudder"))
    assert not ok


def test_direct_execution_under_full_automation(tmp_path, clock):
    g = _carry_out(tmp_path, clock)
    lease(g, NAV_PRINCIPAL, TOKEN)  # the navigator itself holds the conn now (same lease token)
    g.state = model.replace(g.state, mappings=tuple(model.replace(m, authority_holder=NAV_PRINCIPAL) for m in g.state.mappings))
    ok, why = _guide(g, clock, execution="direct")
    assert ok, why


def test_a_refused_guidance_sample_still_counts_for_ordering(tmp_path, clock):
    g = _carry_out(tmp_path, clock)
    _guide(g, clock, function="collision_avoidance", sequence=50)
    ok, why = _guide(g, clock, sequence=49)  # older than the refused one: dropped as out of order
    assert not ok and "out of order" in why


# ---------------------------------------------------------------------------- ResolvedControl


def test_the_cycle_record_carries_what_resolved_control_needs(tmp_path, clock):
    pol = parse({"dofs": DOFS, "groups": {"helm": ["rudder", "throttle"]}})
    g = make(tmp_path, clock, policy=pol)
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    take(g, clock)
    Helm(g, clock).put("rudder", 12)
    g.cycle()
    c1 = g.last_cycle
    g.cycle()
    c2 = g.last_cycle
    assert (c1.sequence, c2.sequence) == (1, 2) and c2.groups == (("helm", 2),)
    assert c2.actuation_allowed and not c2.in_transition and c2.fence == g.state.fence
    from control_gate import codec

    rc = codec.resolved_control(c2, V, "control-gate")
    rudder = next(d for d in rc.dofs if d.dof == "rudder")
    assert rudder.accepted_pct == 12 and rudder.control_state == 1 and rudder.feedback_quality == 4  # ACTIVE, NONE
    g.observe_safety(2, False)  # STOPPED: emits nothing
    g.cycle()
    stopped = codec.resolved_control(g.last_cycle, V, "control-gate")
    assert all(not d.HasField("accepted_pct") and d.control_state == 4 for d in stopped.dofs)
    assert not stopped.actuation_allowed


def test_i15_the_function_held_by_someone_else_is_not_this_sources(tmp_path, clock):
    g = _carry_out(tmp_path, clock, functions={"track_keeping": ("anna@ROC-2", 4)})
    ok, why = _guide(g, clock)
    assert not ok and "does not hold track_keeping" in why


def test_i15_carry_out_naming_a_mapping_whose_source_is_another(tmp_path, clock):
    """EXECUTION on its own: the right mapping id, but that mapping executes someone else."""
    g = _carry_out(tmp_path, clock)
    g.state = model.replace(g.state, mappings=tuple(model.replace(m, execution_source="navigation/1") for m in g.state.mappings))
    why = model.execution_check(g.state, model.Guidance(NAV, "rudder", 5, "n1", 99, clock(), execution="carry_out",
                                                        mapping_id="m-rudder"), clock())
    assert "execution source is navigation/1" in why


def test_a_refused_guidance_sample_is_still_what_the_source_says_for_neutral_or_match(tmp_path, clock):
    """Carry-out needs a fresh sample from the navigator at PREPARE, before its mapping exists."""
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    g.observe_navigation(1, {"track_keeping": (NAV_PRINCIPAL, 4)})
    for i, d in enumerate(DOFS):
        ok, _ = g.guidance(model.Guidance(NAV, d, 0, "n1", i + 1, clock(), decision="nav_function", function="track_keeping",
                                          nav_revision=1, execution="carry_out", mapping_id=f"m-{d}"))
        assert not ok  # no mapping yet: refused to act on
    nav_rows = tuple(Mapping(d, TED, TOKEN, NAV, (), "", f"m-{d}") for d in DOFS)
    ack = g.prepare(Proposal("t1", model.GRANT, nav_rows, "ROC-1", 2000, TED))
    assert ack.accepted, ack.reason


# ---------------------------------------------------------------------------- HUMAN_ORDER (crowsnest-dev#94)


def _ordered(tmp_path, clock, order_id="o-7", producer="navigation/0/conduct"):
    """A human holds the conn; navigation/0 carries out their order o-7 under it."""
    g = _carry_out(tmp_path, clock, functions={})  # the navigator holds no function: only the order decides
    g.observe_navigation_state(producer, order_id)
    return g


def _order(g, clock, order_id="o-7", **kw):
    return _guide(g, clock, decision="human_order", order_id=order_id, function="", nav_revision=0, **kw)


def test_human_order_passes_when_it_is_the_order_in_force_on_a_fresh_record(tmp_path, clock):
    g = _ordered(tmp_path, clock)
    ok, why = _order(g, clock)
    assert ok, why
    assert g.state.accepted["rudder"].value_pct == 5.0


@pytest.mark.parametrize("setup,order_id,why", [
    (lambda g, c: None, "o-6", "order o-6 is not the one in force (o-7)"),
    (lambda g, c: None, "", "HUMAN_ORDER names no order id"),
    (lambda g, c: g.observe_navigation_state("navigation/0/conduct", None), "o-7", "reports no human order in force"),
    (lambda g, c: setattr(c, "t", c.t + 3001), "o-7", "not heard for 3001 ms"),
    (lambda g, c: g.observe_navigation_state("navigation/0/other", "o-7"), "o-7", "more than one navigation_state under navigation/0"),
])
def test_human_order_refusals(tmp_path, clock, setup, order_id, why):
    g = _ordered(tmp_path, clock)
    setup(g, clock)
    ok, got = _order(g, clock, order_id)
    assert not ok and why in got


def test_another_navigators_record_is_not_this_sources(tmp_path, clock):
    g = _carry_out(tmp_path, clock, functions={})
    g.observe_navigation_state("navigation/01/conduct", "o-7")  # a prefix, not a chunk boundary
    ok, why = _order(g, clock)
    assert not ok and "no navigation_state heard from navigation/0" in why


def test_a_stale_second_producer_does_not_make_it_ambiguous(tmp_path, clock):
    g = _carry_out(tmp_path, clock, functions={})
    g.observe_navigation_state("navigation/0/old", "o-1")
    clock.t += 3001
    lease(g)  # the holder renews meanwhile; only the old record has gone stale
    g.observe_navigation_state("navigation/0/conduct", "o-7")
    ok, why = _order(g, clock)
    assert ok, why


def test_human_order_still_needs_execution(tmp_path, clock):
    """DECISION passing implies nothing about EXECUTION (I15)."""
    g = _ordered(tmp_path, clock)
    ok, why = _order(g, clock, mapping_id="m-throttle")
    assert not ok and "not this DOF's" in why


def test_the_order_freshness_window_is_policy(tmp_path, clock):
    from control_gate.policy import parse

    assert parse({"dofs": ["rudder"], "navigation_state_max_age_ms": 4500}).navigation_state_max_age_ms == 4500
    assert parse({"dofs": ["rudder"]}).navigation_state_max_age_ms == 3000


# ---------------------------------------------------------------------------- provenance (crowsnest-dev#98)


def _active_rudder(tmp_path, clock):
    g = make(tmp_path, clock)
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    take(g, clock)
    return g


def test_resolved_control_names_the_sample_it_accepted_and_its_hold(tmp_path, clock):
    """t1 sample, t2 publish, t3 receipt (wall) and t3→t4 on the gate's own clock, copied from the
    accepted candidate onto its ResolvedDof."""
    from keelson.payloads.ControlCandidate_pb2 import ControlCandidate
    from keelson.payloads.GuidanceCandidate_pb2 import CandidateStream, ClockQuality

    from control_gate import codec

    g = _active_rudder(tmp_path, clock)
    pb = ControlCandidate(source="ROC-1/helm-1/rudder", dof="rudder", value_pct=12.0,
                          stream=CandidateStream(stream_id="s1", sequence=99, clock_quality=ClockQuality.CLOCK_QUALITY_SYNCHRONISED))
    pb.timestamp.FromNanoseconds(1_000_000_000)
    recv_ms = clock()
    ok, why = g.sample(codec.sample_from_control_candidate(pb, recv_ms, recv_wall_ns=1_060_000_000, published_ns=1_004_000_000))
    assert ok, why
    clock.t += 37
    g.cycle()
    rc = codec.resolved_control(g.last_cycle, V, "control-gate")
    d = next(d for d in rc.dofs if d.dof == "rudder")
    assert (d.source_stream_id, d.source_sequence, d.source_clock_quality) == ("s1", 99, ClockQuality.CLOCK_QUALITY_SYNCHRONISED)
    assert d.source_sample_time.ToNanoseconds() == 1_000_000_000
    assert d.source_published_at.ToNanoseconds() == 1_004_000_000
    assert d.gate_received_at.ToNanoseconds() == 1_060_000_000
    assert d.HasField("gate_hold_ms") and d.gate_hold_ms == 37


def test_a_row_with_no_accepted_sample_carries_no_provenance(tmp_path, clock):
    from control_gate import codec

    g = _active_rudder(tmp_path, clock)
    Helm(g, clock).put("rudder", 5)
    g.observe_safety(2, False)  # STOPPED: INHIBITED, nothing emitted
    g.cycle()
    stopped = codec.resolved_control(g.last_cycle, V, "control-gate")
    for d in stopped.dofs:
        assert not d.HasField("gate_hold_ms") and not d.HasField("source_sample_time") and d.source_sequence == 0
    clear(g)
    clock.t += POL.max_axis_age_ms + 1  # silent past the dead-man: STALE
    g.cycle()
    stale = codec.resolved_control(g.last_cycle, V, "control-gate")
    assert all(d.control_state == 2 and not d.HasField("gate_hold_ms") for d in stale.dofs)


def test_provenance_decides_nothing(tmp_path, clock):
    """Two gates fed samples that differ only in provenance resolve identically, row for row."""
    out = []
    for i, prov in enumerate(({}, {"sample_time_ns": 1, "published_ns": 2, "clock_quality": 2, "recv_wall_ns": 10**18})):
        g = _active_rudder(tmp_path / str(i), clock)
        src = "ROC-1/helm-1/rudder"
        seq = g._test_seqs[(src, "s1")] + 1
        ok, why = g.sample(Sample(src, "rudder", 33.0, "s1", seq, clock(), **prov))
        out.append((ok, why, g.cycle()))
    assert out[0] == out[1]
    assert Sample("a", "rudder", 1.0, "s", 1, 5) == Sample("a", "rudder", 1.0, "s", 1, 5, sample_time_ns=7, recv_wall_ns=9)
