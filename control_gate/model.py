"""The motion gate's decisions, and nothing else. Pure: no clock, no store, no Zenoh.

Normative sources: keelson `protocols/command_authority.yaml`, `ControlMapping.proto`,
`interfaces/ControlGate.proto` (#331, #359) and `SafetyState.proto` (#332); crowsnest-dev
docs/COMMAND-ARCHITECTURE.md §0, §2.1, §3, §4 ("I2", "I6", "I9", "I10", "I11", "I13" are its §8
invariants).

THE GATE IS THE ENFORCER. Every candidate sample reaches actuation only through `on_sample` and
`resolve`, and only when every line of §4's test holds:

    fresh ∧ ordered ∧ authorised ∧ fenced ∧ sourced ∧ enveloped ∧ permitted

IT MINTS `plant_generation` AND CLAMPS BY THE PLANT CONSTRAINT (§1.1, I16; crowsnest-dev#270). The
authority manager authorises a change of ENGINEER holder by publishing the new holder's lease; a token
this gate has not seen is that change, and the gate mints the next generation under its own term
(`observe_lease`). In the same step it REBASES the constraint in force -- the same devices, revision 1,
the new fence -- so a change of engineer can never relax it. A constraint record is applied only under
the current plant fence, at a newer revision, from the live ENGINEER holder (`observe_constraint`), and
every resolved value of a propulsion DOF is clamped by it (`resolve`). Silence never lifts it.

IT OWNS BOTH HALVES OF THE MOTION FENCE. The term is a random 128-bit value drawn when the store is
(re)initialised and never otherwise. The generation is reserved here on an ACK (`prepare`), made
current only on COMMIT (`commit`), and incremented here ALONE when authority must change with no
other process alive (`fence_alone`, `abort_direct`). Nothing outside this module makes a fence.

Every function takes the current `State` and returns a new one; nothing is mutated. Times are this
process's monotonic milliseconds and every timeout is judged on them; a source timestamp is never
read here (§4: "Timeouts are decided on the receiver's monotonic clock").
"""

from __future__ import annotations

import secrets
from dataclasses import dataclass, field, replace

GRANT, TAKE_CONTROL, STATION_HANDOVER, ABORT_AUTOMATION = "grant", "take_control", "station_handover", "abort_automation"
OVERALL, NAVIGATIONAL = "overall", "navigational"

# keelson.SafetyState.State
SAFETY_CLEAR = 1

# keelson.MaintenanceState.State (keelson#377): only INACTIVE lets her move. The names are for the reason a
# DOF is INHIBITED, so an operator reads which step of the procedure is holding her.
MAINTENANCE_INACTIVE = 1
MAINTENANCE_STATES = {0: "UNSPECIFIED", 1: "INACTIVE", 2: "REQUESTED", 3: "PREPARING", 4: "MAINTENANCE",
                      5: "READY_FOR_CHECK"}

# keelson.HealthLevel, as a helm reports ITSELF on ControlCandidate.device_health (crowsnest-dev#309).
# Named for the log only: no decision here reads a sample's health.
HEALTH_DEGRADED = 3
HEALTH_LEVELS = {0: "UNKNOWN", 1: "INACTIVE", 2: "CRITICAL", 3: "DEGRADED", 4: "NOMINAL", 5: "NOT_ADVERTISED"}

ACTIVE, STALE, TRANSITION, INHIBITED, UNMAPPED = "ACTIVE", "STALE", "TRANSITION", "INHIBITED", "UNMAPPED"


def new_term() -> str:
    return secrets.token_hex(16)


# ---------------------------------------------------------------------------- records


@dataclass(frozen=True)
class Fence:
    term: str
    generation: int


@dataclass(frozen=True)
class Mapping:
    dof: str
    authority_holder: str
    authority_lease_id: str
    execution_source: str
    envelope: tuple = ()  # (min_pct, max_pct, max_rate_pct_per_s), or () for none
    group_id: str = ""
    mapping_id: str = ""
    fence: Fence | None = None


@dataclass(frozen=True)
class DeviceLimit:
    """One `PropulsionConstraint.DeviceConstraint`. `max_power_pct` None is no cap (never 0)."""

    device_id: str
    max_power_pct: float | None = None
    inhibit: bool = False
    reason: str = ""


@dataclass(frozen=True)
class Constraint:
    """The plant constraint in force (`propulsion_constraint/{vessel}`), under `plant_generation`."""

    fence: Fence
    revision: int
    controller_id: str
    controller_site: str = ""
    devices: tuple[DeviceLimit, ...] = ()

    def limit(self, device_id: str) -> DeviceLimit | None:
        return next((d for d in self.devices if d.device_id == device_id), None)


# keelson GateState.last_fence_cause kinds (crowsnest-dev#290)
CAUSE_ABORT, CAUSE_CONN_LAPSE, CAUSE_INVALID_SOURCE, CAUSE_LOCAL_FALLBACK = "abort_automation", "conn_lapse", "invalid_source", "local_fallback"


@dataclass(frozen=True)
class FenceCause:
    """Why the gate last moved its fence ALONE, with the fence it produced. Cleared by the next COMMIT, so
    it never describes a fence a transfer has since replaced. A manager that was down reads it at
    reconcile: an abort it did not see must still take the automation's functions back to the human
    who asked (crowsnest-dev#271/#290)."""

    kind: str
    fence: Fence
    requested_by: str = ""
    assignment_id: str = ""
    assignment_revision: int = 0
    reason: str = ""


@dataclass(frozen=True)
class Proposal:
    """A PREPARE. It carries no fence; a row that names one is refused (I2)."""

    transfer_id: str
    kind: str
    mappings: tuple[Mapping, ...]
    station: str
    prepare_ttl_ms: int
    requested_by: str = ""


@dataclass(frozen=True)
class Reservation:
    proposal: Proposal
    fence: Fence
    deadline_ms: int


@dataclass(frozen=True)
class Ack:
    accepted: bool
    fence: Fence | None = None
    reason: str = ""
    rejected_mapping_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class Heard:
    """Something heard on the bus, stamped with LOCAL receipt."""

    holder: str
    token: str
    ttl_ms: int
    heard_ms: int
    assignment_id: str = ""

    def live(self, now_ms: int) -> bool:
        return now_ms - self.heard_ms <= self.ttl_ms


@dataclass(frozen=True)
class AssignmentSeen:
    assignee: str
    assignment_id: str
    revision: int


@dataclass(frozen=True)
class SafetySeen:
    state: int
    local_lockout: bool
    heard_ms: int


@dataclass(frozen=True)
class MaintenanceSeen:
    """The manager's `maintenance_state/{vessel}`, stamped on LOCAL receipt: the manager republishes it at its
    heartbeat, so a silent manager reads as stale here, never as a standing INACTIVE (crowsnest-dev#302)."""

    state: int
    heard_ms: int


@dataclass(frozen=True)
class Stream:
    stream_id: str
    last_sequence: int


@dataclass(frozen=True)
class Sample:
    """One candidate sample: a station's axis, or the navigator's desired_* value. `recv_ms` is local
    receipt. `fence` is the caller's, when the candidate carries one."""

    source: str
    dof: str
    value_pct: float
    stream_id: str
    sequence: int
    recv_ms: int
    fence: Fence | None = None
    # PROVENANCE, for latency and audit only (crowsnest-dev#98): no decision reads it, and
    # compare=False keeps it out of equality, so two samples that differ only here are one sample.
    sample_time_ns: int | None = field(default=None, compare=False)  # t1, the source's clock
    published_ns: int | None = field(default=None, compare=False)  # t2, the source's clock
    clock_quality: int = field(default=0, compare=False)  # keelson.ClockQuality
    recv_wall_ns: int | None = field(default=None, compare=False)  # t3, this gate's wall clock
    # The helm's OWN word about itself (crowsnest-dev#309), copied onto ResolvedDof like the
    # provenance and as little judged: a DEGRADED helm is accepted exactly as a NOMINAL one would be.
    profile_version: str = field(default="", compare=False)  # the calibration that shaped value_pct
    device_health: int = field(default=0, compare=False)  # keelson.HealthLevel, its own read watchdog


@dataclass(frozen=True)
class Guidance:
    """One GuidanceCandidate from the navigator: a Sample plus the two bases it claims.

    decision   "nav_function" (with `function` and `nav_revision`), "human_order" (with
               `order_id`), or "" for none
    execution  "direct", "carry_out" (with `mapping_id`), or "" for none"""

    source: str
    dof: str
    value_pct: float
    stream_id: str
    sequence: int
    recv_ms: int
    decision: str = ""
    function: str = ""  # e.g. "track_keeping", as the navigation authority document names it
    nav_revision: int = 0
    order_id: str = ""
    execution: str = ""
    mapping_id: str = ""
    # PROVENANCE, for latency and audit only (crowsnest-dev#98): no decision reads it, and
    # compare=False keeps it out of equality, so two samples that differ only here are one sample.
    sample_time_ns: int | None = field(default=None, compare=False)  # t1, the source's clock
    published_ns: int | None = field(default=None, compare=False)  # t2, the source's clock
    clock_quality: int = field(default=0, compare=False)  # keelson.ClockQuality
    recv_wall_ns: int | None = field(default=None, compare=False)  # t3, this gate's wall clock


@dataclass(frozen=True)
class NavStateSeen:
    """One producer's latest `navigation_state`: the human decision it says is in force, if any,
    stamped on the gate's clock at local receipt. What a HUMAN_ORDER basis is checked against."""

    order_id: str | None
    recv_ms: int


@dataclass(frozen=True)
class NavDoc:
    """The navigation authority document, as last heard: revision, and per function (holder, method)."""

    revision: int
    functions: dict  # function -> (holder principal, control method 1..5)


CONTROL_METHOD_DELEGATED = 3


@dataclass(frozen=True)
class Resolved:
    """One DOF of one cycle: what was asked, what goes out, and why. `value` None = emit nothing.
    `provenance` is the accepted sample an ACTIVE row carries, copied into ResolvedDof; never judged."""

    dof: str
    requested: float | None
    value: float | None
    source: str
    control_state: str
    reason: str = ""
    provenance: Sample | None = field(default=None, compare=False)


@dataclass(frozen=True)
class State:
    # ---- durable (store.py persists these; a reinitialised store draws a new term)
    vessel_id: str
    term: str
    generation: int = 1
    mappings: tuple[Mapping, ...] = ()
    station: str = ""
    in_transition: bool = True  # a gate with no committed map is in its transition state
    transition_reason: str = "no map committed"
    plant_generation: int = 1  # under the same term: the motion gate's (Fence.proto)
    engineer_token: str = ""  # the ENGINEER token the current plant_generation belongs to
    constraint: Constraint | None = None  # the plant constraint in force; never lifted by silence
    last_cause: FenceCause | None = None  # why the gate last fenced alone (#290); None since a COMMIT
    # ---- volatile: lost on restart by design (a reservation does not survive the gate, §4)
    reservation: Reservation | None = None
    boot_ms: int = 0
    leases: dict = field(default_factory=dict)  # slot -> Heard (the conn is slot "conn")
    sessions: dict = field(default_factory=dict)  # role -> Heard (naming its assignment)
    assignments: dict = field(default_factory=dict)  # role -> AssignmentSeen
    feed: Heard | None = None  # the command_assignment feed: holder = issuer, ttl = feed_ttl
    safety: SafetySeen | None = None
    maintenance: MaintenanceSeen | None = None  # volatile like `safety`: a restarted gate waits to hear it
    streams: dict = field(default_factory=dict)  # source -> Stream
    latest: dict = field(default_factory=dict)  # (source, dof) -> Sample, every ordered sample heard
    accepted: dict = field(default_factory=dict)  # dof -> Sample that passed the test
    emitting: dict = field(default_factory=dict)  # dof -> value last resolved
    navigation: NavDoc | None = None  # the navigation authority document, for DECISION
    nav_states: dict = field(default_factory=dict)  # navigation_state producer -> NavStateSeen

    @property
    def fence(self) -> Fence:
        return Fence(self.term, self.generation)

    @property
    def plant_fence(self) -> Fence:
        return Fence(self.term, self.plant_generation)

    def mapping(self, dof: str) -> Mapping | None:
        return next((m for m in self.mappings if m.dof == dof), None)


# ---------------------------------------------------------------------------- policy


@dataclass(frozen=True)
class Policy:
    """The vessel's control matrix and timings (platform registry, `safe_state_policy`; changes only in
    maintenance). Every timeout is judged on the gate's own clock."""

    command_profile: str = "test"
    dofs: tuple[str, ...] = ()
    automation_sources: frozenset = frozenset()
    max_axis_age_ms: int = 1000  # the dead-man: vehicle_control/v1's, unchanged, never disableable
    neutral_band_pct: float = 5.0
    match_band_pct: float = 5.0
    transition_name: str = "neutral"
    transition_values: dict = field(default_factory=dict)  # dof -> value in the transition state
    safety_ttl_ms: int = 15000  # the supervisor republishes at 5 s
    require_safety_state: bool = True  # False only in a test rig with no supervisor, and labelled so
    maintenance_ttl_ms: int = 15000  # the manager republishes at 5 s
    # False only where nothing can publish it -- an SDK without MaintenanceState (keelson#377), a rig with
    # no manager -- and then the bin says so. True and unreadable is a gate that refuses to start.
    require_maintenance_state: bool = True
    lease_grace_ms: int = 5000  # after boot, before an unheard holder's lease counts as lapsed
    groups: dict = field(default_factory=dict)  # group_id -> dofs that move together (ResolvedControl)
    propulsion: dict = field(default_factory=dict)  # dof -> the platform's propulsion[].id it drives
    navigation_state_max_age_ms: int = 3000  # a HUMAN_ORDER is checked against a record this fresh

    def transition_value(self, dof: str) -> float:
        return float(self.transition_values.get(dof, 0.0))

    def is_automation(self, source: str) -> bool:
        return source in self.automation_sources


# ---------------------------------------------------------------------------- the transfer


def fresh(vessel_id: str, now_ms: int, term: str | None = None) -> State:
    """A newly (re)initialised store: a new term, generation 1, no map, the transition state. Every
    fence from a previous life is from a term that no longer exists."""
    return State(vessel_id=vessel_id, term=term or new_term(), boot_ms=now_ms)


def _expire(st: State, now_ms: int) -> State:
    r = st.reservation
    return replace(st, reservation=None) if r is not None and now_ms > r.deadline_ms else st


def _neutral_or_matched(st: State, pol: Policy, row: Mapping, now_ms: int) -> str | None:
    """The gate's final word (§4): the incoming source's latest sample is fresh, and either neutral
    or within the band of what the gate is emitting NOW. A manager-side check would be a
    time-of-check / time-of-use race."""
    s = st.latest.get((row.execution_source, row.dof))
    if s is None or now_ms - s.recv_ms > pol.max_axis_age_ms:
        return "no fresh sample from the incoming source"
    emitting = st.emitting.get(row.dof, pol.transition_value(row.dof))
    if abs(s.value_pct) <= pol.neutral_band_pct or abs(s.value_pct - emitting) <= pol.match_band_pct:
        return None
    return f"not neutral or matched ({s.value_pct:.0f} % against {emitting:.0f} % emitted)"


def prepare(st: State, pol: Policy, p: Proposal, now_ms: int) -> tuple[State, Ack]:
    """Validate against what the gate sees now and RESERVE (T, n+1). All or nothing (I10): one row
    that fails NAKs the whole transfer. Authority does not change here."""
    st = _expire(st, now_ms)
    if st.reservation is not None:
        return st, Ack(False, None, f"transfer {st.reservation.proposal.transfer_id} is reserved")
    if p.kind not in (GRANT, TAKE_CONTROL, STATION_HANDOVER, ABORT_AUTOMATION):
        return st, Ack(False, None, f"unknown transfer kind {p.kind!r}")
    if p.prepare_ttl_ms <= 0:
        return st, Ack(False, None, "a proposal names its TTL")
    bad: list[str] = []
    reasons: list[str] = []
    if p.kind == ABORT_AUTOMATION:
        if p.mappings:
            return st, Ack(False, None, "an abort proposes no map")
    else:
        dofs = [m.dof for m in p.mappings]
        missing = sorted(set(pol.dofs) - set(dofs))
        extra = sorted(set(dofs) - set(pol.dofs))
        if missing or extra or len(set(dofs)) != len(dofs):
            return st, Ack(False, None, "a transfer moves every motion DOF of this vessel, once each")
        for m in p.mappings:
            why = None
            if m.fence is not None:
                why = "a proposal names no fence: the gate mints it"
            elif pol.is_automation(m.execution_source):
                why = None
            elif not p.station or not m.execution_source.startswith(f"{p.station}/"):
                why = f"a human source must be at the active control station {p.station or '(none)'}"
            if why is None:
                why = _neutral_or_matched(st, pol, m, now_ms)
            if why is not None:
                bad.append(m.mapping_id or m.dof)
                reasons.append(f"{m.dof}: {why}")
        if bad:
            return st, Ack(False, None, "; ".join(reasons), tuple(bad))
    fence = Fence(st.term, st.generation + 1)
    return replace(st, reservation=Reservation(p, fence, now_ms + p.prepare_ttl_ms)), Ack(True, fence)


def commit(st: State, transfer_id: str, fence: Fence, now_ms: int) -> tuple[State, bool, str]:
    """Activate a reservation: map, station and fence together. A COMMIT for a reservation the gate
    no longer holds -- lapsed, or lost in a restart after the ACK -- changes nothing (I13)."""
    st = _expire(st, now_ms)
    r = st.reservation
    if r is None or r.proposal.transfer_id != transfer_id:
        return st, False, "no such reservation (lapsed, or the gate restarted after its ACK)"
    if fence != r.fence:
        return st, False, "the fence is not the one this reservation was ACKed with"
    p = r.proposal
    abort = p.kind == ABORT_AUTOMATION
    nxt = replace(st, generation=fence.generation, reservation=None,
                  mappings=tuple(replace(m, fence=fence) for m in p.mappings),
                  station=p.station if p.station else st.station,
                  in_transition=abort or not p.mappings,
                  transition_reason=f"abort automation by {p.requested_by}" if abort else "",
                  # The fence moved by a transfer: a cause of an older fence must never be read against it.
                  last_cause=None,
                  accepted={})  # every sample accepted under the old fence is from another authority
    return nxt, True, ""


def fence_alone(st: State, reason: str, cause: str = CAUSE_CONN_LAPSE, **who) -> State:
    """Authority must change and nobody else can do it: increment the generation here, drop the map and
    enter the transition state (§4). The manager reconciles afterwards, and reads WHY in `last_cause`:
    the kind, the fence it produced, and for an abort who asked (#290)."""
    generation = st.generation + 1
    return replace(st, generation=generation, mappings=(), in_transition=True, transition_reason=reason,
                   reservation=None, accepted={},
                   last_cause=FenceCause(cause, Fence(st.term, generation), reason=reason, **who))


# ---------------------------------------------------------------------------- observations


def observe_lease(st: State, slot: str, holder: str, token: str, ttl_ms: int, released: bool, now_ms: int,
                  site: str = "") -> State:
    """A lease record. On the ENGINEER slot, a token this gate has not seen is a new holder the manager
    authorised: the gate mints the next `plant_generation` and rebases the constraint in force onto it
    (`_new_engineer`). A released or lapsed ENGINEER lease mints nothing and lifts nothing."""
    leases = dict(st.leases)
    if released:
        leases.pop(slot, None)
    else:
        leases[slot] = Heard(holder, token, ttl_ms, now_ms)
    st = replace(st, leases=leases)
    if slot == "engineer" and not released and token and token != st.engineer_token:
        st = _new_engineer(st, holder, site, token)
    return st


def _new_engineer(st: State, holder: str, site: str, token: str) -> State:
    """I16: the incoming holder inherits the constraint UNCHANGED, at revision 1 under the new
    generation, in the same step that mints it. Only afterwards may they change anything."""
    st = replace(st, plant_generation=st.plant_generation + 1, engineer_token=token)
    c = st.constraint
    if c is not None:
        st = replace(st, constraint=replace(c, fence=st.plant_fence, revision=1, controller_id=holder, controller_site=site))
    return st


def observe_constraint(st: State, c: Constraint, now_ms: int) -> tuple[State, bool, str]:
    """A `propulsion_constraint` record: applied only under the current plant fence, at a revision newer
    than the one in force under it, and written by the live ENGINEER holder. Anything else is dropped
    -- so the outgoing engineer's record under the old generation can never relax the inherited one."""
    if c.fence != st.plant_fence:
        return st, False, "not under the current plant_generation"
    cur = st.constraint
    if cur is not None and cur.fence == c.fence and c.revision <= cur.revision:
        return st, False, f"revision {c.revision} is not newer than {cur.revision}"
    eng = st.leases.get("engineer")
    if eng is None or not eng.live(now_ms) or eng.holder != c.controller_id:
        return st, False, f"{c.controller_id} does not hold a live ENGINEER lease"
    return replace(st, constraint=c), True, ""


def _clamp(st: State, pol: Policy, dof: str, v: float) -> tuple[float, str]:
    """The plant constraint on one resolved value: an inhibited device sits at zero, a derated one at
    |value| <= max_power_pct. A DOF that drives no constrained device passes unchanged."""
    c = st.constraint
    device = pol.propulsion.get(dof)
    lim = c.limit(device) if c is not None and device else None
    if lim is None:
        return v, ""
    why = f" ({lim.reason})" if lim.reason else ""
    if lim.inhibit:
        return 0.0, f"inhibited by propulsion_constraint: {device}{why}"
    if lim.max_power_pct is not None and abs(v) > lim.max_power_pct:
        cap = float(lim.max_power_pct)
        return (cap if v > 0 else -cap), f"clamped by propulsion_constraint: {device} {cap:g} %{why}"
    return v, ""


def observe_session(st: State, role: str, holder: str, token: str, assignment_id: str, ttl_ms: int, released: bool,
                    now_ms: int) -> State:
    sessions = dict(st.sessions)
    if released:
        sessions.pop(role, None)
    else:
        sessions[role] = Heard(holder, token, ttl_ms, now_ms, assignment_id)
    return replace(st, sessions=sessions)


def observe_assignments(st: State, overall: AssignmentSeen | None, navigational: AssignmentSeen | None, issuer: str,
                        feed_ttl_ms: int, now_ms: int) -> State:
    """The `command_assignment` feed: the gate's own projection, for the direct abort."""
    assignments = {}
    if overall is not None:
        assignments[OVERALL] = overall
    if navigational is not None:
        assignments[NAVIGATIONAL] = navigational
    return replace(st, assignments=assignments, feed=Heard(issuer, "", feed_ttl_ms, now_ms))


def observe_safety(st: State, state: int, local_lockout: bool, now_ms: int) -> State:
    return replace(st, safety=SafetySeen(state, local_lockout, now_ms))


def observe_maintenance(st: State, state: int, now_ms: int) -> State:
    return replace(st, maintenance=MaintenanceSeen(int(state), now_ms))


def actuation_allowed(st: State, pol: Policy, now_ms: int) -> tuple[bool, str]:
    """I6: ACTUATION_ALLOWED ⇒ SafetyState CLEAR ∧ ¬local_lockout ∧ maintenance INACTIVE (∧ the lifecycle
    permits -- it has no subject yet, so it cannot refuse here). A silent supervisor is not a CLEAR one,
    and a silent manager is not an INACTIVE one: an unheard or stale maintenance_state is not INACTIVE
    (keelson protocols/maintenance.yaml)."""
    s = st.safety
    if s is not None and s.local_lockout:
        return False, "local lockout: remote operation inhibited"
    if pol.require_safety_state:  # off only in a rig with no supervisor, by policy; the lockout still binds
        if s is None:
            return False, "no safety_state heard"
        if now_ms - s.heard_ms > pol.safety_ttl_ms:
            return False, "safety_state is stale"
        if s.state != SAFETY_CLEAR:
            return False, "the vessel is not CLEAR"
    if pol.require_maintenance_state:
        m = st.maintenance
        if m is None:
            return False, "no maintenance_state heard"
        if now_ms - m.heard_ms > pol.maintenance_ttl_ms:
            return False, "maintenance_state is stale"
        if m.state != MAINTENANCE_INACTIVE:
            name = MAINTENANCE_STATES.get(m.state, f"state {m.state}")
            return False, f"maintenance_state is {name}: actuation inhibited until it is INACTIVE"
    return True, ""


def _holder_lease_live(st: State, m: Mapping, now_ms: int) -> bool:
    """`authorised?` -- the HOLDER's lease, not the source's: navigation/0 executing under a human's
    conn holds no lease of its own (I9)."""
    lease = st.leases.get("conn")
    return (lease is not None and lease.live(now_ms) and lease.holder == m.authority_holder
            and lease.token == m.authority_lease_id)


def on_sample(st: State, pol: Policy, s: Sample, now_ms: int) -> tuple[State, bool, str]:
    """Judge one candidate sample: ordered first (it is remembered for neutral-or-match whether or not
    it is mapped), then the per-DOF test. Returns (state, accepted, why not)."""
    known = st.streams.get(s.source)
    if known is None or known.stream_id != s.stream_id:
        # A new stream: a physical helm restarting at 80 % ahead must not regain motion by starting a
        # new sequence -- it waits for the neutral-reconnect interlock (I11).
        if not pol.is_automation(s.source) and abs(s.value_pct) > pol.neutral_band_pct:
            return st, False, "a new stream from a helm must start at neutral (neutral-reconnect interlock)"
    elif s.sequence <= known.last_sequence:
        return st, False, f"out of order: sequence {s.sequence} after {known.last_sequence}"
    st = replace(st, streams={**st.streams, s.source: Stream(s.stream_id, s.sequence)},
                 latest={**st.latest, (s.source, s.dof): s})

    m = st.mapping(s.dof)
    if m is None or st.in_transition:
        return st, False, "no map in force for this DOF"
    if s.source != m.execution_source:
        return st, False, f"not the mapped source ({m.execution_source})"  # sourced?
    if not _holder_lease_live(st, m, now_ms):
        return st, False, f"{m.authority_holder}'s conn lease is not live"  # authorised?
    if m.fence != st.fence or (s.fence is not None and s.fence != st.fence):
        return st, False, "fenced out: not the current (term, generation)"  # fenced?
    lo, hi, rate = m.envelope if m.envelope else (-100.0, 100.0, 0.0)
    if not lo <= s.value_pct <= hi:
        return st, False, f"outside the envelope [{lo}, {hi}]"  # enveloped?
    prev = st.accepted.get(s.dof)
    if rate > 0 and prev is not None and s.recv_ms > prev.recv_ms:
        if abs(s.value_pct - prev.value_pct) / ((s.recv_ms - prev.recv_ms) / 1000) > rate:
            return st, False, f"faster than {rate} %/s"
    # permitted? The station half holds by construction: the sample IS the mapped source, and
    # prepare() admitted a human source only at the proposal's station, which commit made current.
    allowed, why = actuation_allowed(st, pol, now_ms)
    if not allowed:
        return st, False, why
    return replace(st, accepted={**st.accepted, s.dof: s}), True, ""


# ---------------------------------------------------------------------------- guidance (I15)


def observe_navigation(st: State, revision: int, functions: dict) -> State:
    return replace(st, navigation=NavDoc(int(revision), dict(functions)))


def observe_navigation_state(st: State, producer: str, order_id: str | None, recv_ms: int) -> State:
    return replace(st, nav_states={**st.nav_states, producer: NavStateSeen(order_id or None, recv_ms)})


def _human_order_check(st: State, pol: Policy, g: Guidance, now_ms: int) -> str:
    """HUMAN_ORDER{order_id}: the id must be the human decision in force in the source's latest
    navigation_state, heard fresh on the gate's clock (keelson navigation_conduct, order_id). The
    source's records are the producers it spells: `navigation/0` or `navigation/0/...`. None fresh,
    or more than one, refuses: a current order cannot be told from a stale one, or from another's."""
    if not g.order_id:
        return "HUMAN_ORDER names no order id"
    mine = {p: n for p, n in st.nav_states.items() if p == g.source or p.startswith(f"{g.source}/")}
    fresh = {p: n for p, n in mine.items() if now_ms - n.recv_ms <= pol.navigation_state_max_age_ms}
    if not fresh:
        if not mine:
            return f"no navigation_state heard from {g.source}"
        age = min(now_ms - n.recv_ms for n in mine.values())
        return f"navigation_state from {g.source} not heard for {age} ms"
    if len(fresh) > 1:
        return f"more than one navigation_state under {g.source}: {', '.join(sorted(fresh))}"
    (producer, seen), = fresh.items()
    if seen.order_id is None:
        return f"{producer} reports no human order in force"
    if seen.order_id != g.order_id:
        return f"order {g.order_id} is not the one in force ({seen.order_id})"
    return ""


def _is(principal: str, source: str) -> bool:
    """A principal ("navigation/0@usv-alpha") names the process a source ("navigation/0") spells."""
    return bool(source) and (principal == source or principal.split("@", 1)[0] == source)


def decision_check(st: State, pol: Policy, g: Guidance, now_ms: int) -> str:
    """May this source DECIDE this? "" when it may, else why not. Independent of EXECUTION (I15).

    NAV_FUNCTION: the source holds the named function in the navigation authority document, at
    control method ≥ DELEGATED, under the document's current revision. HUMAN_ORDER: the order id is
    the one in force in the source's fresh navigation_state (_human_order_check)."""
    if g.decision == "human_order":
        return _human_order_check(st, pol, g, now_ms)
    if g.decision != "nav_function":
        return "no decision basis"
    doc = st.navigation
    if doc is None:
        return "no navigation authority document heard"
    held = doc.functions.get(g.function)
    if held is None or not _is(held[0], g.source):
        return f"{g.source} does not hold {g.function}"
    if held[1] < CONTROL_METHOD_DELEGATED:
        return f"{g.function} is held at method {held[1]}, below DELEGATED"
    if g.nav_revision != doc.revision:
        return f"decided under navigation revision {g.nav_revision}, the document is at {doc.revision}"
    return ""


def execution_check(st: State, g: Guidance, now_ms: int) -> str:
    """May this source EXECUTE this? "" when it may, else why not. Independent of DECISION (I15).

    DIRECT: the source holds the conn itself (full automation). CARRY_OUT: the DOF's mapping is the
    one named, and that mapping's execution_source is this source -- the human's conn, pre-authorised
    carry-out. The per-DOF test (on_sample) then still runs: the holder's lease, the fence, the
    envelope."""
    m = st.mapping(g.dof)
    if g.execution == "direct":
        lease = st.leases.get("conn")
        if lease is None or not lease.live(now_ms) or not _is(lease.holder, g.source):
            return f"DIRECT, but {g.source} does not hold the conn"
        return ""
    if g.execution == "carry_out":
        if m is None or not g.mapping_id or m.mapping_id != g.mapping_id:
            return f"CARRY_OUT names mapping {g.mapping_id or '(none)'}, which is not this DOF's"
        if m.execution_source != g.source:
            return f"CARRY_OUT, but the mapping's execution source is {m.execution_source}"
        return ""
    return "no execution basis"


def on_guidance(st: State, pol: Policy, g: Guidance, now_ms: int) -> tuple[State, bool, str]:
    """A guidance candidate: DECISION and EXECUTION, each refusing on its own, then the per-DOF test.
    A refused guidance sample is still remembered for ordering, so a later one is judged in order."""
    why = decision_check(st, pol, g, now_ms)
    if not why:
        why = execution_check(st, g, now_ms)
    sample = Sample(g.source, g.dof, g.value_pct, g.stream_id, g.sequence, g.recv_ms, sample_time_ns=g.sample_time_ns,
                    published_ns=g.published_ns, clock_quality=g.clock_quality, recv_wall_ns=g.recv_wall_ns)
    if why:
        # Refused to ACT on, but still what the source says now: neutral-or-match (prepare) reads it.
        # Without this, a carry-out mapping could never be committed -- prepare needs a fresh sample
        # from the incoming source, and that source's samples are refused until the mapping exists.
        known = st.streams.get(g.source)
        if known is None or known.stream_id != g.stream_id or g.sequence > known.last_sequence:
            st = replace(st, streams={**st.streams, g.source: Stream(g.stream_id, g.sequence)},
                         latest={**st.latest, (g.source, g.dof): sample})
        return st, False, why
    return on_sample(st, pol, sample, now_ms)


def resolve(st: State, pol: Policy, now_ms: int) -> tuple[State, tuple[Resolved, ...]]:
    """One cycle: every DOF's output. Under a lockout or a stop the gate emits NOTHING; in the
    transition state it emits the configured transition value -- never an implicit "nobody"."""
    allowed, why = actuation_allowed(st, pol, now_ms)
    rows, emitting = [], dict(st.emitting)
    for dof in pol.dofs:
        acc = st.accepted.get(dof)
        m = st.mapping(dof)
        requested = acc.value_pct if acc is not None else None
        if not allowed:
            rows.append(Resolved(dof, requested, None, "", INHIBITED, why))
            emitting[dof] = _clamp(st, pol, dof, pol.transition_value(dof))[0]
            continue
        if st.in_transition or m is None:
            v, clamped = _clamp(st, pol, dof, pol.transition_value(dof))
            rows.append(Resolved(dof, requested, v, "", TRANSITION, _why(st.transition_reason or pol.transition_name, clamped)))
            emitting[dof] = v
            continue
        if acc is None or now_ms - acc.recv_ms > pol.max_axis_age_ms or acc.source != m.execution_source:
            v, clamped = _clamp(st, pol, dof, pol.transition_value(dof))
            rows.append(Resolved(dof, requested, v, m.execution_source, STALE, _why("the mapped source is silent", clamped)))
            emitting[dof] = v
            continue
        v, clamped = _clamp(st, pol, dof, acc.value_pct)
        rows.append(Resolved(dof, requested, v, acc.source, ACTIVE, clamped, provenance=acc))
        emitting[dof] = v
    return replace(st, emitting=emitting), tuple(rows)


def _why(reason: str, clamped: str) -> str:
    return f"{reason}; {clamped}" if clamped else reason


def tick(st: State, pol: Policy, now_ms: int) -> tuple[State, str]:
    """The watchdogs this slice owns: an orphaned reservation lapses, and a map whose holder's conn
    lease has lapsed is fenced alone (I9). Returns (state, why it fenced or "")."""
    st = _expire(st, now_ms)
    if st.mappings and not st.in_transition:
        holders = {(m.authority_holder, m.authority_lease_id) for m in st.mappings}
        lease = st.leases.get("conn")
        unheard_in_grace = lease is None and now_ms - st.boot_ms < pol.lease_grace_ms
        if not unheard_in_grace and not any(_holder_lease_live(st, m, now_ms) for m in st.mappings):
            who = ", ".join(sorted(h for h, _ in holders))
            reason = f"the conn lease of {who} lapsed"
            return fence_alone(st, reason), reason
    return st, ""


# ---------------------------------------------------------------------------- the direct abort


@dataclass(frozen=True)
class DirectAbort:
    requester: str
    assignment_id: str
    assignment_revision: int
    session_token: str
    reason: str = ""


def abort_direct(st: State, d: DirectAbort, now_ms: int) -> tuple[State, bool, str]:
    """The one authority-changing call the gate takes from a human without the manager (§4). The gate
    verifies the caller on its OWN projection: the NAVIGATIONAL or OVERALL assignee, the current
    revision, and that assignment's session token.

    A SESSION WITH THE MANAGER DEAD. Sessions are renewed through the manager and republished by it,
    so with the manager gone every session lapses in this projection although its holder is still at
    the console. While the feed is live, the session must be live; once the feed is stale, the last
    session heard for that assignment is accepted. This call only ever makes the vessel MORE
    inhibited -- it fences the automation and grants nothing -- so it does not fail closed the way
    the supervisor's RESET does (I7′)."""
    feed_live = st.feed is not None and st.feed.live(now_ms)
    for role in (NAVIGATIONAL, OVERALL):
        a = st.assignments.get(role)
        if a is None or a.assignee != d.requester:
            continue
        if a.assignment_id != d.assignment_id or a.revision != d.assignment_revision:
            return st, False, f"acting under a stale assignment: {role.upper()} is at revision {a.revision}"
        s = st.sessions.get(role)
        if s is None or s.token != d.session_token or s.assignment_id != a.assignment_id:
            return st, False, f"no session of the {role.upper()} assignment with this token"
        if feed_live and not s.live(now_ms):
            return st, False, f"the {role.upper()} session has lapsed"
        return fence_alone(st, f"abort automation by {d.requester} (direct){': ' + d.reason if d.reason else ''}",
                           CAUSE_ABORT, requested_by=d.requester, assignment_id=a.assignment_id,
                           assignment_revision=a.revision), True, ""
    return st, False, "not the NAVIGATIONAL_COMMAND or OVERALL_COMMAND assignee"
