"""The gate's sequence: boot, the control_gate/v1 calls, observations, samples, the cycle. No Zenoh
here; bin/ wires it.

ONE LOCK DECIDES: every call -- an RPC on a queryable thread, an observation from a subscriber, the
cycle -- takes the same lock, so the gate's state moves one decision at a time.

PERSIST BEFORE REPLY (I13): `commit` persists the activated fence before it answers `applied`;
`fence_alone` and the direct abort persist before the next cycle emits under the new fence.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from . import attestation, model
from .model import Ack, DirectAbort, Fence, Policy, Proposal, Sample

log = logging.getLogger(__name__)


def mono_ms() -> int:
    return int(time.monotonic() * 1000)


@dataclass(frozen=True)
class Cycle:
    """One resolution, with everything ResolvedControl carries (codec.resolved_control)."""

    rows: tuple
    fence: Fence
    sequence: int
    groups: tuple  # (group_id, sequence)
    actuation_allowed: bool
    actuation_reason: str
    in_transition: bool
    transition_reason: str
    now_ms: int | None = None  # the cycle's monotonic time: t4, against which a row's hold is measured
    attestation_challenge: bytes = b""  # the newest the direct abort accepts (#316)


class Gate:
    def __init__(self, *, store, policy: Policy, vessel_id: str, actuator, now_ms=mono_ms):
        self.store = store
        self.policy = policy
        self.vessel_id = vessel_id
        self.actuator = actuator
        self.now_ms = now_ms
        self.lock = threading.RLock()
        self.state: model.State | None = None
        self.events: list[str] = []  # what changed authority, for the log and the tests
        self.sequence = 0  # ResolvedControl's, per incarnation of this process
        self.group_sequences: dict[str, int] = {}
        self.last_cycle: Cycle | None = None
        self.helm_health: dict[str, int] = {}  # source -> its last accepted sample's own device_health (#309)
        self.outbox: list = []  # plant constraints the gate rebased, for bin/ to publish (take_publications)
        # The direct abort's attestation (crowsnest-dev#316): the challenges this gate drew and the
        # sequences it accepted. In memory only, so a restart makes every earlier attestation stale.
        self.challenges = attestation.Challenges(policy.challenge_ttl_ms)
        self.seen: dict = {}

    def boot(self) -> model.State:
        with self.lock:
            st, why = self.store.load(self.vessel_id, self.now_ms())
            if why:
                log.warning("REINITIALISED with a new term (%s): every fence of the old term is dead", why)
                self.store.save(st)
                self.events.append(f"reinitialised: {why}")
            self.state = st
            self.challenges.rotate(self.now_ms())
            log.info("fence (%s…, %d), %s", st.term[:8], st.generation,
                     "transition" if st.in_transition else f"{len(st.mappings)} DOF(s) mapped at {st.station}")
            return st

    # ------------------------------------------------------------------ control_gate/v1

    def prepare(self, p: Proposal) -> Ack:
        with self.lock:
            self.state, ack = model.prepare(self.state, self.policy, p, self.now_ms())
            log.info("PREPARE %s (%s, by %s): %s", p.transfer_id, p.kind, p.requested_by,
                     f"reserved generation {ack.fence.generation}" if ack.accepted else f"NAK — {ack.reason}")
            return ack

    def commit(self, transfer_id: str, fence: Fence) -> tuple[bool, str, Fence]:
        with self.lock:
            st, applied, reason = model.commit(self.state, transfer_id, fence, self.now_ms())
            if applied:
                self.store.save(st)  # durable before the reply says so
                self.events.append(f"committed {transfer_id} at generation {st.generation}")
            self.state = st
            log.info("COMMIT %s: %s", transfer_id, "applied" if applied else f"refused — {reason}")
            return applied, reason, st.fence

    def exposed(self) -> model.State:
        with self.lock:
            self.state = model._expire(self.state, self.now_ms())
            return self.state

    def abort_direct(self, d: DirectAbort) -> tuple[bool, str, Fence]:
        with self.lock:
            principal, refused = self._attest(d)
            if refused:
                log.warning("direct abort by %s REFUSED: %s", d.requester, refused)
                return False, refused, self.state.fence
            st, ok, detail = model.abort_direct(self.state, d, self.now_ms())
            if ok:
                self.store.save(st)
                by = f" (attested by {principal['service']} for {principal['station'] or principal['site']})" if principal else ""
                self.events.append(f"direct abort by {d.requester} at generation {st.generation}{by}")
                log.warning("ABORT AUTOMATION (direct) by %s%s: fenced at generation %d", d.requester, by, st.generation)
            self.state = st
            return ok, detail, st.fence

    def _attest(self, d: DirectAbort) -> tuple[dict | None, str]:
        """(principal, "") when the call may proceed, else (None, why refused). A present attestation is
        always verified; a missing one is refused only where the policy requires one."""
        att = d.attestation
        if att is not None:
            v = attestation.verify(att, d.request, self.policy.attestation_keys, self.challenges, self.seen,
                                   self.now_ms())
            return (v.principal, "") if v.ok else (None, v.outcome)
        if self.policy.attestation_required:
            return None, attestation.MISSING
        return None, ""

    def attestation_challenge(self) -> bytes:
        """The newest challenge, drawn anew every `challenge_rotate_ms` on the gate's clock."""
        with self.lock:
            now = self.now_ms()
            drawn = self.challenges.drawn_ms()
            if drawn is None or now - drawn >= self.policy.challenge_rotate_ms:
                self.challenges.rotate(now)
            return self.challenges.newest(now)

    # ------------------------------------------------------------------ what the gate hears

    def observe_lease(self, slot, holder, token, ttl_ms, released, site=""):
        with self.lock:
            before = self.state.plant_generation
            st = model.observe_lease(self.state, slot, holder, token, ttl_ms, released, self.now_ms(), site)
            if st.plant_generation != before:
                # A new ENGINEER holder: the generation and the rebased constraint are durable before
                # anything is published or clamped under them.
                self.store.save(st)
                self.events.append(f"plant_generation {st.plant_generation} for {holder}")
                log.warning("NEW ENGINEER %s: plant_generation %d%s", holder, st.plant_generation,
                            "; the constraint in force is inherited at revision 1" if st.constraint else "")
                if st.constraint is not None:
                    self.outbox.append(st.constraint)
            self.state = st

    def observe_constraint(self, c: model.Constraint | None) -> tuple[bool, str]:
        with self.lock:
            if c is None:
                return False, "no fence"
            held = self.state.plant_hold
            st, ok, why = model.observe_constraint(self.state, c, self.now_ms())
            if ok and held:
                log.warning("the ENGINEER %s confirmed the plant constraint at revision %d: the hold is lifted",
                            c.controller_id, c.revision)
                self.events.append(f"plant hold lifted by {c.controller_id}")
            if ok:
                self.store.save(st)
                log.info("propulsion_constraint revision %d by %s: %s", c.revision, c.controller_id,
                         ", ".join(f"{d.device_id} {'inhibited' if d.inhibit else d.max_power_pct}" for d in c.devices) or "none")
            else:
                log.info("propulsion_constraint revision %d by %s dropped: %s", c.revision, c.controller_id, why)
            self.state = st
            return ok, why

    def restore_constraints(self, records: list, invalid: list[str]) -> str:
        """What storage held for `propulsion_constraint/{vessel}`, read once after boot (#345). On a new
        term it is validated and rebased, and the rebased record is published so storage holds it under
        the new term; on a surviving store it is an ordinary record, judged by `observe_constraint`."""
        with self.lock:
            st, what = model.restore_constraints(self.state, records, invalid, self.now_ms())
            if not what:
                for c in records:
                    self.observe_constraint(c)
                return ""
            self.store.save(st)  # durable before anything is clamped or published under it
            self.state = st
            if what == "restored":
                devs = ", ".join(f"{d.device_id} {'inhibited' if d.inhibit else d.max_power_pct}" for d in st.constraint.devices)
                log.warning("RESTORED propulsion_constraint after a new term (%s), rebased at revision 1; "
                            "actuation held until the ENGINEER confirms", devs or "no devices")
                self.outbox.append(st.constraint)
            elif what == "invalid":
                log.error("the stored propulsion_constraint cannot be restored: %s", st.plant_hold)
            else:
                log.info("no stored propulsion_constraint after a new term: none in force")
            self.events.append(f"plant constraint {what} after a new term")
            return what

    def observe_motion(self, m: model.MotionLimit | None) -> tuple[bool, str]:
        """A motion_constraint record, by the interim's rules (model.observe_motion). Persisted when
        applied: a latched slot outlives this gate's restart as it outlives its writer (I5)."""
        with self.lock:
            if m is None:
                return False, "no fence"
            held = self.state.motion_hold
            st, ok, why = model.observe_motion(self.state, m)
            if ok:
                self.store.save(st)
                limits = ", ".join(x for x in (
                    f"speed <= {m.max_speed_knots:g} kn" if m.max_speed_knots is not None else "",
                    f"rot <= {m.max_rot_degps:g} deg/s" if m.max_rot_degps is not None else "",
                    f"heading {m.heading_band[0]:g}-{m.heading_band[1]:g}" if m.heading_band is not None else "") if x) or "none"
                log.warning("motion_constraint %s revision %d by %s%s: %s%s", m.payload_id, m.revision, m.writer_id,
                            " LATCHED" if m.latched else "", limits,
                            "" if m.latched or not m.has_limit() else f" -- reported, NOT enforced ({model.MOTION_UNITS_ISSUE})")
                if held and not st.motion_hold:
                    self.events.append("motion hold lifted")
            else:
                log.info("motion_constraint %s revision %d dropped: %s", m.payload_id, m.revision, why)
            self.state = st
            return ok, why

    def restore_motion(self, records: list, invalid: list) -> str:
        """What storage held for `motion_constraint/{vessel}/*`, read once after boot (I16). On a new term
        the valid slots are restored as they were and an invalid one holds actuation; on a surviving
        store each record is an ordinary one, judged by `observe_motion`."""
        with self.lock:
            st, what = model.restore_motion(self.state, records, invalid)
            if not what:
                for m in records:
                    self.observe_motion(m)
                return ""
            self.store.save(st)
            self.state = st
            if what == "invalid":
                log.error("a stored motion_constraint cannot be restored: %s", st.motion_hold)
            elif what == "restored":
                log.warning("RESTORED motion_constraint after a new term: %s", ", ".join(
                    f"{p}{' LATCHED' if m.latched else ''}" for p, m in sorted(st.motion.items())))
            self.events.append(f"motion constraints {what} after a new term")
            return what

    def take_publications(self) -> list:
        with self.lock:
            out, self.outbox = self.outbox, []
            return out

    def observe_session(self, role, holder, token, assignment_id, ttl_ms, released):
        with self.lock:
            self.state = model.observe_session(self.state, role, holder, token, assignment_id, ttl_ms, released, self.now_ms())

    def observe_assignments(self, overall, navigational, issuer, feed_ttl_ms):
        with self.lock:
            self.state = model.observe_assignments(self.state, overall, navigational, issuer, feed_ttl_ms, self.now_ms())

    def observe_safety(self, state, local_lockout, start_warning=None):
        with self.lock:
            self.state = model.observe_safety(self.state, state, local_lockout, self.now_ms(), start_warning)

    def observe_maintenance(self, state):
        with self.lock:
            self.state = model.observe_maintenance(self.state, state, self.now_ms())

    def sample(self, s: Sample) -> tuple[bool, str]:
        with self.lock:
            self.state, ok, why = model.on_sample(self.state, self.policy, s, self.now_ms())
            if ok:
                self._note_health(s)
            return ok, why

    def _note_health(self, s: Sample) -> None:
        """Log a helm entering or leaving its own DEGRADED (crowsnest-dev#309), once per transition, so
        the log says why its samples are late. Reported, never judged: the sample is already accepted."""
        was = self.helm_health.get(s.source, 0)
        self.helm_health[s.source] = s.device_health
        if s.device_health == was:
            return
        name = model.HEALTH_LEVELS.get(s.device_health, f"level {s.device_health}")
        if s.device_health == model.HEALTH_DEGRADED:
            log.warning("helm %s reports itself DEGRADED (profile %r): still judged on receipt freshness alone",
                        s.source, s.profile_version)
        elif was == model.HEALTH_DEGRADED:
            log.info("helm %s reports itself %s again", s.source, name)

    def guidance(self, g: model.Guidance) -> tuple[bool, str]:
        with self.lock:
            self.state, ok, why = model.on_guidance(self.state, self.policy, g, self.now_ms())
            if not ok:
                log.debug("guidance %s %s refused: %s", g.source, g.dof, why)
            return ok, why

    def observe_navigation(self, revision, functions, assignment_id: str = "") -> tuple[bool, str]:
        with self.lock:
            st, ok, why = model.observe_navigation(self.state, revision, functions, assignment_id)
            if ok:
                self.store.save(st)  # the floor is durable: a restart must not take an older document back
            else:
                log.info("navigation authority dropped: %s", why)
            self.state = st
            return ok, why

    def observe_navigation_state(self, producer: str, order_id: str | None):
        with self.lock:
            self.state = model.observe_navigation_state(self.state, producer, order_id, self.now_ms())

    # ------------------------------------------------------------------ the cycle

    def cycle(self):
        """The watchdogs, then one resolution, then the actuator."""
        with self.lock:
            now = self.now_ms()
            st, fenced = model.tick(self.state, self.policy, now)
            if fenced:
                self.store.save(st)  # before anything is emitted under the new fence
                self.events.append(f"fenced alone: {fenced}")
                log.warning("FENCED ALONE at generation %d: %s", st.generation, fenced)
            st, rows = model.resolve(st, self.policy, now)
            self.state = st
            allowed, why = model.actuation_allowed(st, self.policy, now)
            self.sequence += 1
            groups = []
            for gid in sorted(self.policy.groups):
                self.group_sequences[gid] = self.group_sequences.get(gid, 0) + 1
                groups.append((gid, self.group_sequences[gid]))
            cycle = Cycle(rows, st.fence, self.sequence, tuple(groups), allowed, why, st.in_transition, st.transition_reason, now,
                          self.attestation_challenge())
        self.actuator.apply(rows)
        self.last_cycle = cycle
        return rows
