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

from . import model
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

    def boot(self) -> model.State:
        with self.lock:
            st, why = self.store.load(self.vessel_id, self.now_ms())
            if why:
                log.warning("REINITIALISED with a new term (%s): every fence of the old term is dead", why)
                self.store.save(st)
                self.events.append(f"reinitialised: {why}")
            self.state = st
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
            st, ok, detail = model.abort_direct(self.state, d, self.now_ms())
            if ok:
                self.store.save(st)
                self.events.append(f"direct abort by {d.requester} at generation {st.generation}")
                log.warning("ABORT AUTOMATION (direct) by %s: fenced at generation %d", d.requester, st.generation)
            self.state = st
            return ok, detail, st.fence

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
            st, ok, why = model.observe_constraint(self.state, c, self.now_ms())
            if ok:
                self.store.save(st)
                log.info("propulsion_constraint revision %d by %s: %s", c.revision, c.controller_id,
                         ", ".join(f"{d.device_id} {'inhibited' if d.inhibit else d.max_power_pct}" for d in c.devices) or "none")
            else:
                log.info("propulsion_constraint revision %d by %s dropped: %s", c.revision, c.controller_id, why)
            self.state = st
            return ok, why

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

    def observe_safety(self, state, local_lockout):
        with self.lock:
            self.state = model.observe_safety(self.state, state, local_lockout, self.now_ms())

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

    def observe_navigation(self, revision, functions):
        with self.lock:
            self.state = model.observe_navigation(self.state, revision, functions)

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
            cycle = Cycle(rows, st.fence, self.sequence, tuple(groups), allowed, why, st.in_transition, st.transition_reason, now)
        self.actuator.apply(rows)
        self.last_cycle = cycle
        return rows
