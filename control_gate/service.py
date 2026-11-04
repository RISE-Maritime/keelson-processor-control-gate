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

from . import model
from .model import Ack, DirectAbort, Fence, Policy, Proposal, Sample

log = logging.getLogger(__name__)


def mono_ms() -> int:
    return int(time.monotonic() * 1000)


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

    def observe_lease(self, slot, holder, token, ttl_ms, released):
        with self.lock:
            self.state = model.observe_lease(self.state, slot, holder, token, ttl_ms, released, self.now_ms())

    def observe_session(self, role, holder, token, assignment_id, ttl_ms, released):
        with self.lock:
            self.state = model.observe_session(self.state, role, holder, token, assignment_id, ttl_ms, released, self.now_ms())

    def observe_assignments(self, overall, navigational, issuer, feed_ttl_ms):
        with self.lock:
            self.state = model.observe_assignments(self.state, overall, navigational, issuer, feed_ttl_ms, self.now_ms())

    def observe_safety(self, state, local_lockout):
        with self.lock:
            self.state = model.observe_safety(self.state, state, local_lockout, self.now_ms())

    def sample(self, s: Sample) -> tuple[bool, str]:
        with self.lock:
            self.state, ok, why = model.on_sample(self.state, self.policy, s, self.now_ms())
            return ok, why

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
        self.actuator.apply(rows)
        return rows
