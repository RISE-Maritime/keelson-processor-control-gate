"""The gate's own durable store: the term, the generation, the map in force, the station and the
transition state, in one sqlite database (stdlib, WAL, synchronous=FULL).

PERSIST BEFORE REPLY. A COMMIT is applied, persisted, and only then answered `applied`; a fence the
gate incremented alone is persisted before anything is emitted under it. So the fence the gate
exposes after a restart is the fence it last activated (I13).

A STORE THAT CANNOT BE TRUSTED IS A NEW TERM. Missing, corrupt, of an unknown schema or failing its
hash: the gate reinitialises with a fresh random term, no map and the transition state. Every command
from before -- (T, anything) -- is then from a term that no longer exists and is rejected (§4). That
is the safe direction here, unlike the manager, which refuses to start: an enforcer that forgets
grants nothing. A reservation is never persisted: it does not survive the gate.

A new term is also born holding actuation for the plant constraint (model.fresh, #345), and that hold is
persisted with the rest: a gate restarted before the ENGINEER confirmed a restore keeps waiting."""

from __future__ import annotations

import hashlib
import json
import sqlite3
from pathlib import Path

from .model import Constraint, DeviceLimit, Fence, FenceCause, Mapping, State, fresh

SCHEMA_VERSION = 1


def _canonical(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"))


def _digest(s: str) -> str:
    return hashlib.sha256(f"v{SCHEMA_VERSION}:{s}".encode()).hexdigest()


def _mapping_json(m: Mapping) -> dict:
    return {"dof": m.dof, "authority_holder": m.authority_holder, "authority_lease_id": m.authority_lease_id,
            "execution_source": m.execution_source, "envelope": list(m.envelope), "group_id": m.group_id,
            "mapping_id": m.mapping_id, "fence": None if m.fence is None else [m.fence.term, m.fence.generation]}


def _constraint_json(c: Constraint | None):
    if c is None:
        return None
    return {"fence": [c.fence.term, c.fence.generation], "revision": c.revision, "controller_id": c.controller_id,
            "controller_site": c.controller_site,
            "devices": [[d.device_id, d.max_power_pct, d.inhibit, d.reason] for d in c.devices]}


def _constraint(d) -> Constraint | None:
    if d is None:
        return None
    return Constraint(Fence(d["fence"][0], int(d["fence"][1])), int(d["revision"]), d["controller_id"], d["controller_site"],
                      tuple(DeviceLimit(x[0], None if x[1] is None else float(x[1]), bool(x[2]), x[3]) for x in d["devices"]))


def _cause_json(c: FenceCause | None):
    if c is None:
        return None
    return {"kind": c.kind, "fence": [c.fence.term, c.fence.generation], "requested_by": c.requested_by,
            "assignment_id": c.assignment_id, "assignment_revision": c.assignment_revision, "reason": c.reason}


def _cause(d) -> FenceCause | None:
    if d is None:
        return None
    return FenceCause(d["kind"], Fence(d["fence"][0], int(d["fence"][1])), d["requested_by"], d["assignment_id"],
                      int(d["assignment_revision"]), d["reason"])


def _mapping(d: dict) -> Mapping:
    f = d.get("fence")
    return Mapping(d["dof"], d["authority_holder"], d["authority_lease_id"], d["execution_source"], tuple(d["envelope"]),
                   d.get("group_id", ""), d.get("mapping_id", ""), None if f is None else Fence(f[0], int(f[1])))


class Store:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # The RPC handlers run on zenoh's queryable threads; every access is under service.Gate's lock.
        self.db = sqlite3.connect(self.path, isolation_level=None, check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.execute("CREATE TABLE IF NOT EXISTS meta (k TEXT PRIMARY KEY, v TEXT NOT NULL)")

    def close(self):
        self.db.close()

    def load(self, vessel_id: str, now_ms: int) -> tuple[State, str]:
        """The stored state, or a FRESH one with the reason it is fresh ("" when restored)."""
        try:
            if self.db.execute("PRAGMA quick_check").fetchone()[0] != "ok":
                return fresh(vessel_id, now_ms), "the store failed its integrity check"
            rows = dict(self.db.execute("SELECT k, v FROM meta").fetchall())
        except sqlite3.DatabaseError as e:
            return fresh(vessel_id, now_ms), f"the store is unreadable: {e}"
        if "state" not in rows:
            return fresh(vessel_id, now_ms), "no stored state"
        if rows.get("schema") != str(SCHEMA_VERSION):
            return fresh(vessel_id, now_ms), f"unknown store schema {rows.get('schema')!r}"
        if _digest(rows["state"]) != rows.get("state_hash"):
            return fresh(vessel_id, now_ms), "the stored state does not match its hash"
        try:
            d = json.loads(rows["state"])
            if d["vessel_id"] != vessel_id:
                return fresh(vessel_id, now_ms), f"the store is for vessel {d['vessel_id']!r}"
            st = State(vessel_id=vessel_id, term=d["term"], generation=int(d["generation"]),
                       mappings=tuple(_mapping(m) for m in d["mappings"]), station=d["station"],
                       in_transition=bool(d["in_transition"]), transition_reason=d["transition_reason"], boot_ms=now_ms,
                       # The plant domain (crowsnest-dev#270). Absent in a store written before it: the defaults,
                       # never a new term -- a schema bump would reinitialise the gate.
                       plant_generation=int(d.get("plant_generation", 1)), engineer_token=str(d.get("engineer_token", "")),
                       constraint=_constraint(d.get("constraint")),
                       # Why it last fenced alone (#290); absent in an older store, never a new term.
                       last_cause=_cause(d.get("last_cause")),
                       # A restore still waiting for the ENGINEER (#345) waits across a restart too.
                       plant_hold=str(d.get("plant_hold", "")))
        except (ValueError, KeyError, TypeError) as e:
            return fresh(vessel_id, now_ms), f"the stored state is inconsistent: {e}"
        if len(st.term) != 32:
            return fresh(vessel_id, now_ms), "the stored term is not 16 bytes of hex"
        return st, ""

    def save(self, st: State) -> None:
        body = _canonical({"vessel_id": st.vessel_id, "term": st.term, "generation": st.generation,
                           "mappings": [_mapping_json(m) for m in st.mappings], "station": st.station,
                           "in_transition": st.in_transition, "transition_reason": st.transition_reason,
                           "plant_generation": st.plant_generation, "engineer_token": st.engineer_token,
                           "constraint": _constraint_json(st.constraint),
                           "last_cause": _cause_json(st.last_cause), "plant_hold": st.plant_hold})
        self.db.execute("BEGIN IMMEDIATE")
        try:
            self.db.executemany("INSERT INTO meta (k, v) VALUES (?, ?) ON CONFLICT(k) DO UPDATE SET v=excluded.v",
                                [("schema", str(SCHEMA_VERSION)), ("state", body), ("state_hash", _digest(body))])
            self.db.execute("COMMIT")
        except BaseException:
            self.db.execute("ROLLBACK")
            raise
