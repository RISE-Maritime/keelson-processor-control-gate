"""The gate's control matrix and `safe_state_policy` timings, from a local JSON file -- platform
registry data that changes only in maintenance (command_authority `not_solved`, "The values")."""

from __future__ import annotations

import json
from pathlib import Path

from .model import Policy


def parse(d: dict) -> Policy:
    dofs = tuple(d.get("dofs") or ())
    if not dofs:
        raise ValueError("a policy names the vessel's motion DOFs")
    transition = d.get("transition") or {}
    values = {str(k): float(v) for k, v in (transition.get("values") or {}).items()}
    if not set(values) <= set(dofs):
        raise ValueError("a transition value for a DOF the vessel does not have")
    return Policy(
        command_profile=d.get("command_profile", "test"),
        dofs=dofs,
        automation_sources=frozenset(d.get("automation_sources") or ()),
        max_axis_age_ms=int(d.get("max_axis_age_ms", 1000)),
        neutral_band_pct=float(d.get("neutral_band_pct", 5.0)),
        match_band_pct=float(d.get("match_band_pct", 5.0)),
        transition_name=str(transition.get("name", "neutral")),
        transition_values=values,
        safety_ttl_ms=int(d.get("safety_ttl_ms", 15000)),
        require_safety_state=bool(d.get("require_safety_state", True)),
        lease_grace_ms=int(d.get("lease_grace_ms", 5000)),
        groups={str(k): tuple(v) for k, v in (d.get("groups") or {}).items()},
    )


def load(path: Path) -> Policy:
    return parse(json.loads(Path(path).read_text()))
