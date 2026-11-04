"""Where resolved values go. This cut has only a logging actuator: the gate emits nothing to a vessel
until mavlink2keelson / Slipway consume `ResolvedControl` (crowsnest-dev #85, item 14). Each real
output is its own change, with its own safety review."""

from __future__ import annotations

import logging

log = logging.getLogger(__name__)


class LoggingActuator:
    def __init__(self):
        self.last = None
        self.applied = []

    def apply(self, rows) -> None:
        summary = tuple((r.dof, r.value, r.control_state) for r in rows)
        if summary != self.last:
            log.info("resolved: %s", ", ".join(f"{d}={'—' if v is None else f'{v:.0f}%'} {s}" for d, v, s in summary))
            self.last = summary
        self.applied.append(rows)
