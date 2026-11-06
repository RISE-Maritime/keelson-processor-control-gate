# CLAUDE.md

The onboard motion gate (README). **Safety-critical: every change is reviewed by a human before it is
merged or run anywhere but the isolated tests.** Every commit message carries a
`Safety-review: required` trailer naming what changed.

## Rules that must not be broken

- **Only the gate mints a motion fence**, both halves: the term when the store is (re)initialised, the
  generation on an ACK (reserved), a COMMIT (current) or alone (`fence_alone`, the direct abort).
- **Authority changes only on COMMIT**, never on the gate's own ACK; persist before replying `applied`.
- **Every line of the test** -- fresh ∧ ordered ∧ authorised ∧ fenced ∧ sourced ∧ enveloped ∧
  permitted -- holds before a sample reaches actuation. The dead-man (`max_axis_age_ms`) is never
  disableable.
- **Never less inhibited by accident:** a silent supervisor is not a CLEAR one; under a stop or the
  local lockout the gate emits nothing; a lost store is a new term and the transition state.
- **A plant constraint is never relaxed by a change of holder.** A new ENGINEER token mints
  `plant_generation` and rebases the constraint in force UNCHANGED at revision 1, persisted before it is
  published or clamped under. A record applies only under the current plant fence, newer, from the live
  holder. Silence lifts nothing.
- **Pure decisions.** `control_gate/model.py` has no clock, no store and no Zenoh.
- **One lock decides** (service.Gate): every RPC, observation and cycle.
- **Tests never touch a network**, and nothing runs against a vessel or the shared Zenoh network.
- **keelson is an exact published pin** (README, "Developing before the build").

## Commands

```bash
.venv/bin/python -m pytest -q          # all
.venv/bin/python -m pytest -m e2e    # the CLI (and the manager's) on an isolated loopback router
.venv/bin/ruff check .
```
