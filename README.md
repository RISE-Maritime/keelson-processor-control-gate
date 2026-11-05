# keelson-processor-control-gate

The onboard **motion gate**, one per vessel: the enforcer between every candidate source (a station's
helm, the navigator's `desired_*`) and the actuators. It owns the motion fence. It takes the authority
manager's transfer as a PREPARE, reserves `(T, n+1)`, and changes authority only on the COMMIT. Every
sample has to pass the gate's own test before it reaches actuation.

- **Protocol:** keelson `protocols/command_authority.yaml`, `ControlMapping.proto` and
  `interfaces/ControlGate.proto` (keelson #331 and #359), plus `SafetyState.proto` (#332).
- **Design:** crowsnest-dev `docs/COMMAND-ARCHITECTURE.md` §2.1, §3 and §4. The work item is
  crowsnest-dev #85.
- **Its caller:** `keelson-processor-control-authority`, the onboard authority manager.

> **Status: it gates over the bus, and actuates nothing.**
> - It hears its candidates (`control_candidate`, `guidance_candidate`) and publishes
>   `resolved_control` every cycle.
> - Its own actuator still only logs. A downstream connector consuming `resolved_control` is its
>   own change.
> - Never run this against a real vessel or a shared network.

## What it does

- **It owns both halves of the fence.** The term is a random 128-bit value drawn when the store is
  (re)initialised, and never otherwise. The generation changes in three places:
  - it is reserved on an ACK;
  - it becomes current only on COMMIT;
  - it is incremented by the gate alone when a mapping's holder lease lapses, or on the direct abort.

  A reservation lapses on `prepare_ttl_ms`, measured on the gate's own clock, and does not survive a
  restart. The activated fence is persisted before the gate replies `applied`.
- **Neutral-or-match is its final word, and a transfer is all or nothing (I10).** Each incoming
  source's latest sample must be fresh, and either neutral or within the band of what the gate is
  emitting now. Each transfer must also move every DOF, with human sources at the proposed station.
  A row that names a fence is refused (I2).
- **The per-DOF test (§4).** A sample drives a DOF only if all of these hold:
  - **ordered:** the stream is the known one and the sequence is rising. A new stream from a helm
    must start at neutral (I11).
  - **sourced:** it comes from the mapped `execution_source`.
  - **authorised:** the HOLDER's conn lease is live on the gate's clock. That lease is matched by
    `authority_lease_id` against the published token, and it is the holder's, not the source's (I9).
  - **fenced:** the map's fence is current, and so is the sample's, if it carries one.
  - **enveloped:** the value is within `[min, max]` and the rate limit.
  - **permitted:** actuation is allowed.
  - **fresh:** this is judged when the gate resolves, against the dead-man `max_axis_age_ms`.
- **ACTUATION_ALLOWED ⇒ CLEAR ∧ ¬local_lockout (I6).** A silent supervisor is not a CLEAR one. Under
  a stop or the lockout, the gate emits nothing at all. In the transition state it emits the
  configured value, never an implicit "nobody". A commit forgets every sample accepted under the old
  authority.
- **The direct abort.** The NAVIGATIONAL or OVERALL assignee can abort automation directly. The gate
  verifies them on its own projection of `command_assignment` and the session records: the assignee,
  the current revision, and that assignment's session token.
  - **With the manager dead, the last session heard counts.** Sessions are renewed through the
    manager, so they lapse with it. The abort only ever makes the vessel more inhibited.
  - **Not a back door:** it fences the automation, proposes no map and grants nothing.
- **A guidance candidate is checked twice, independently (I15).** The navigator's
  `GuidanceCandidate` must pass both:
  - **DECISION:** under NAV_FUNCTION, the source holds the named function in the navigation
    authority document, at method ≥ DELEGATED, under the document's current revision. Under
    HUMAN_ORDER, the `order_id` is the human decision in force in the source's latest
    `navigation_state`, heard within `navigation_state_max_age_ms` (default 3000) on the gate's
    clock (keelson `NavigationState.order_id`, crowsnest-dev#94). The source's records are the
    producers it spells, `navigation/0` or `navigation/0/…`. If none is fresh, if more than one
    is, or if the record reports no order, the candidate is refused.
  - **EXECUTION:** under DIRECT, the source holds the conn itself; under CARRY_OUT, the named
    mapping is the DOF's and executes this source.

  Then the per-DOF test runs. A navigator that holds `track_keeping` but not `collision_avoidance`
  cannot smuggle an avoidance through: EXECUTION passes and DECISION refuses.
- **`resolved_control`, every cycle.** For each DOF it reports:
  - the requested and accepted values. The accepted value is **absent when the gate emits
    nothing**, never 0;
  - the control state and its reason;
  - feedback marked NONE, since no connector reports what it applied yet.

  The message also carries the motion fence, a message sequence and a sequence per coupled group.
- **A lost store is a new term.** If the store is missing, corrupt or fails its hash, the gate
  reinitialises with no map and enters the transition state. Every command from before is then from
  a term that no longer exists. That is the safe direction for an enforcer, unlike the manager,
  which refuses to start.

## Keys

| Serves / hears | |
| --- | --- |
| serves `control_gate/v1` as responder `control-gate` | `prepare`, `commit`, `get_gate_state`, `abort_automation` |
| hears `command_authority/{vessel}` and `…/{vessel}/{overall,navigational}/session` | the holder's lease and the sessions, stamped on local receipt |
| hears `command_assignment/{vessel}` | its own projection, for the direct abort |
| hears `safety_state/{vessel}` | ACTUATION_ALLOWED |
| hears `control_candidate/**` and `guidance_candidate/**` | the helm's and the navigator's proposals, per DOF |
| hears `command_authority/{vessel}/navigation` | the navigation authority document, for DECISION |
| hears `navigation_state/**` | each producer's human order in force, for HUMAN_ORDER, stamped on local receipt |
| publishes `resolved_control/{vessel}` | every cycle: the rolling acknowledgement |

## Developing before the build

The pin is the experimental build that carries #331, #332, #358 and #359. Until it exists, develop
against a **local merge** of those branches (never pushed), **without committing that**:

```bash
# in the keelson checkout: one scratch worktree with the four branches merged, SDK regenerated
uv venv --python 3.13 .venv
uv pip install --python .venv/bin/python -e ../keelson.worktrees/gate-sdk/sdks/python pytest==8.3.4 ruff==0.14.11
uv pip install --python .venv/bin/python --no-deps -e ../keelson-processor-control-authority  # the manager, for the joint tests
.venv/bin/python -m pytest -q
```

The joint tests (`tests/test_with_manager.py`, and the two-process e2e) are skipped when the manager
is not installed. `tests/test_with_navigator.py` runs the navigator's own guidance candidates
through `on_guidance`, under NAV_FUNCTION, HUMAN_ORDER and DIRECT. It is skipped unless
keelson-processor-navigation is installed the same way (`uv pip install --no-deps -e
../keelson-processor-navigation`).

**HUMAN_ORDER needs `NavigationState.order_id`,** from keelson's `feat/navigation-state-order-id`,
stacked on #376. `0.6.0.dev21` does not carry it. On that build every `navigation_state` reads as
"no human order in force", so HUMAN_ORDER is still refused: the check fails closed, as before. To
develop against it, install a local SDK carrying both that branch and the pin's PRs. The joint e2e
also needs the manager's maintenance PR (#377).

## Not in this cut (#85's later slices)

- **HUMAN_ORDER verification**, which needs an order id on `navigation_state`.
- **Comparing a function envelope (knots) with a DOF's percent.** The DOF's execution envelope still
  bounds the value.
- **Each navigation function's `holder_lease_id`**, and its `comms_loss_behaviour` when that lapses.
- **Constraint clamping** (`propulsion_constraint`, `motion_constraint`), keeping a latched constraint
  when its writer dies (I5), and restore-and-rebase on a new term (I16).
- **The rest of the §3.2 watchdogs:** `CONTROL_LINK_HEALTH`, `CONTROLLER_SUPERVISION_CHANNEL_HEALTH`,
  payload-gate liveliness, supervisor liveliness in `operational`, and
  `on_safety_supervisor_loss_while_active`.
- **`maintenance_state`** (no subject yet) and **the lifecycle** in ACTUATION_ALLOWED. Arm-path
  refusals for the legacy interfaces.
- **Accepting stored control-plane state** only under the current fence and a newer `revision`.
