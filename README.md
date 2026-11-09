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
> - Its own actuator still only logs. A downstream connector consuming `resolved_control` is proposed
>   in keelson#429 (`mavlink2keelson --resolved-control-gate`, opt-in) and not merged.
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
- **A start waits for the on-board warning (crowsnest-dev#377, keelson #428).** The first mapping onto a vessel with
  none is refused at PREPARE unless `start_warning` is SATISFIED or NOT_FITTED. An absent record reads as NOT_FITTED;
  anything else, or a `safety_state` unheard or stale, holds the start. It never interrupts a vessel already under
  way: a handover and every running cycle ignore it. Legacy `arm` does not come through the gate.
- **ACTUATION_ALLOWED ⇒ maintenance_state INACTIVE (crowsnest-dev#302).** Any other state of the
  procedure inhibits every DOF, with a reason that names it ("maintenance_state is PREPARING: …"). So do
  a record older than `maintenance_ttl_ms` (15 s: three of the manager's heartbeats) and an unheard one:
  a silent manager is not an INACTIVE one. The record is stamped on local receipt, never read from storage
  at boot, and not persisted. It binds whatever `require_safety_state` says. `require_maintenance_state:
  false` is only for a rig where nothing publishes it. On an SDK without `MaintenanceState` (the pin,
  `0.6.0.dev21`: keelson#377 is not in it), a gate that requires it refuses to start rather than inhibit
  forever.
- **The direct abort.** The NAVIGATIONAL or OVERALL assignee can abort automation directly. The gate
  verifies them on its own projection of `command_assignment` and the session records: the assignee,
  the current revision, and that assignment's session token.
  - **With the manager dead, the last session heard counts.** Sessions are renewed through the
    manager, so they lapse with it. The abort only ever makes the vessel more inhibited.
  - **Not a back door:** it fences the automation, proposes no map and grants nothing.
  - **It says why, for the manager that missed it** (crowsnest-dev#290). Every fence the gate moves
    alone records its cause with the fence it produced: an abort with the requester and the
    assignment they acted under, a CONN lapse with nobody. The cause is persisted, cleared by the next
    COMMIT, and exposed as `GateState.last_fence_cause` in the keelson builds that carry it. A manager
    that was down reads it at reconcile and takes the navigator's functions back to that human.
  - **Attested, by the gate itself** (crowsnest-dev#316; trust chain B, COMMAND-ARCHITECTURE §6).
    `requester_id` is a label. The ROC authority service's Ed25519-signed claim on
    `AbortAutomationRequest.attestation` (the manager's own `Attestation` type) is what names who
    is asking. The gate checks it in `control_gate/attestation.py`, a port of the manager's
    verifier, against the policy's trusted keys, before the projection check:
    - **Its challenge is the gate's own**, because this call is for when the manager is down. A new
      one is drawn every `rotate_ms`, lives `challenge_ttl_ms` on the gate's monotonic clock, is
      published as `resolved_control.attestation_challenge`, and is forgotten on restart.
    - **The claim names `control_gate/v1/abort_automation`**, this request's digest (with
      `attestation` cleared), and `requester_id`/`requester_site` as its operator and site. Each
      `(key, session, sequence)` is accepted once, rising. A refused call burns no sequence.
    - **Missing** is refused only where the policy requires it (`operational` by default), with
      "requires an attestation … none was presented". **Present and failing** is always refused,
      with the check that failed. Every refusal is a WARNING; the fence does not move.
    - **Never silently closed.** A policy that requires attestation with no trusted key refuses to
      load, and the bin refuses to start on an SDK that cannot carry one. Either way the error says
      why.
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
  - on an ACTIVE row, the **provenance** of the accepted sample (crowsnest-dev#98): its stream,
    sequence and clock quality, its sample and publish times, the gate's wall-clock receipt, and
    `gate_hold_ms`, receipt to this cycle on the gate's monotonic clock. A station times the hops
    from these. They are copied, never judged: they are kept out of a sample's equality, and a test
    proves that two samples differing only there resolve identically.
  - on an ACTIVE row, the **helm's own word about itself** (crowsnest-dev#309): the accepted
    sample's `device_health` and `profile_version`, as `source_device_health` and
    `source_profile_version`. A station can then tell why a helm's samples are late (its own read
    watchdog says DEGRADED), and a recording can say which calibration shaped the value. The gate
    logs each helm entering and leaving DEGRADED, once per transition. It **never refuses a
    DEGRADED helm for being DEGRADED**: validity is receive-side freshness, and a test proves that a
    DEGRADED sample resolves exactly as a NOMINAL one would.

  The message also carries the motion fence, a message sequence and a sequence per coupled group.
- **A lost store is a new term.** If the store is missing, corrupt or fails its hash, the gate
  reinitialises with no map and enters the transition state. Every command from before is then from
  a term that no longer exists. That is the safe direction for an enforcer, unlike the manager,
  which refuses to start.
- **It mints `plant_generation` and clamps by the engineer's constraint** (§1.1, I16,
  crowsnest-dev#270).
  - **What mints a generation:** the authority manager authorises a change of ENGINEER holder by
    publishing the new holder's lease. A token the gate has not seen is that change, and the gate
    mints the next generation under its own term.
  - **Inheritance:** in the same step, persisted before anything else, it rebases the constraint
    in force onto the new generation. The devices are the same, at revision 1, and the gate
    publishes the result on `propulsion_constraint/{vessel}`. A change of engineer can therefore
    never relax a constraint.
  - **Applying a record:** a record is applied only if it is under the current plant fence, at a
    newer revision, and written by the live ENGINEER holder. The outgoing engineer's late record,
    under the old generation, is dropped.
  - **Clamping:** every resolved value of a DOF that the policy's `propulsion` maps to a device is
    clamped, whether ACTIVE, STALE or in TRANSITION. An inhibited device gives 0, and a derate caps
    the magnitude, astern included. The reason is on the row.
  - **What never moves it:** a lapsed or released ENGINEER lease mints nothing and lifts nothing.
    The conn and the payload leases never move `plant_generation`, and a plant change never moves
    the motion fence.
  - **A new term restores it and waits for a person** (crowsnest-dev#345). A reinitialised store
    is born holding ACTUATION_ALLOWED false until the gate has read `propulsion_constraint/{vessel}`
    from storage at boot.
    - **No record:** no constraint, and the hold is lifted.
    - **A record:** it is validated, rebased under the new term at revision 1 (the live ENGINEER as
      its writer, if one is heard), persisted, published, and clamped by from the first cycle. The
      hold stays until the live ENGINEER writes a revision under the new plant fence. Storage keeps
      the last record anybody put, which can be one the old gate refused, so a restored record is
      never trusted to be the one that was in force. Several replies restore the tightest of them.
    - **An invalid record, or a failed read:** nothing is restored, and the hold stays until the
      ENGINEER's own record under the new term.
    - **The hold is persisted,** so a gate restarted before the confirmation keeps waiting. The
      reason is `actuation_reason` on `resolved_control`.

- **The navigation authority document applies only at a newer revision** (I11, crowsnest-dev#85).
  keelson's `NavigationAuthority` has no motion fence, so its revision is judged within its
  incarnation (`assignment_id`): revisions 5, 7, 6 apply 5 then 7 and drop 6. A new `assignment_id`
  is the document re-issued, with its own revisions. The floors are persisted, so a restart does not
  take an older document back from storage.
- **`motion_constraint`, an interim** (crowsnest-dev#85; keelson#421, keelson#422). One slot per
  payload, `motion_constraint/{vessel}/{payload_id}`, subscribed and read from storage at boot.
  - **I11 under one fence:** a revision applies only if it is newer than the slot's.
  - **The latch outlives its writer (I5):** nothing expires a slot. A latch lifts only by a newer
    revision carrying a `release` that matches `clears_on`: sensor for sensor, the same procedure id.
  - **A fence this gate cannot judge:** keelson does not yet say how the motion gate learns a payload's
    current `payload_epoch` (keelson#421), and terms cannot be ordered. A record under another fence is
    applied only if it relaxes nothing: every limit as tight or tighter, the band no wider, a latch kept.
  - **No clamp yet:** nothing converts knots, deg/s or a heading band into a DOF's percent
    (keelson#422). So a **latched** slot with a limit refuses ACTUATION_ALLOWED, and the gate enters its
    transition state. This is deliberately severe: no motion while a payload's latch is live. A
    non-latched slot is logged as reported and not enforced.
  - **I16:** a new term holds until storage is read. Valid slots are restored as they were (their fence
    is the payload gate's). An invalid or unreadable one holds actuation until a valid record for that
    payload is accepted. The slots and the hold are persisted.

## Keys

| Serves / hears | |
| --- | --- |
| serves `control_gate/v1` as responder `control-gate` | `prepare`, `commit`, `get_gate_state`, `abort_automation` |
| hears `command_authority/{vessel}` and `…/{vessel}/{overall,navigational}/session` | the holder's lease and the sessions, stamped on local receipt |
| hears `command_assignment/{vessel}` | its own projection, for the direct abort |
| hears `safety_state/{vessel}` | ACTUATION_ALLOWED |
| hears `maintenance_state/{vessel}` | ACTUATION_ALLOWED: INACTIVE and fresh, or every DOF INHIBITED |
| hears `control_candidate/**` and `guidance_candidate/**` | the helm's and the navigator's proposals, per DOF |
| hears `command_authority/{vessel}/navigation` | the navigation authority document, for DECISION |
| hears `navigation_state/**` | each producer's human order in force, for HUMAN_ORDER, stamped on local receipt |
| hears `command_authority/{vessel}/engineer` | a new ENGINEER token mints `plant_generation` |
| hears `propulsion_constraint/{vessel}` (and reads storage at boot) | the plant constraint, under the plant fence |
| publishes `propulsion_constraint/{vessel}` | only on a new ENGINEER holder: the inherited constraint at revision 1 |
| hears `motion_constraint/{vessel}/*` (and reads storage at boot) | one slot per payload; the interim (above): a latched one refuses actuation |
| publishes `resolved_control/{vessel}` | every cycle: the rolling acknowledgement, and the direct abort's attestation challenge |

The policy's `attestation` block, the same shape as the manager's:

```json
"attestation": {
  "keys": {"roc1-a": {"service_id": "roc1-authority", "public_key": "<base64, raw 32-byte Ed25519>"}},
  "required": true,
  "challenge_ttl_ms": 15000,
  "rotate_ms": 5000
}
```

`required` defaults to `command_profile == "operational"`. The keys are maintenance configuration and
never come over the link.

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

**A helm's health and calibration need `ControlCandidate.device_health` / `profile_version`
(keelson#382) and `ResolvedDof.source_device_health` / `source_profile_version`** (keelson
`feat/resolved-dof-device-health`, stacked on #386). `0.6.0.dev21` carries neither: the codec reads
UNKNOWN and an empty profile, writes nothing, and `tests/test_helm_health.py` skips its two codec
cases.

**The attested direct abort needs `AbortAutomationRequest.attestation` and
`ResolvedControl.attestation_challenge`** (keelson `feat/gate-abort-attestation`, stacked on #383).
`0.6.0.dev21` carries neither. On it, the codec reads every abort as unattested and publishes no
challenge, so under a policy that requires attestation the bin refuses to start.
`tests/test_abort_attestation.py` skips everything that builds an attested request, naming the
fields. To develop against it, regenerate that branch's `ControlGate`, `ResolvedControl` and
`ControlCandidate` modules (merged with `feat/gate-fence-cause` and
`feat/resolved-dof-device-health`) over a copy of the pinned SDK, and put it first on `PYTHONPATH`.

## Not in this cut (#85's later slices)

- **The motion_constraint clamp**, and comparing a function envelope (knots) with a DOF's percent:
  keelson#422. Until then a latched motion_constraint refuses actuation (above).
- **Judging a payload's current `payload_epoch`:** keelson#421. Until then nothing is relaxed under a
  fence the gate cannot judge.
- **Each navigation function's `holder_lease_id`**, and its `comms_loss_behaviour` when that lapses: a
  process holder's liveliness key is keelson#424; the timeout is on keelson#363.
- **The rest of the §3.2 watchdogs:**
  - `CONTROL_LINK_HEALTH` (what to watch: keelson#423);
  - `CONTROLLER_SUPERVISION_CHANNEL_HEALTH` (keelson#380);
  - payload-gate liveliness;
  - supervisor liveliness in `operational`;
  - `on_safety_supervisor_loss_while_active` (policy on keelson#363).
- **The lifecycle** in ACTUATION_ALLOWED (keelson#425). Arm-path refusals for the legacy interfaces are
  mavlink2keelson's (keelson `feat/maintenance-gated-writes`).
