"""control_gate/v1 and the subjects the gate hears, as the model's records. The only module that
imports keelson's types, so model.py stays pure."""

from __future__ import annotations

from keelson.interfaces.ControlGate_pb2 import AbortAutomationResponse, CommitResponse, GateState
from keelson.interfaces.VehicleCommon_pb2 import CommandResult
from keelson.payloads.ControlMapping_pb2 import ControlMapping, TransferAck, TransferKind
from keelson.payloads.Fence_pb2 import Fence as PbFence
from keelson.payloads.PropulsionConstraint_pb2 import PropulsionConstraint

from . import model
from .model import DirectAbort, Fence, Mapping, Proposal

KIND = {
    TransferKind.TRANSFER_KIND_GRANT: model.GRANT,
    TransferKind.TRANSFER_KIND_TAKE_CONTROL: model.TAKE_CONTROL,
    TransferKind.TRANSFER_KIND_STATION_HANDOVER: model.STATION_HANDOVER,
    TransferKind.TRANSFER_KIND_ABORT_AUTOMATION: model.ABORT_AUTOMATION,
}


def constraint_from_proto(pb) -> model.Constraint | None:
    """A `PropulsionConstraint`, or None when it carries no fence (never applied)."""
    fence = fence_from_proto(pb.fence) if pb.HasField("fence") else None
    if fence is None:
        return None
    devices = tuple(model.DeviceLimit(d.device_id, d.max_power_pct if d.HasField("max_power_pct") else None, bool(d.inhibit),
                                      d.reason) for d in pb.devices)
    return model.Constraint(fence, int(pb.revision), pb.controller_id, pb.controller_site, devices)


def constraint_to_proto(c: model.Constraint, vessel_id: str, ts) -> PropulsionConstraint:
    out = PropulsionConstraint(vessel_id=vessel_id, controller_id=c.controller_id, controller_site=c.controller_site,
                               fence=fence_to_proto(c.fence), revision=c.revision)
    out.timestamp.CopyFrom(ts)
    for d in c.devices:
        dc = out.devices.add(device_id=d.device_id, inhibit=d.inhibit, reason=d.reason)
        if d.max_power_pct is not None:
            dc.max_power_pct = d.max_power_pct
    return out


# keelson GateState.last_fence_cause, in the builds that carry it (crowsnest-dev#290).
HAS_FENCE_CAUSE = "last_fence_cause" in GateState.DESCRIPTOR.fields_by_name
_CAUSE_KIND = {model.CAUSE_ABORT: 1, model.CAUSE_CONN_LAPSE: 2, model.CAUSE_INVALID_SOURCE: 3, model.CAUSE_LOCAL_FALLBACK: 4}


def fence_to_proto(f: Fence) -> PbFence:
    return PbFence(term=f.term, generation=f.generation)


def fence_from_proto(p) -> Fence | None:
    return None if p is None or not p.term else Fence(p.term, int(p.generation))


def mapping_from_proto(pb: ControlMapping) -> Mapping:
    env = ()
    if pb.HasField("execution_envelope"):
        e = pb.execution_envelope
        env = (e.min_pct, e.max_pct, e.max_rate_pct_per_s)
    fence = fence_from_proto(pb.fence) if pb.HasField("fence") else None
    return Mapping(pb.dof, pb.authority_holder, pb.authority_lease_id, pb.execution_source, env, pb.group_id, pb.mapping_id,
                   fence)


def mapping_to_proto(m: Mapping) -> ControlMapping:
    pb = ControlMapping(mapping_id=m.mapping_id, dof=m.dof, authority_holder=m.authority_holder,
                        authority_lease_id=m.authority_lease_id, execution_source=m.execution_source, group_id=m.group_id)
    if m.envelope:
        e = pb.execution_envelope
        e.min_pct, e.max_pct, e.max_rate_pct_per_s = m.envelope
    if m.fence is not None:
        pb.fence.CopyFrom(fence_to_proto(m.fence))
    return pb


def proposal_from_proto(pb) -> Proposal:
    """A TransferPrepare. A row's fence is kept so that prepare() can refuse it (I2)."""
    return Proposal(pb.transfer_id, KIND.get(pb.kind, "unspecified"), tuple(mapping_from_proto(m) for m in pb.mappings),
                    pb.active_control_station, int(pb.prepare_ttl_ms), pb.requested_by)


def ack_to_proto(transfer_id: str, ack: model.Ack) -> TransferAck:
    out = TransferAck(transfer_id=transfer_id, accepted=ack.accepted, reason=ack.reason,
                      rejected_mapping_ids=list(ack.rejected_mapping_ids))
    if ack.fence is not None:
        out.proposed_fence.CopyFrom(fence_to_proto(ack.fence))
    return out


def commit_response(transfer_id: str, applied: bool, reason: str, fence: Fence) -> CommitResponse:
    return CommitResponse(transfer_id=transfer_id, applied=applied, reason=reason, fence=fence_to_proto(fence))


def gate_state(st: model.State) -> GateState:
    out = GateState(vessel_id=st.vessel_id, fence=fence_to_proto(st.fence), active_control_station=st.station,
                    mappings=[mapping_to_proto(m) for m in st.mappings], in_transition=st.in_transition)
    if st.reservation is not None:
        out.reserved_transfer_id = st.reservation.proposal.transfer_id
    c = st.last_cause
    if c is not None and HAS_FENCE_CAUSE:
        # keelson GateState.last_fence_cause (crowsnest-dev#290); a build without it says nothing.
        pc = out.last_fence_cause
        pc.kind = _CAUSE_KIND.get(c.kind, 0)
        pc.fence.CopyFrom(fence_to_proto(c.fence))
        pc.requested_by, pc.assignment_id, pc.assignment_revision, pc.reason = (
            c.requested_by, c.assignment_id, c.assignment_revision, c.reason)
    return out


def direct_abort_from_proto(pb) -> DirectAbort:
    # keelson AbortAutomationRequest.attestation (crowsnest-dev#316); a build without it carries none.
    att = pb.attestation if "attestation" in pb.DESCRIPTOR.fields_by_name and pb.HasField("attestation") else None
    return DirectAbort(pb.requester_id, pb.assignment_id, int(pb.assignment_revision), pb.session_token, pb.reason,
                       attestation=att, request=pb)


def abort_response(ok: bool, detail: str, fence: Fence) -> AbortAutomationResponse:
    return AbortAutomationResponse(result=CommandResult.COMMAND_RESULT_ACCEPTED if ok else CommandResult.COMMAND_RESULT_DENIED,
                                   detail=detail, fence=fence_to_proto(fence))


def assignment_seen(pb):
    return model.AssignmentSeen(pb.assignee, pb.assignment_id, int(pb.revision)) if pb.assignee else None


# ---------------------------------------------------------------------------- the data plane (#272)


def _provenance(pb, recv_wall_ns, published_ns) -> dict:
    """What a candidate says about its own times, plus the gate's wall receipt (crowsnest-dev#98)."""
    return dict(sample_time_ns=pb.timestamp.ToNanoseconds() if pb.HasField("timestamp") else None,
                published_ns=published_ns, clock_quality=int(pb.stream.clock_quality), recv_wall_ns=recv_wall_ns)


def sample_from_control_candidate(pb, recv_ms: int, recv_wall_ns: int | None = None,
                                  published_ns: int | None = None) -> model.Sample:
    fence = fence_from_proto(pb.fence) if pb.HasField("fence") else None
    # The helm's own health and calibration (keelson#382); a build without the fields says nothing.
    own = dict(profile_version=getattr(pb, "profile_version", ""), device_health=int(getattr(pb, "device_health", 0)))
    return model.Sample(pb.source, pb.dof, float(pb.value_pct), pb.stream.stream_id, int(pb.stream.sequence), recv_ms, fence,
                        **_provenance(pb, recv_wall_ns, published_ns), **own)


def _function_name(value: int) -> str:
    from keelson.payloads.NavigationAuthority_pb2 import NavigationFunction

    return NavigationFunction.Name(value).removeprefix("NAVIGATION_FUNCTION_").lower()


def guidance_from_proto(pb, recv_ms: int, recv_wall_ns: int | None = None,
                        published_ns: int | None = None) -> model.Guidance:
    decision = pb.WhichOneof("decision_basis") or ""
    execution = pb.WhichOneof("execution_basis") or ""
    g = dict(source=pb.source, dof=pb.dof, value_pct=float(pb.value_pct), stream_id=pb.stream.stream_id,
             sequence=int(pb.stream.sequence), recv_ms=recv_ms, decision=decision, execution=execution,
             **_provenance(pb, recv_wall_ns, published_ns))
    if decision == "nav_function":
        g.update(function=_function_name(pb.nav_function.function), nav_revision=int(pb.nav_function.navigation_authority_revision))
    elif decision == "human_order":
        g["order_id"] = pb.human_order.order_id
    if execution == "carry_out":
        g["mapping_id"] = pb.carry_out.control_mapping_id
    return model.Guidance(**g)


def navigation_from_record(rec) -> tuple[int, dict]:
    """The navigation authority document on `command_authority/{vessel}/navigation`."""
    doc = rec.navigation_authority
    return int(doc.revision), {_function_name(f.function): (f.holder, int(f.control_method)) for f in doc.functions}


_STATE = {"ACTIVE": 1, "STALE": 2, "TRANSITION": 3, "INHIBITED": 4, "UNMAPPED": 5}


def resolved_control(cycle, vessel_id: str, gate_id: str, now=None):
    """The cycle as keelson.ResolvedControl. Absence is absence: `accepted_pct` is absent when the gate
    emits nothing, and feedback is NONE because no connector reports what it applied yet."""
    from keelson.payloads.ResolvedControl_pb2 import FeedbackQuality, GroupSequence, ResolvedControl, ResolvedDof

    out = ResolvedControl(vessel_id=vessel_id, gate_id=gate_id, sequence=cycle.sequence, motion_epoch=fence_to_proto(cycle.fence),
                          actuation_allowed=cycle.actuation_allowed, actuation_reason=cycle.actuation_reason,
                          in_transition=cycle.in_transition, transition_reason=cycle.transition_reason,
                          groups=[GroupSequence(group_id=g, sequence=n) for g, n in cycle.groups])
    if now is not None:
        out.timestamp.CopyFrom(now)
    if cycle.attestation_challenge and "attestation_challenge" in out.DESCRIPTOR.fields_by_name:
        out.attestation_challenge = cycle.attestation_challenge  # #316; a build without it says nothing
    for r in cycle.rows:
        d = ResolvedDof(dof=r.dof, control_state=_STATE.get(r.control_state, 0), source=r.source, reason=r.reason,
                        feedback_quality=FeedbackQuality.FEEDBACK_QUALITY_NONE)
        if r.requested is not None:
            d.requested_pct = r.requested
        if r.value is not None:
            d.accepted_pct = r.value
        _put_provenance(d, r.provenance, cycle.now_ms)
        out.dofs.append(d)
    return out


def _put_provenance(d, p, cycle_now_ms) -> None:
    """The accepted sample's stream, sequence and times on its ResolvedDof, copied and never judged.
    gate_hold_ms is receipt -> this cycle on the gate's monotonic clock: exact. Absent when no
    sample is accepted (only an ACTIVE row carries one)."""
    if p is None:
        return
    d.source_stream_id, d.source_sequence, d.source_clock_quality = p.stream_id, p.sequence, p.clock_quality
    for name, ns in (("source_sample_time", p.sample_time_ns), ("source_published_at", p.published_ns),
                     ("gate_received_at", p.recv_wall_ns)):
        if ns is not None:
            getattr(d, name).FromNanoseconds(ns)
    if cycle_now_ms is not None:
        d.gate_hold_ms = max(0, cycle_now_ms - p.recv_ms)
    if "source_device_health" in d.DESCRIPTOR.fields_by_name:
        # The helm's own word about itself (crowsnest-dev#309): copied, never judged. A guidance
        # candidate says neither, and a build without the fields carries neither.
        d.source_profile_version = getattr(p, "profile_version", "")
        d.source_device_health = getattr(p, "device_health", 0)
