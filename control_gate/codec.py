"""control_gate/v1 and the subjects the gate hears, as the model's records. The only module that
imports keelson's types, so model.py stays pure."""

from __future__ import annotations

from keelson.interfaces.ControlGate_pb2 import AbortAutomationResponse, CommitResponse, GateState
from keelson.interfaces.VehicleCommon_pb2 import CommandResult
from keelson.payloads.ControlMapping_pb2 import ControlMapping, TransferAck, TransferKind
from keelson.payloads.Fence_pb2 import Fence as PbFence

from . import model
from .model import DirectAbort, Fence, Mapping, Proposal

KIND = {
    TransferKind.TRANSFER_KIND_GRANT: model.GRANT,
    TransferKind.TRANSFER_KIND_TAKE_CONTROL: model.TAKE_CONTROL,
    TransferKind.TRANSFER_KIND_STATION_HANDOVER: model.STATION_HANDOVER,
    TransferKind.TRANSFER_KIND_ABORT_AUTOMATION: model.ABORT_AUTOMATION,
}


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
    return out


def direct_abort_from_proto(pb) -> DirectAbort:
    return DirectAbort(pb.requester_id, pb.assignment_id, int(pb.assignment_revision), pb.session_token, pb.reason)


def abort_response(ok: bool, detail: str, fence: Fence) -> AbortAutomationResponse:
    return AbortAutomationResponse(result=CommandResult.COMMAND_RESULT_ACCEPTED if ok else CommandResult.COMMAND_RESULT_DENIED,
                                   detail=detail, fence=fence_to_proto(fence))


def assignment_seen(pb):
    return model.AssignmentSeen(pb.assignee, pb.assignment_id, int(pb.revision)) if pb.assignee else None


# ---------------------------------------------------------------------------- the data plane (#272)


def sample_from_control_candidate(pb, recv_ms: int) -> model.Sample:
    fence = fence_from_proto(pb.fence) if pb.HasField("fence") else None
    return model.Sample(pb.source, pb.dof, float(pb.value_pct), pb.stream.stream_id, int(pb.stream.sequence), recv_ms, fence)


def _function_name(value: int) -> str:
    from keelson.payloads.NavigationAuthority_pb2 import NavigationFunction

    return NavigationFunction.Name(value).removeprefix("NAVIGATION_FUNCTION_").lower()


def guidance_from_proto(pb, recv_ms: int) -> model.Guidance:
    decision = pb.WhichOneof("decision_basis") or ""
    execution = pb.WhichOneof("execution_basis") or ""
    g = dict(source=pb.source, dof=pb.dof, value_pct=float(pb.value_pct), stream_id=pb.stream.stream_id,
             sequence=int(pb.stream.sequence), recv_ms=recv_ms, decision=decision, execution=execution)
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
    for r in cycle.rows:
        d = ResolvedDof(dof=r.dof, control_state=_STATE.get(r.control_state, 0), source=r.source, reason=r.reason,
                        feedback_quality=FeedbackQuality.FEEDBACK_QUALITY_NONE)
        if r.requested is not None:
            d.requested_pct = r.requested
        if r.value is not None:
            d.accepted_pct = r.value
        out.dofs.append(d)
    return out

