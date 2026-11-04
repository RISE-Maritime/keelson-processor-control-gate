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
