"""The direct abort is attested (crowsnest-dev#316): control_gate/v1 abort_automation carries the ROC
service's signed claim, verified by the gate against a challenge the GATE published on resolved_control,
because the manager may be the thing that is down. No network, apart from the one e2e at the end
(isolated loopback, like test_e2e.py).

Everything that builds an attested request needs keelson's AbortAutomationRequest.attestation and
ResolvedControl.attestation_challenge, which the pin does not carry yet: those tests skip, naming it,
and run against a local build (README, "Developing before the build")."""

import base64
import hashlib
import json
import time

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey
from keelson.interfaces import ControlGate_pb2
from keelson.interfaces.AuthorityManager_pb2 import AttestationClaim
from keelson.interfaces.ControlGate_pb2 import AbortAutomationRequest
from keelson.payloads.ResolvedControl_pb2 import ResolvedControl

from control_gate import attestation, codec, model
from control_gate.actuation import LoggingActuator
from control_gate.model import DirectAbort
from control_gate.policy import parse
from control_gate.service import Gate
from control_gate.store import Store
from test_gate import DOFS, V, Clock, clear, lease, take, Helm

HAS_FIELDS = ("attestation" in AbortAutomationRequest.DESCRIPTOR.fields_by_name
              and "attestation_challenge" in ResolvedControl.DESCRIPTOR.fields_by_name)
needs_sdk = pytest.mark.skipif(not HAS_FIELDS, reason="keelson without AbortAutomationRequest.attestation / "
                               "ResolvedControl.attestation_challenge (feat/gate-abort-attestation, crowsnest-dev#316)")

# THE SHARED VECTOR. The same seed, key id and claim are in the authority manager's
# tests/test_attestation.py: the two verifiers are copies, and a change to the claim's encoding or to
# the key format that one of them does not follow fails there or here.
SEED = bytes(range(32))
KEY = Ed25519PrivateKey.from_private_bytes(SEED)
PUBLIC_B64 = base64.b64encode(KEY.public_key().public_bytes_raw()).decode()
VECTOR_PUBLIC_B64 = "A6EHv/POEL4dcN0Y50vAmWfk1jCbpQ1fHdyGZBJVMbg="
VECTOR_CLAIM = AttestationClaim(service_id="roc1-authority", key_id="roc1-a", operator_id="anna@ROC-2", site="ROC-2",
                                station="ROC-2/console-1", session_id="sess-9", sequence=1, challenge=b"\x07" * 16,
                                procedure="control_gate/v1/abort_automation", request_sha256=b"\x00" * 32)
VECTOR_CLAIM_SHA256 = "b0476d3bc1f2e749b1064505d7309f833a266dbb126dda71ccb7e7f525572db7"
VECTOR_SIGNATURE_SHA256 = "eec00c942b4bff59c6224ac637aa67f7db281cca162874b55f84dd8849a864c6"

KEYS = {"roc1-a": {"service_id": "roc1-authority", "public_key": PUBLIC_B64}}
BASE = {"dofs": DOFS, "automation_sources": ["navigation/0"], "transition": {"values": {"throttle": 0}}}


@pytest.fixture
def clock():
    return Clock()


def policy(**att):
    return parse({**BASE, "attestation": {"keys": KEYS, **att}})


def make(tmp_path, clock, pol):
    g = Gate(store=Store(tmp_path / "gate.sqlite"), policy=pol, vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    g.boot()
    g.restore_constraints([], [])  # storage holds none (#345)
    g.restore_motion([], [])  # nor any motion_constraint (#85)
    clear(g)
    lease(g)
    Helm(g, clock).all(0)
    ack, applied = take(g, clock)
    assert ack.accepted and applied
    g.observe_assignments(None, model.AssignmentSeen("anna@ROC-2", "as-7", 4), "authority-manager/0", 15000)
    g.observe_session(model.NAVIGATIONAL, "anna@ROC-2", "sess-a", "as-7", 30000, False)
    return g


def request(reason="the automation is heading for the breakwater", **kw):
    return AbortAutomationRequest(**{"requester_id": "anna@ROC-2", "requester_site": "ROC-2", "assignment_id": "as-7",
                                     "assignment_revision": 4, "session_token": "sess-a", "reason": reason, **kw})


class Roc:
    """The ROC authority service: reads the gate's challenge off resolved_control, signs the call."""

    def __init__(self, key=KEY, key_id="roc1-a", service_id="roc1-authority", session_id="sess-9"):
        self.key, self.key_id, self.service_id, self.session_id = key, key_id, service_id, session_id
        self.sequence = 0

    def sign(self, req, challenge, procedure=attestation.PROCEDURE, sequence=None, operator=("anna@ROC-2", "ROC-2"),
             digest_of=None):
        self.sequence = sequence if sequence is not None else self.sequence + 1
        claim = AttestationClaim(service_id=self.service_id, key_id=self.key_id, operator_id=operator[0], site=operator[1],
                                 station="ROC-2/console-1", session_id=self.session_id, sequence=self.sequence,
                                 challenge=challenge, procedure=procedure,
                                 request_sha256=attestation.request_digest(digest_of or req)).SerializeToString()
        req.attestation.claim = claim
        req.attestation.signature = self.key.sign(claim)
        return req


def challenge_of(g, clock):
    """What a station reads: the challenge on this cycle's resolved_control."""
    g.cycle()
    rc = ResolvedControl.FromString(codec.resolved_control(g.last_cycle, V, "control-gate").SerializeToString())
    return rc.attestation_challenge


def call(g, req):
    return g.abort_direct(codec.direct_abort_from_proto(req))


# ---------------------------------------------------------------------------- policy, and the pin


def test_operational_requires_attestation_and_a_gate_with_no_key_refuses_to_load():
    assert parse({**BASE, "command_profile": "operational", "attestation": {"keys": KEYS}}).attestation_required
    assert not parse({**BASE}).attestation_required  # test: optional
    with pytest.raises(ValueError, match="no trusted key: the direct abort would refuse everyone"):
        parse({**BASE, "command_profile": "operational"})
    with pytest.raises(ValueError, match="no trusted key"):
        parse({**BASE, "attestation": {"required": True}})
    with pytest.raises(ValueError, match="rotate_ms"):
        parse({**BASE, "attestation": {"keys": KEYS, "rotate_ms": 15000, "challenge_ttl_ms": 15000}})


def test_under_operational_a_missing_attestation_is_refused_and_says_why(tmp_path, clock):
    g = make(tmp_path, clock, policy(required=True))
    n = g.state.generation
    ok, detail, fence = g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a"))
    assert not ok and fence.generation == n
    assert detail == attestation.MISSING and "requires an attestation" in detail and "none was presented" in detail


def test_under_test_a_missing_attestation_is_still_the_projection_check_alone(tmp_path, clock):
    g = make(tmp_path, clock, policy())
    n = g.state.generation
    assert not g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 3, "sess-a"))[0]  # stale: refused as before
    ok, _, fence = g.abort_direct(DirectAbort("anna@ROC-2", "as-7", 4, "sess-a"))
    assert ok and fence.generation == n + 1


def test_the_shared_vector():
    """The manager's copy pins the same: if either verifier's key or claim format drifts, one fails."""
    assert PUBLIC_B64 == VECTOR_PUBLIC_B64
    keys = attestation.parse_keys(KEYS)
    assert keys["roc1-a"].public_key == base64.b64decode(VECTOR_PUBLIC_B64)
    claim = VECTOR_CLAIM.SerializeToString(deterministic=True)
    assert hashlib.sha256(claim).hexdigest() == VECTOR_CLAIM_SHA256  # the claim's encoding
    # Ed25519 is deterministic: the same key and bytes give the same signature in both repos.
    signature = KEY.sign(claim)
    assert hashlib.sha256(signature).hexdigest() == VECTOR_SIGNATURE_SHA256
    Ed25519PublicKey.from_public_bytes(keys["roc1-a"].public_key).verify(signature, claim)  # raises if not


# ---------------------------------------------------------------------------- an attested abort


@needs_sdk
def test_the_gate_publishes_its_own_challenge_and_rotates_it_on_its_own_clock(tmp_path, clock):
    g = make(tmp_path, clock, policy(rotate_ms=5000, challenge_ttl_ms=15000))
    first = challenge_of(g, clock)
    assert len(first) == 16
    clock.t += 4999
    assert challenge_of(g, clock) == first
    clock.t += 1
    second = challenge_of(g, clock)
    assert second != first and first in g.challenges.live(clock.t)  # the old one still counts within its TTL


@needs_sdk
def test_an_attested_direct_abort_fences_and_names_who_attested(tmp_path, clock):
    g = make(tmp_path, clock, policy(required=True))
    n = g.state.generation
    ok, detail, fence = call(g, Roc().sign(request(), challenge_of(g, clock)))
    assert ok, detail
    assert fence.generation == n + 1 and g.state.in_transition
    assert g.state.last_cause.kind == model.CAUSE_ABORT and g.state.last_cause.requested_by == "anna@ROC-2"
    assert "attested by roc1-authority for ROC-2/console-1" in g.events[-1]


@needs_sdk
@pytest.mark.parametrize("tamper, why", [
    (lambda r, c: Roc(key=Ed25519PrivateKey.generate()).sign(r, c), "signature does not verify"),
    (lambda r, c: Roc(key_id="roc9").sign(r, c), "'roc9' is not one this vessel trusts"),
    (lambda r, c: Roc(service_id="roc9-authority").sign(r, c), "belongs to roc1-authority"),
    (lambda r, c: Roc().sign(r, b"\x01" * 16), "not one this gate issued"),
    (lambda r, c: Roc().sign(r, c, procedure="abort_automation"), "not 'control_gate/v1/abort_automation'"),
    (lambda r, c: Roc().sign(r, c, digest_of=request(reason="another")), "its digest does not match"),
    (lambda r, c: Roc().sign(r, c, operator=("ted@ROC-1", "ROC-1")), "is not the attested ted@ROC-1 at ROC-1"),
])
def test_a_bad_attestation_is_refused_with_its_reason_even_where_none_is_required(tmp_path, clock, tamper, why):
    g = make(tmp_path, clock, policy())  # test profile: optional, yet present-and-bad is refused
    n = g.state.generation
    ok, detail, fence = call(g, tamper(request(), challenge_of(g, clock)))
    assert not ok and why in detail
    assert fence.generation == n and not g.state.in_transition


@needs_sdk
def test_a_claim_that_does_not_parse_is_refused(tmp_path, clock):
    g = make(tmp_path, clock, policy())
    req = request()
    req.attestation.claim, req.attestation.signature = b"\xff\xff", b"x"
    ok, detail, _ = call(g, req)
    assert not ok and "does not parse" in detail


@needs_sdk
def test_a_challenge_past_its_ttl_on_the_gates_clock_is_stale(tmp_path, clock):
    g = make(tmp_path, clock, policy(rotate_ms=5000, challenge_ttl_ms=15000))
    c = challenge_of(g, clock)
    clock.t += 15001
    ok, detail, _ = call(g, Roc().sign(request(), c))
    assert not ok and "not one this gate issued within its TTL" in detail


@needs_sdk
def test_a_replayed_sequence_is_refused_and_a_refused_call_burns_no_sequence(tmp_path, clock):
    g = make(tmp_path, clock, policy())
    roc, c = Roc(), challenge_of(g, clock)
    # Refused at the LAST checks -- after key, signature, challenge and sequence all passed -- so sequence 5
    # must still be free: a call that is refused never burns the sequence it carried.
    assert not call(g, roc.sign(request(), c, sequence=5, procedure="abort_automation"))[0]
    assert not call(g, roc.sign(request(), c, sequence=5, digest_of=request(reason="another")))[0]
    assert not call(g, roc.sign(request(), c, sequence=5, operator=("ted@ROC-1", "ROC-1")))[0]
    ok, detail, _ = call(g, roc.sign(request(), c, sequence=5))
    assert ok, detail
    again = call(g, roc.sign(request(), c, sequence=5))
    assert not again[0] and "a replay" in again[1]


@needs_sdk
def test_a_restart_forgets_every_challenge(tmp_path, clock):
    g = make(tmp_path, clock, policy())
    c = challenge_of(g, clock)
    reborn = Gate(store=Store(tmp_path / "gate.sqlite"), policy=policy(), vessel_id=V, actuator=LoggingActuator(), now_ms=clock)
    reborn.boot()
    reborn.restore_constraints([], [])  # storage holds none (#345)
    reborn.restore_motion([], [])  # nor any motion_constraint (#85)
    ok, detail, _ = reborn.abort_direct(codec.direct_abort_from_proto(Roc().sign(request(), c)))
    assert not ok and "not one this gate issued" in detail


@needs_sdk
def test_the_codec_carries_the_attestation_and_the_signed_request(tmp_path, clock):
    req = Roc().sign(request(), b"\x07" * 16)
    d = codec.direct_abort_from_proto(ControlGate_pb2.AbortAutomationRequest.FromString(req.SerializeToString()))
    assert d.attestation is not None and d.attestation.claim == req.attestation.claim
    assert attestation.request_digest(d.request) == attestation.request_digest(req)
    assert codec.direct_abort_from_proto(request()).attestation is None


# ---------------------------------------------------------------------------- end to end


@needs_sdk
@pytest.mark.e2e
def test_over_the_bus_the_station_reads_the_challenge_from_resolved_control_and_aborts_attested(tmp_path):
    from keelson.interfaces.ControlGate_pb2 import AbortAutomationResponse
    from keelson.interfaces.VehicleCommon_pb2 import CommandResult
    from keelson.payloads.CommandAuthority_pb2 import Assignment, CommandAssignment, CommandAuthority, RecordKind, Role

    import keelson
    from test_e2e import BASE as BUS, V as EV, Harness  # the harness's vessel, not test_gate's

    h = Harness(tmp_path)
    try:
        heard, errors = [], []

        def on_resolved(s):
            try:
                heard.append(ResolvedControl.FromString(keelson.uncover(s.payload.to_bytes())[2]))
            except Exception as e:  # noqa: BLE001 -- a callback's exception is otherwise swallowed
                errors.append(repr(e))

        sub = h.session.declare_subscriber(f"{BUS}/resolved_control/{EV}", on_resolved)
        h.start_gate(command_profile="operational", attestation={"keys": KEYS})
        h.put(f"command_assignment/{EV}", CommandAssignment(vessel_id=EV, issuer_id="authority-manager/0", feed_ttl_seconds=15,
                                                           navigational=Assignment(assignee="anna@ROC-2", assignment_id="as-7", revision=4)))
        h.put(f"command_authority/{EV}/navigational/session",
              CommandAuthority(vessel_id=EV, controller_id="anna@ROC-2", token="sess-a", lease_ttl_seconds=30,
                               role=Role.ROLE_NAVIGATIONAL_COMMAND, kind=RecordKind.RECORD_KIND_SESSION,
                               assignment=Assignment(assignee="anna@ROC-2", assignment_id="as-7", revision=4)))
        h.wait(lambda: errors or (heard and heard[-1].attestation_challenge), what="a challenge on resolved_control")
        assert not errors, errors
        time.sleep(0.5)
        bare = h.gate("abort_automation", request(), AbortAutomationResponse)
        assert bare.result == CommandResult.COMMAND_RESULT_DENIED and "requires an attestation" in bare.detail
        signed = h.gate("abort_automation", Roc().sign(request(), heard[-1].attestation_challenge), AbortAutomationResponse)
        assert signed.result == CommandResult.COMMAND_RESULT_ACCEPTED, signed.detail
        sub.undeclare()
        out = h.stop("gate")
        assert "attested by roc1-authority" in out and "REFUSED: this vessel requires an attestation" in out
        assert PUBLIC_B64 not in out and json.dumps(KEYS) not in out
    finally:
        h.close()


@pytest.mark.skipif(HAS_FIELDS, reason="only an SDK without AbortAutomationRequest.attestation is refused at start")
@pytest.mark.e2e
def test_a_gate_that_requires_attestation_on_an_sdk_that_cannot_carry_one_refuses_to_start(tmp_path):
    from test_e2e import Harness

    h = Harness(tmp_path)
    try:
        with pytest.raises(AssertionError, match=r"(?s)gate exited.*has no AbortAutomationRequest.attestation"):
            h.start_gate(command_profile="operational", attestation={"keys": KEYS})
    finally:
        h.procs.clear()
        h.close()
