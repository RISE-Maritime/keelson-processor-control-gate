"""The ROC service's attestation on the DIRECT abort, verified by the gate itself (crowsnest-dev#316;
keelson control_gate/v1 `AbortAutomationRequest.attestation`; COMMAND-ARCHITECTURE §6, trust chain B).

`requester_id` is a label. Who is really asking is the ROC authority service's Ed25519-signed
`AttestationClaim` -- the authority manager's own type (keelson#383), so one service signs both --
checked here against the public keys the vessel holds (policy `attestation.keys`, maintenance
configuration, never over the link), in the protocol's order, refusing at the first failure with the
reason:

  1. the claim parses          5. the sequence is new and rising for (key_id, session_id)
  2. its key is one we trust   6. it names THIS call: "control_gate/v1/abort_automation"
  3. the signature is good     7. its request digest is THIS request's
  4. its challenge is live     8. requester_id / requester_site equal its operator and site

THE GATE'S CHALLENGES, NOT THE MANAGER'S. This call exists for when the manager is down, so the gate
draws its own, publishes the newest on resolved_control (`attestation_challenge`), and ages them on its
own monotonic clock (§0). They are never persisted: after a restart every earlier attestation is stale.
The procedure is qualified so that a claim for the manager's `abort_automation` is never this one.

A PORT of keelson-processor-control-authority's control_authority/attestation.py, which verifies the
manager's calls. The two differ only in where the attestation and the label sit on the request;
tests/test_abort_attestation.py and the manager's tests pin both to one signed vector, and moving the
verifier into the keelson Python SDK is a follow-up.

Pure apart from the signature check: no clock (it is passed in), no store, no Zenoh.
"""

from __future__ import annotations

import base64
import hashlib
import secrets
from dataclasses import dataclass, field

ACCEPTED = "accepted"
PROCEDURE = "control_gate/v1/abort_automation"
MISSING = "this vessel requires an attestation from a ROC service it trusts on the direct abort; none was presented"


@dataclass(frozen=True)
class TrustedKey:
    key_id: str
    service_id: str
    public_key: bytes  # raw 32-byte Ed25519


def parse_keys(d: dict) -> dict:
    """`{key_id: {service_id, public_key (base64, raw 32 bytes)}}` -> {key_id: TrustedKey}."""
    keys = {}
    for key_id, k in (d or {}).items():
        if not isinstance(k, dict) or not k.get("service_id") or not k.get("public_key"):
            raise ValueError(f"attestation key {key_id!r} needs service_id and public_key")
        try:
            raw = base64.b64decode(k["public_key"], validate=True)
        except (ValueError, TypeError) as e:
            raise ValueError(f"attestation key {key_id!r}: public_key is not base64") from e
        if len(raw) != 32:
            raise ValueError(f"attestation key {key_id!r}: an Ed25519 public key is 32 bytes, not {len(raw)}")
        keys[str(key_id)] = TrustedKey(str(key_id), str(k["service_id"]), raw)
    return keys


def request_digest(request) -> bytes:
    """SHA-256 of the AbortAutomationRequest serialized deterministically, with `attestation` cleared."""
    bare = type(request)()
    bare.CopyFrom(request)
    bare.ClearField("attestation")
    return hashlib.sha256(bare.SerializeToString(deterministic=True)).digest()


@dataclass
class Challenges:
    """The challenges this gate accepts now: drawn every `rotate_ms`, each valid `ttl_ms` on our own
    monotonic clock, newest first."""

    ttl_ms: int
    issued: list = field(default_factory=list)  # [(challenge, issued_ms)], newest first

    def rotate(self, now_ms: int, draw=lambda: secrets.token_bytes(16)) -> bytes:
        self.issued.insert(0, (draw(), now_ms))
        self.prune(now_ms)
        return self.issued[0][0]

    def prune(self, now_ms: int) -> None:
        self.issued = [(c, t) for c, t in self.issued if now_ms - t <= self.ttl_ms]

    def live(self, now_ms: int) -> list[bytes]:
        self.prune(now_ms)
        return [c for c, _ in self.issued]

    def newest(self, now_ms: int) -> bytes:
        live = self.live(now_ms)
        return live[0] if live else b""

    def drawn_ms(self) -> int | None:
        return self.issued[0][1] if self.issued else None


@dataclass(frozen=True)
class Verdict:
    outcome: str  # ACCEPTED, or the reason refused
    principal: dict | None = None  # who, as attested; on acceptance only

    @property
    def ok(self) -> bool:
        return self.outcome == ACCEPTED


def verify(attestation, request, keys: dict, challenges: Challenges, seen: dict, now_ms: int) -> Verdict:
    """Check the direct abort's attestation. `seen` maps (key_id, session_id) to the last accepted
    sequence and is updated only when the whole attestation is accepted."""
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    from keelson.interfaces.AuthorityManager_pb2 import AttestationClaim

    try:
        claim = AttestationClaim.FromString(attestation.claim)
    except Exception:  # noqa: BLE001 -- any decode failure is the same refusal
        return Verdict("the attestation's claim does not parse")
    key = keys.get(claim.key_id)
    if key is None:
        return Verdict(f"attestation key {claim.key_id!r} is not one this vessel trusts")
    if claim.service_id != key.service_id:
        return Verdict(f"key {claim.key_id!r} belongs to {key.service_id}, not {claim.service_id}")
    try:
        Ed25519PublicKey.from_public_bytes(key.public_key).verify(attestation.signature, attestation.claim)
    except InvalidSignature:
        return Verdict("the attestation's signature does not verify")
    if claim.challenge not in challenges.live(now_ms):
        return Verdict("the attestation's challenge is not one this gate issued within its TTL "
                       "(read it from resolved_control: the manager's are not the gate's)")
    last = seen.get((claim.key_id, claim.session_id))
    if last is not None and claim.sequence <= last:
        return Verdict(f"sequence {claim.sequence} is not above {last} for this session: a replay")
    if claim.procedure != PROCEDURE:
        return Verdict(f"the attestation is for {claim.procedure!r}, not {PROCEDURE!r}")
    if claim.request_sha256 != request_digest(request):
        return Verdict("the attestation is for another request: its digest does not match")
    if (request.requester_id, request.requester_site) != (claim.operator_id, claim.site):
        return Verdict(f"the label {request.requester_id}@{request.requester_site or '?'} is not the attested "
                       f"{claim.operator_id} at {claim.site}")
    seen[(claim.key_id, claim.session_id)] = claim.sequence
    return Verdict(ACCEPTED, {"operator": claim.operator_id, "site": claim.site, "station": claim.station,
                              "service": claim.service_id, "key_id": claim.key_id, "session": claim.session_id,
                              "sequence": claim.sequence})
