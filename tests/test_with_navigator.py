"""The navigator's own candidates through the gate's own checks (crowsnest-dev#94): what
keelson-processor-navigation's guidance/candidates.py decides and bus/gate_candidates.py puts on the
wire, decoded by this gate's codec and judged by on_guidance, under both bases. Skipped unless the
navigator is installed (--no-deps, as the manager is for test_with_manager.py)."""

from __future__ import annotations

import pytest

from control_gate import codec, model
from control_gate.model import Mapping
from test_gate import DOFS, NAV, NAV_PRINCIPAL, TED, TOKEN, Clock, clear, lease, make

nav_c = pytest.importorskip("nav.guidance.candidates")
nav_bus = pytest.importorskip("nav.bus.gate_candidates")
nav_fn = pytest.importorskip("nav.conduct.functions")
nav_config = pytest.importorskip("nav.config")


@pytest.fixture
def clock():
    return Clock()


def _gate_with_navigator_rows(tmp_path, clock, holder=TED):
    from control_gate.model import Proposal, Sample

    g = make(tmp_path, clock)
    clear(g)
    lease(g, holder, TOKEN)
    nav_rows = tuple(Mapping(d, holder, TOKEN, NAV, (), "", f"m-{d}") for d in DOFS)
    for i, d in enumerate(DOFS):
        g.sample(Sample(NAV, d, 0, "warmup", i + 1, clock()))
    ack = g.prepare(Proposal("t1", model.GRANT, nav_rows, "ROC-1", 2000, holder))
    assert ack.accepted, ack.reason
    g.commit("t1", ack.fence)
    return g, nav_rows


def _navigator(conf_dofs):
    conf = nav_config.DEFAULTS | {"entity_id": "usv-alpha"}
    conf = {**conf, "guidance": {**conf["guidance"], "backend": "gate",
                                 "gate": {**nav_config.GATE_DEFAULTS, "dofs": conf_dofs}}}
    return nav_bus.GateBackend(None, conf, None)


def _doc(*held, revision=7):
    return nav_fn.read_document({"navigation_authority": {"revision": str(revision), "functions": [
        {"function": f"NAVIGATION_FUNCTION_{f.upper()}", "holder": NAV_PRINCIPAL,
         "control_method": "CONTROL_METHOD_MONITORED"} for f in held]}})


def _through_the_gate(g, clock, backend, found):
    from keelson.payloads.GuidanceCandidate_pb2 import GuidanceCandidate

    results = []
    for c in found:
        pb = GuidanceCandidate.FromString(backend.message(c, 10**18).SerializeToString())
        results.append(g.guidance(codec.guidance_from_proto(pb, clock())))
    return results


def _map_of(g, holder):
    return nav_c.GateMap(tuple(nav_c.GateRow(m.dof, m.mapping_id, m.authority_holder, m.execution_source)
                               for m in g.state.mappings), (g.state.term, g.state.generation))


def test_nav_function_under_carry_out_passes_both_checks(tmp_path, clock):
    g, _ = _gate_with_navigator_rows(tmp_path, clock)
    g.observe_navigation(7, {"track_keeping": (NAV_PRINCIPAL, 4), "speed_control": (NAV_PRINCIPAL, 4)})
    dofs = {d: "steering" if d == "rudder" else "throttle" for d in DOFS}
    found, why = nav_c.candidates((5.0, 20.0), _map_of(g, TED), dofs, NAV, NAV_PRINCIPAL,
                                  _doc("track_keeping", "speed_control"), TED, None, False, None, "track", None)
    assert found and why == ()
    results = _through_the_gate(g, clock, _navigator(dofs), found)
    assert all(ok for ok, _ in results), results


def test_human_order_passes_when_the_navigators_state_names_it(tmp_path, clock):
    g, _ = _gate_with_navigator_rows(tmp_path, clock)
    g.observe_navigation(7, {})
    dofs = {d: "steering" if d == "rudder" else "throttle" for d in DOFS}
    found, _ = nav_c.candidates((5.0, 20.0), _map_of(g, TED), dofs, NAV, NAV_PRINCIPAL, None, TED, None,
                                False, None, "track", "o-42")
    backend = _navigator(dofs)
    refused = _through_the_gate(g, clock, backend, found)
    assert not any(ok for ok, _ in refused)  # the gate has heard no navigation_state yet
    g.observe_navigation_state("navigation/0/conduct", "o-42")
    assert all(ok for ok, _ in _through_the_gate(g, clock, backend, found))
    g.observe_navigation_state("navigation/0/conduct", "o-43")  # a newer order: the old id is stale
    assert not any(ok for ok, _ in _through_the_gate(g, clock, backend, found))


def test_direct_under_full_automation(tmp_path, clock):
    g, _ = _gate_with_navigator_rows(tmp_path, clock, holder=NAV_PRINCIPAL)
    g.observe_navigation(7, {"track_keeping": (NAV_PRINCIPAL, 4), "speed_control": (NAV_PRINCIPAL, 4)})
    dofs = {d: "steering" if d == "rudder" else "throttle" for d in DOFS}
    found, _ = nav_c.candidates((5.0, 20.0), _map_of(g, NAV_PRINCIPAL), dofs, NAV, NAV_PRINCIPAL,
                                _doc("track_keeping", "speed_control"), NAV_PRINCIPAL, None, False, None,
                                "track", None)
    assert {c.execution for c in found} == {"direct"}
    assert all(ok for ok, _ in _through_the_gate(g, clock, _navigator(dofs), found))
