"""Board netlist checks: ERC clean, the ERC actually catches faults, pin map, BOM, outputs."""
import csv

import pytest

from board import erc, generate, schematic
from board.pinmap import AF_FACTS, ASSIGNMENTS


def _errors(b):
    return [f for f in erc.check(b) if f.level == "error"]


@pytest.fixture(scope="module")
def outputs():
    return generate.generate()


def test_erc_clean(outputs):
    board, findings, sk, st = outputs
    assert [f for f in findings if f.level == "error"] == []
    assert sk["errors"] == []
    # SKiDL warnings are only the deliberately unused MCU GPIOs
    assert all("Unconnected pin" in w and "STM32G474" in w for w in sk["warnings"])


def test_netlist_written_with_all_parts(outputs):
    board, _, _, st = outputs
    txt = (generate.OUT / "controller.net").read_text()
    import re
    assert len(re.findall(r"\(comp\s*\n\s*\(ref", txt)) == st["parts"] > 100
    for ref in ("U1", "U2", "U3", "U4", "U8", "U9", "Q2", "R_SH", "J8"):
        assert f'(ref "{ref}")' in txt


def test_erc_detects_unconnected_required_pin():
    b = schematic.build()
    b.part("U2")["VSENSE"].disconnect()
    assert any(f.rule == "R1" and "U2.VSENSE" in f.msg for f in _errors(b))


def test_erc_detects_shorted_rails():
    b = schematic.build()
    b.nets["+5V"] += b.nets["+3V3"]
    assert any(f.rule == "R2" for f in _errors(b))


def test_erc_detects_missing_decoupling():
    b = schematic.build()
    cap = next(ref for ref, m in b.meta.items() if m["decouples"] == {"U8"})
    for pin in b.part(cap).pins:
        pin.disconnect()
    assert any(f.rule == "R3" and "U8" in f.msg for f in _errors(b))


def test_every_ic_has_decoupling_cap():
    b = schematic.build()
    ics = [ref for ref, m in b.meta.items() if m["meta"].is_ic]
    tagged = set().union(*(m["decouples"] for m in b.meta.values()))
    assert set(ics) <= tagged


def test_estop_chain_topology():
    """Servo power needs BOTH the e-stop loop (Q3) and firmware enable (Q4) to pull the P-FET gate low;
    firmware can read the loop state; the loop returns only through the external NC contact."""
    b = schematic.build()
    net = lambda ref, pin: b.part(ref)[pin].net.name
    assert net("Q2", "S") == "V12" and net("Q2", "D") == "SH_HI" and net("Q2", "G") == "GATE_SW"
    assert net("Q3", "D") == "GSW_LOW" and net("Q3", "S") == net("Q4", "D") == "GSW_MID"
    assert net("Q4", "S") == "GND" and net("Q3", "G") == "ESTOP_G"
    assert net("Q4", "G") == "SERVO_PWR_EN_G"
    assert net("J8", "LOOP_OUT") == "ESTOP_OUT" and net("J8", "LOOP_RET") == "ESTOP_RET"
    mcu_nets = {p.net.name for p in b.part("U4").pins if p.net is not None and len(p.net.pins)}
    assert {"ESTOP_SENSE", "SERVO_PWR_EN"} <= mcu_nets
    # the MCU has no path to ESTOP_G / GATE_SW (cannot override the e-stop)
    assert not ({"ESTOP_G", "GATE_SW", "GSW_LOW"} & mcu_nets)


def test_current_sense_kelvin_and_adc():
    b = schematic.build()
    net = lambda ref, pin: b.part(ref)[pin].net.name
    assert {net("R_SH", "1"), net("R_SH", "2")} == {net("U8", "IN+"), net("U8", "IN-")}
    assert net("U8", "VS") == "+3V3"


def test_pin_assignment_unique_and_af_backed():
    pins = [a.pin for a in ASSIGNMENTS]
    assert len(pins) == len(set(pins))
    for a in ASSIGNMENTS:
        if a.af.startswith("AF"):
            assert AF_FACTS[(a.pin, a.function)] == a.af
    assert not {"PB4", "PB6"} & set(pins)           # UCPD dead-battery pull-downs
    assert next(a for a in ASSIGNMENTS if a.signal == "CAN_RXD").pin != "PB8-BOOT0"


def test_bom_complete(outputs):
    board, *_ = outputs
    rows = list(csv.DictReader(open(generate.OUT / "bom.csv")))
    assert sum(int(r["qty"]) for r in rows) == len(board.circuit.parts)
    assert all(r["mpn_status"] in ("confident", "verify", "TBD", "generic") for r in rows)
    mcu = next(r for r in rows if r["refs"] == "U4")
    assert mcu["mpn"] == "STM32G474RET6"
