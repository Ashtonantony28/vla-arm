"""Controller board schematic as code (SKiDL, explicit-pin parts, no KiCad library dependency).

Blocks (docs/PLAN.md 3.2): 12 V input (fuse, LM74700 ideal diode, TVS, bulk), 5 V + 3.3 V bucks,
STM32G474RET6 (decoupling, 24 MHz HSE, SWD, BOOT0, reset), USB-FS with ESD, half-duplex servo bus
(3-state buffers, hardware DE), INA240 high-side servo-rail current sense + rail dividers, NTC,
hardware e-stop loop cutting servo power via a P-MOSFET (firmware can only AND it off and read it),
FDCAN transceiver + termination jumper (v2), LEDs, test points.

build() returns a Board: the SKiDL circuit plus per-part metadata (BOM fields, decoupling intent,
rail anchors) used by board/erc.py.
"""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import skidl
from skidl import Net, POWER

from .library import TEMPLATES
from .pinmap import ASSIGNMENTS

RAILS = ["VIN_RAW", "VIN_F", "V12", "SH_HI", "V12_SERVO", "+5V", "+3V3", "+3V3A", "USB_VBUS", "GND"]


@dataclass
class Board:
    circuit: object
    meta: dict = field(default_factory=dict)      # ref -> dict(key, meta, value, decouples, rail)
    nets: dict = field(default_factory=dict)      # name -> Net

    def part(self, ref):
        return next(p for p in self.circuit.parts if p.ref == ref)


class _Builder:
    def __init__(self):
        self.count = defaultdict(int)
        import builtins
        self.b = Board(builtins.default_circuit)

    def net(self, name) -> Net:
        if name not in self.b.nets:
            n = Net(name)
            if name in RAILS:
                n.drive = POWER
            self.b.nets[name] = n
        return self.b.nets[name]

    def part(self, key, value="", ref=None, decouples=None, rail=None, conn=None):
        tp, meta = TEMPLATES[key]
        if ref is None:
            while True:
                self.count[tp.ref_prefix] += 1
                ref = f"{tp.ref_prefix}{self.count[tp.ref_prefix]}"
                if ref not in self.b.meta:
                    break
        assert ref not in self.b.meta, f"duplicate ref {ref}"
        p = tp(ref=ref, value=value or meta.mpn or key, tag=ref)
        self.b.meta[ref] = dict(key=key, meta=meta, value=value or meta.mpn or key,
                                decouples=set([decouples] if isinstance(decouples, str) else (decouples or [])),
                                rail=rail)
        for pin, netname in (conn or {}).items():
            self.net(netname).__iadd__(p[pin])
        return p

    # two-terminal helpers
    def r(self, value, a, b, key="R0603"):
        return self.part(key, value, conn={"1": a, "2": b})

    def c(self, value, a, b="GND", key="C0402", decouples=None):
        return self.part(key, value, conn={"1": a, "2": b}, decouples=decouples)

    def tp(self, net, rail=None):
        return self.part("TP", net, conn={"TP": net}, rail=rail)


def build() -> Board:
    skidl.reset()
    B = _Builder()
    r, c, part, tp = B.r, B.c, B.part, B.tp

    # ------------------------------------------------------------- 12 V input + protection
    part("CONN_PWR", "12V IN", ref="J1", conn={"+": "VIN_RAW", "-": "GND"})
    part("FUSE", "15A slow", ref="F1", conn={"1": "VIN_RAW", "2": "VIN_F"})
    part("LM74700", ref="U1", conn={"ANODE": "VIN_F", "CATHODE": "V12", "GATE": "IDEAL_GATE",
                                    "VCAP": "IDEAL_VCAP", "EN": "VIN_F", "GND": "GND"})
    c("100n", "IDEAL_VCAP", "VIN_F", decouples="U1")            # charge-pump cap VCAP-ANODE
    c("100n", "VIN_F", "GND", key="C0603", decouples="U1")
    part("NMOS_PWR", ref="Q1", conn={"G": "IDEAL_GATE", "S": "VIN_F", "D": "V12"})
    part("TVS_12V", ref="D1", conn={"K": "V12", "A": "GND"})
    for _ in range(2):
        part("C_BULK", "470u 25V", conn={"+": "V12", "-": "GND"})
    for _ in range(2):
        c("10u 25V", "V12", key="C1206")
    r("100k", "V12", "VIN_ADC"); r("15k", "VIN_ADC", "GND"); c("100n", "VIN_ADC")

    # ------------------------------------------------------------- 5 V buck (TPS54331)
    part("TPS54331", ref="U2", conn={"VIN": "V12", "GND": "GND", "EN": "EN_5V", "BOOT": "BOOT_5V",
                                     "PH": "SW_5V", "VSENSE": "FB_5V", "COMP": "COMP_5V", "SS": "SS_5V"})
    c("10u 25V", "V12", key="C1206", decouples="U2"); c("10u 25V", "V12", key="C1206", decouples="U2")
    c("100n", "V12", decouples="U2")
    r("330k", "V12", "EN_5V"); r("68k", "EN_5V", "GND")        # UVLO ~7 V (recompute per datasheet)
    c("100n", "BOOT_5V", "SW_5V", key="C0603")
    part("SCHOTTKY", ref="D2", conn={"K": "SW_5V", "A": "GND"})
    part("L_5V", "10u", ref="L1", conn={"1": "SW_5V", "2": "+5V"})
    c("22u 10V", "+5V", key="C1206"); c("22u 10V", "+5V", key="C1206")
    r("52.3k", "+5V", "FB_5V"); r("10k", "FB_5V", "GND")       # 0.8 V ref -> 4.98 V
    r("7.5k", "COMP_5V", "COMP_RC"); c("6.8n", "COMP_RC"); c("47p", "COMP_5V")
    c("15n", "SS_5V")

    # ------------------------------------------------------------- 3.3 V buck (TLV62569) + analog rail
    part("TLV62569", ref="U3", conn={"VIN": "+5V", "EN": "+5V", "GND": "GND", "SW": "SW_3V3", "FB": "FB_3V3"})
    c("10u 10V", "+5V", key="C0603", decouples="U3")
    part("L_3V3", "2.2u", ref="L2", conn={"1": "SW_3V3", "2": "+3V3"})
    c("22u 6.3V", "+3V3", key="C0603")
    r("100k", "+3V3", "FB_3V3"); r("22.1k", "FB_3V3", "GND")   # 0.6 V ref -> 3.31 V
    part("FB", ref="FB1", conn={"1": "+3V3", "2": "+3V3A"})
    r("1k", "+3V3", "LED_PWR_A"); part("LED", "green PWR", conn={"A": "LED_PWR_A", "K": "GND"})

    # ------------------------------------------------------------- MCU
    mcu_conn = {"VBAT": "+3V3", "VDD_1": "+3V3", "VDD_2": "+3V3", "VDD_3": "+3V3", "VDD_4": "+3V3",
                "VDDA": "+3V3A", "VREF+": "+3V3A",
                "VSS_1": "GND", "VSS_2": "GND", "VSS_3": "GND", "VSS_4": "GND", "VSSA": "GND"}
    for a in ASSIGNMENTS:
        mcu_conn[a.pin] = a.signal
    part("STM32G474RET6", ref="U4", conn=mcu_conn)
    for _ in range(4):
        c("100n", "+3V3", decouples="U4")
    c("4.7u", "+3V3", key="C0603", decouples="U4")
    c("100n", "+3V3", decouples="U4")                         # VBAT
    c("1u", "+3V3A", key="C0603", decouples="U4"); c("10n", "+3V3A", decouples="U4")
    c("100n", "+3V3A", decouples="U4")                        # VREF+
    part("XTAL", "24MHz", ref="Y1", conn={"1": "OSC_IN", "2": "OSC_OUT"})
    c("10p C0G", "OSC_IN"); c("10p C0G", "OSC_OUT")
    c("100n", "NRST")
    part("BUTTON", "RESET", ref="SW1", conn={"1": "NRST", "2": "GND"})
    r("10k", "BOOT0", "GND")
    part("BUTTON", "BOOT", ref="SW2", conn={"1": "BOOT0", "2": "+3V3"})
    part("CONN_SWD", ref="J2", conn={"VTREF": "+3V3", "SWDIO": "SWDIO", "SWCLK": "SWCLK", "SWO": "SWO",
                                     "NRST": "NRST", "GND1": "GND", "GND2": "GND", "GNDDET": "GND"})
    part("CONN_UART", ref="J3", conn={"TX": "LOG_TX", "RX": "LOG_RX", "GND": "GND"})
    for i, sig in ((1, "LED1_DRV"), (2, "LED2_DRV")):
        r("330", sig, f"LED{i}_A"); part("LED", f"status{i}", conn={"A": f"LED{i}_A", "K": "GND"})

    # ------------------------------------------------------------- USB-FS
    part("USB_C", ref="J4", conn={"VBUS": "USB_VBUS", "GND": "GND", "CC1": "USB_CC1", "CC2": "USB_CC2",
                                  "DP1": "USB_DP", "DP2": "USB_DP", "DN1": "USB_DM", "DN2": "USB_DM",
                                  "SHIELD": "USB_SHIELD"})
    r("5.1k", "USB_CC1", "GND"); r("5.1k", "USB_CC2", "GND")
    r("1M", "USB_SHIELD", "GND"); c("4.7n 100V", "USB_SHIELD", key="C0603")
    part("USBLC6", ref="U5", conn={"IO1": "USB_DP", "IO1b": "USB_DP", "IO2": "USB_DM", "IO2b": "USB_DM",
                                   "VBUS": "USB_VBUS", "GND": "GND"})
    c("100n", "USB_VBUS", decouples="U5")
    r("10k", "USB_VBUS", "VBUS_SENSE"); r("20k", "VBUS_SENSE", "GND")

    # ------------------------------------------------------------- servo bus (half duplex, 1 Mbps)
    part("AHCT1G126", ref="U6", conn={"A": "SERVO_TX", "OE": "SERVO_DIR", "Y": "BUS_TX_OUT",
                                      "VCC": "+5V", "GND": "GND"})
    c("100n", "+5V", decouples="U6")
    r("100", "BUS_TX_OUT", "SERVO_DATA")
    part("LVC1G125", ref="U7", conn={"A": "SERVO_DATA", "~OE": "SERVO_DIR", "Y": "SERVO_RX",
                                     "VCC": "+3V3", "GND": "GND"})
    c("100n", "+3V3", decouples="U7")
    r("10k", "SERVO_RX", "+3V3")            # idle-high while the RX buffer is disabled (no echo)
    r("10k", "SERVO_DIR", "GND")            # TX driver off during reset
    r("10k", "SERVO_DATA", "+5V")           # bus idle high
    part("TVS_5V", conn={"K": "SERVO_DATA", "A": "GND"})
    for i in range(3):
        part("CONN_SERVO", f"SERVO{i + 1}", conn={"GND": "GND", "V+": "V12_SERVO", "DATA": "SERVO_DATA"})

    # ------------------------------------------------------------- servo power switch + hardware e-stop
    part("PMOS_PWR", ref="Q2", conn={"S": "V12", "D": "SH_HI", "G": "GATE_SW"})
    r("10k", "V12", "GATE_SW")                                  # off unless pulled down
    part("ZENER15", conn={"K": "V12", "A": "GATE_SW"})          # |Vgs| <= 15 V
    c("47n 50V", "GATE_SW", "SH_HI", key="C0603")               # Miller cap: slow turn-on, limits inrush
    r("4.7k", "GATE_SW", "GSW_LOW")
    part("NMOS_SIG", ref="Q3", conn={"D": "GSW_LOW", "S": "GSW_MID", "G": "ESTOP_G"})       # e-stop leg
    part("NMOS_SIG", ref="Q4", conn={"D": "GSW_MID", "S": "GND", "G": "SERVO_PWR_EN_G"})   # firmware leg
    r("100", "SERVO_PWR_EN", "SERVO_PWR_EN_G"); r("100k", "SERVO_PWR_EN_G", "GND")
    r("2.2k", "V12", "ESTOP_OUT")                               # loop current limit (short-safe)
    part("CONN_ESTOP", ref="J8", conn={"LOOP_OUT": "ESTOP_OUT", "LOOP_RET": "ESTOP_RET"})
    r("22k", "ESTOP_RET", "ESTOP_G"); r("10k", "ESTOP_G", "GND"); c("100n", "ESTOP_G")
    r("100k", "ESTOP_RET", "ESTOP_SENSE"); r("33k", "ESTOP_SENSE", "GND")
    r("4.7k", "V12_SERVO", "LED_SERVO_A"); part("LED", "amber SERVO PWR", conn={"A": "LED_SERVO_A", "K": "GND"})

    # ------------------------------------------------------------- current sense + rail sense + NTC
    part("R_SHUNT", "3m", ref="R_SH", conn={"1": "SH_HI", "2": "V12_SERVO"})
    part("INA240", ref="U8", conn={"IN+": "SH_HI", "IN-": "V12_SERVO", "VS": "+3V3", "GND": "GND",
                                   "REF1": "GND", "REF2": "GND", "OUT": "ISENSE_RAW"})
    c("100n", "+3V3", decouples="U8")
    r("100", "ISENSE_RAW", "ISENSE_ADC"); c("1n C0G", "ISENSE_ADC")
    for _ in range(2):
        part("C_BULK", "470u 25V", conn={"+": "V12_SERVO", "-": "GND"})
    c("10u 25V", "V12_SERVO", key="C1206")
    r("100k", "V12_SERVO", "VSERVO_ADC"); r("15k", "VSERVO_ADC", "GND"); c("100n", "VSERVO_ADC")
    r("10k", "+3V3A", "NTC_ADC"); part("NTC", "10k", ref="TH1", conn={"1": "NTC_ADC", "2": "GND"})
    c("100n", "NTC_ADC")

    # ------------------------------------------------------------- FDCAN (v2)
    part("TCAN1462V", ref="U9", conn={"TXD": "CAN_TXD", "RXD": "CAN_RXD", "VCC": "+5V", "VIO": "+3V3",
                                      "GND": "GND", "STB": "CAN_STB", "CANH": "CANH", "CANL": "CANL"})
    c("100n", "+5V", decouples="U9"); c("100n", "+3V3", decouples="U9")
    r("10k", "CAN_STB", "+3V3")
    part("JUMPER2", "CAN TERM", ref="JP1", conn={"1": "CANH", "2": "CAN_TERM"})
    r("60.4", "CAN_TERM", "CAN_SPLIT"); r("60.4", "CAN_SPLIT", "CANL"); c("4.7n", "CAN_SPLIT")
    part("PESD2CAN", conn={"CANH": "CANH", "CANL": "CANL", "GND": "GND"})
    part("CONN_CAN", ref="J9", conn={"CANH": "CANH", "CANL": "CANL", "GND": "GND"})

    # ------------------------------------------------------------- test points (also rail anchors for the short check)
    for n in ("VIN_F", "V12", "V12_SERVO", "+5V", "+3V3", "+3V3A", "GND"):
        tp(n, rail=n)
    tp("GND", rail="GND")
    for n in ("ISENSE_ADC", "SERVO_DATA", "ESTOP_RET", "GATE_SW", "SERVO_PWR_EN"):
        tp(n)
    return B.b
