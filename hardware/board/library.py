"""Explicit-pin part templates for SKiDL (no KiCad symbol library needed).

PIN NUMBERS ARE LOGICAL unless a template says otherwise: connectivity is defined by pin NAME.
Before layout, swap each template for the official KiCad symbol/footprint and let KiCad map names to
pads (the netlist uses the footprint names listed here as placeholders). The STM32 template carries
LQFP-64 pad numbers recalled from the STM32G474RETx pinout; they are flagged UNVERIFIED in
board/out/mcu_pins.md and must be checked against DS12288 / the KiCad symbol.

Each template also carries BOM metadata. `mpn_status`:
  "confident"  - part number is a real, current part as far as the author knows (still re-check stock)
  "verify"     - plausible part number, details (suffix, package, rating) must be checked
  "TBD"        - no specific part chosen; any part meeting the description
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field

os.environ.setdefault("KICAD8_SYMBOL_DIR", "")
logging.getLogger("skidl").setLevel(logging.ERROR)

import skidl  # noqa: E402
from skidl import Pin, Part, SKIDL, TEMPLATE  # noqa: E402

T = Pin.types
PWR, GNDP, IN, OUT, BI, PAS, TRI, NC = T.PWRIN, T.PWRIN, T.INPUT, T.OUTPUT, T.BIDIR, T.PASSIVE, T.TRISTATE, T.NOCONNECT
POUT = T.PWROUT


@dataclass
class Meta:
    desc: str
    mfr: str = ""
    mpn: str = ""
    mpn_status: str = "TBD"
    optional: set = field(default_factory=set)     # pins allowed to stay unconnected
    ground_pins: set = field(default_factory=lambda: {"GND"})
    is_ic: bool = False


TEMPLATES: dict[str, tuple[Part, Meta]] = {}


def tmpl(key, prefix, pins, footprint, meta: Meta):
    p = Part(tool=SKIDL, name=key, ref_prefix=prefix, dest=TEMPLATE, footprint=footprint,
             pins=[Pin(num=str(n), name=nm, func=f) for n, nm, f in pins])
    TEMPLATES[key] = (p, meta)
    return p


def two(key, prefix, fp, meta, n1="1", n2="2"):
    return tmpl(key, prefix, [(1, n1, PAS), (2, n2, PAS)], fp, meta)


# ------------------------------------------------------------------ passives / generic
two("R0603", "R", "Resistor_SMD:R_0603_1608Metric", Meta("Resistor 0603 1 %"))
two("R0402", "R", "Resistor_SMD:R_0402_1005Metric", Meta("Resistor 0402 1 %"))
two("R_SHUNT", "R", "Resistor_SMD:R_2512_6332Metric",
    Meta("Current shunt 3 mOhm 1 % 2 W 2512, Kelvin pads", "Vishay", "WSL2512R0030FEA", "verify"))
two("C0402", "C", "Capacitor_SMD:C_0402_1005Metric", Meta("Capacitor 0402 X7R"))
two("C0603", "C", "Capacitor_SMD:C_0603_1608Metric", Meta("Capacitor 0603 X7R/X5R"))
two("C1206", "C", "Capacitor_SMD:C_1206_3216Metric", Meta("Capacitor 1206 X7R 25 V"))
two("C_BULK", "C", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
    Meta("Electrolytic 470 uF 25 V low-ESR, radial 10 mm", "Panasonic", "EEU-FR1E471", "verify"), "+", "-")
two("L_5V", "L", "Inductor_SMD:L_Wuerth_HCI-5040",
    Meta("Inductor ~10 uH, Isat >= 4 A, DCR < 40 mOhm (TPS54331 5 V/3 A)", mpn_status="TBD"))
two("L_3V3", "L", "Inductor_SMD:L_0805_2012Metric",
    Meta("Inductor 2.2 uH, Isat >= 1.5 A", "Murata", "DFE201610E-2R2M", "verify"))
two("FB", "FB", "Inductor_SMD:L_0603_1608Metric", Meta("Ferrite bead 220 Ohm @100 MHz", "Murata", "BLM18PG221SN1D", "confident"))
two("FUSE", "F", "Fuse:Fuse_Littelfuse_NANO2-451_453",
    Meta("Fuse 15 A slow-blow, >= 32 V (or blade fuse in the PSU lead)", mpn_status="TBD"))
two("NTC", "TH", "Resistor_SMD:R_0603_1608Metric", Meta("NTC 10k B3380 0603", "Murata", "NCP18XH103F03RB", "confident"))
two("LED", "D", "LED_SMD:LED_0603_1608Metric", Meta("LED 0603"), "K", "A")
two("SCHOTTKY", "D", "Diode_SMD:D_SMA", Meta("Schottky 40 V 3 A SMA (buck catch diode)", "Diodes Inc", "B340A-13-F", "confident"), "K", "A")
two("ZENER15", "D", "Diode_SMD:D_SOD-123", Meta("Zener 15 V gate clamp", "Diodes Inc", "BZT52C15-7-F", "verify"), "K", "A")
two("TVS_12V", "D", "Diode_SMD:D_SMC", Meta("TVS 15 V standoff unidirectional 1500 W", "Littelfuse", "SMCJ15A", "confident"), "K", "A")
two("TVS_5V", "D", "Diode_SMD:D_SOD-323", Meta("ESD/TVS 5 V unidirectional (servo data line)", "Nexperia", "PESD5V0S1BL", "verify"), "K", "A")
two("XTAL", "Y", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
    Meta("Crystal 24 MHz, CL 8-10 pF, +-20 ppm, 3225", "Abracon", "ABM8-24.000MHZ-B2-T", "verify"))
two("BUTTON", "SW", "Button_Switch_SMD:SW_SPST_PTS810", Meta("Tactile switch SMD", "C&K", "PTS810SJM250SMTRLFS", "verify"))
two("JUMPER2", "JP", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical", Meta("2-pin jumper header + shunt"))
tmpl("TP", "TP", [(1, "TP", PAS)], "TestPoint:TestPoint_Pad_D1.5mm", Meta("Test point pad"))

# ------------------------------------------------------------------ discretes
tmpl("NMOS_PWR", "Q", [(1, "G", IN), (2, "S", PAS), (3, "D", PAS)], "Package_TO_SOT_SMD:TDSON-8-1",
     Meta("N-MOSFET 30 V, < 3 mOhm, SON5x6 (ideal-diode pass FET)", "TI", "CSD17573Q5B", "verify"))
tmpl("PMOS_PWR", "Q", [(1, "G", IN), (2, "S", PAS), (3, "D", PAS)], "Package_TO_SOT_SMD:TO-252-2",
     Meta("P-MOSFET -40 V, ~15 mOhm, Id >= 30 A, DPAK (servo power switch)", "Alpha & Omega", "AOD4185", "verify"))
tmpl("NMOS_SIG", "Q", [(1, "G", IN), (2, "S", PAS), (3, "D", PAS)], "Package_TO_SOT_SMD:SOT-23",
     Meta("N-MOSFET logic level 50 V 200 mA SOT-23", "onsemi", "BSS138LT1G", "verify"))

# ------------------------------------------------------------------ ICs
tmpl("LM74700", "U", [(1, "GND", GNDP), (2, "EN", IN), (3, "VCAP", PAS), (4, "GATE", OUT),
                      (5, "CATHODE", PAS), (6, "ANODE", PWR)], "Package_TO_SOT_SMD:SOT-23-6",
     Meta("Ideal-diode controller (reverse-polarity protection)", "TI", "LM74700QDBVRQ1", "confident", is_ic=True))
tmpl("TPS54331", "U", [(1, "BOOT", PAS), (2, "VIN", PWR), (3, "EN", IN), (4, "SS", PAS), (5, "VSENSE", IN),
                       (6, "COMP", PAS), (7, "GND", GNDP), (8, "PH", OUT)], "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
     Meta("Buck 3.5-28 V in, 3 A (12 V -> 5 V)", "TI", "TPS54331DR", "confident", is_ic=True))
tmpl("TLV62569", "U", [(1, "EN", IN), (2, "GND", GNDP), (3, "SW", OUT), (4, "VIN", PWR), (5, "FB", IN)],
     "Package_TO_SOT_SMD:SOT-23-5", Meta("Sync buck 2.5-5.5 V in, 2 A (5 V -> 3.3 V)", "TI", "TLV62569DBVR", "confident", is_ic=True))
tmpl("INA240", "U", [(1, "GND", GNDP), (2, "IN+", IN), (3, "IN-", IN), (4, "NC", NC), (5, "REF2", IN),
                     (6, "REF1", IN), (7, "OUT", OUT), (8, "VS", PWR)], "Package_SO:TSSOP-8_4.4x3mm_P0.65mm",
     Meta("Current-sense amp, gain 50, -4..80 V CM, PWM rejection", "TI", "INA240A2PWR", "confident",
          optional={"NC"}, is_ic=True))
tmpl("AHCT1G126", "U", [(1, "OE", IN), (2, "A", IN), (3, "GND", GNDP), (4, "Y", TRI), (5, "VCC", PWR)],
     "Package_TO_SOT_SMD:SOT-23-5", Meta("Single buffer, 3-state, OE active high, TTL inputs @5 V (bus TX)",
                                         "TI", "SN74AHCT1G126DBVR", "confident", is_ic=True))
tmpl("LVC1G125", "U", [(1, "~OE", IN), (2, "A", IN), (3, "GND", GNDP), (4, "Y", TRI), (5, "VCC", PWR)],
     "Package_TO_SOT_SMD:SOT-23-5", Meta("Single buffer, 3-state, OE active low, 5 V-tolerant input @3.3 V (bus RX)",
                                         "TI", "SN74LVC1G125DBVR", "confident", is_ic=True))
tmpl("USBLC6", "U", [(1, "IO1", PAS), (2, "GND", GNDP), (3, "IO2", PAS), (4, "IO2b", PAS), (5, "VBUS", PWR),
                     (6, "IO1b", PAS)], "Package_TO_SOT_SMD:SOT-23-6",
     Meta("USB ESD protection, 2 lines + VBUS", "ST", "USBLC6-2SC6", "confident", is_ic=True))
tmpl("TCAN1462V", "U", [(1, "TXD", IN), (2, "GND", GNDP), (3, "VCC", PWR), (4, "RXD", OUT), (5, "VIO", PWR),
                        (6, "CANL", BI), (7, "CANH", BI), (8, "STB", IN)], "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
     Meta("CAN-FD transceiver with VIO (8 Mbps, SIC) - v2 joint modules", "TI", "TCAN1462VDRQ1", "verify", is_ic=True))
tmpl("PESD2CAN", "D", [(1, "CANH", PAS), (2, "CANL", PAS), (3, "GND", PAS)], "Package_TO_SOT_SMD:SOT-23",
     Meta("CAN bus ESD protection", "Nexperia", "PESD2CAN,215", "verify"))

# ------------------------------------------------------------------ connectors
tmpl("CONN_PWR", "J", [(1, "+", PAS), (2, "-", PAS)], "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal",
     Meta("12 V input, XT30 PCB male (rated 15 A+)", "Amass", "XT30PW-M", "verify"))
tmpl("CONN_ESTOP", "J", [(1, "LOOP_OUT", PAS), (2, "LOOP_RET", PAS)], "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
     Meta("E-stop loop (NC mushroom button), JST-XH 2p", "JST", "B2B-XH-A(LF)(SN)", "confident"))
tmpl("CONN_SERVO", "J", [(1, "GND", PAS), (2, "V+", PAS), (3, "DATA", PAS)], "Connector_Molex:Molex_Micro-Latch_53254-0370_1x03_P2.00mm_Vertical",
     Meta("Servo bus port, 3-pin (match the servo cable: pin order and housing MUST be checked against the STS3215 lead)", mpn_status="TBD"))
tmpl("CONN_UART", "J", [(1, "TX", PAS), (2, "RX", PAS), (3, "GND", PAS)], "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
     Meta("Debug UART header 3p"))
tmpl("CONN_CAN", "J", [(1, "CANH", PAS), (2, "CANL", PAS), (3, "GND", PAS)], "Connector_JST:JST_GH_SM03B-GHS-TB_1x03-1MP_P1.25mm_Horizontal",
     Meta("CAN-FD port (v2), JST-GH 3p", "JST", "SM03B-GHS-TB(LF)(SN)", "verify"))
tmpl("CONN_SWD", "J", [(1, "VTREF", PAS), (2, "SWDIO", PAS), (3, "GND1", PAS), (4, "SWCLK", PAS), (5, "GND2", PAS),
                       (6, "SWO", PAS), (7, "KEY", NC), (8, "NC", NC), (9, "GNDDET", PAS), (10, "NRST", PAS)],
     "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
     Meta("Cortex-M 10-pin SWD header 1.27 mm", "Samtec", "FTSH-105-01-F-DV-K", "verify", optional={"KEY", "NC"}))
tmpl("USB_C", "J", [(1, "VBUS", PAS), (2, "GND", PAS), (3, "CC1", PAS), (4, "CC2", PAS), (5, "DP1", PAS), (6, "DP2", PAS),
                    (7, "DN1", PAS), (8, "DN2", PAS), (9, "SBU1", NC), (10, "SBU2", NC), (11, "SHIELD", PAS)],
     "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal",
     Meta("USB-C receptacle, USB 2.0 device", "GCT", "USB4105-GF-A", "verify", optional={"SBU1", "SBU2"}))

# ------------------------------------------------------------------ MCU (LQFP-64)
# Pad numbers recalled from the STM32G474RETx LQFP-64 pinout - UNVERIFIED, check against DS12288.
MCU_PADS = [
    (1, "VBAT"), (2, "PC13"), (3, "PC14"), (4, "PC15"), (5, "PF0-OSC_IN"), (6, "PF1-OSC_OUT"), (7, "PG10-NRST"),
    (8, "PC0"), (9, "PC1"), (10, "PC2"), (11, "PC3"), (12, "PA0"), (13, "PA1"), (14, "PA2"), (15, "VSS_1"),
    (16, "VDD_1"), (17, "PA3"), (18, "PA4"), (19, "PA5"), (20, "PA6"), (21, "PA7"), (22, "PC4"), (23, "PC5"),
    (24, "PB0"), (25, "PB1"), (26, "PB2"), (27, "VSSA"), (28, "VREF+"), (29, "VDDA"), (30, "PB10"), (31, "VSS_2"),
    (32, "VDD_2"), (33, "PB11"), (34, "PB12"), (35, "PB13"), (36, "PB14"), (37, "PB15"), (38, "PC6"), (39, "PC7"),
    (40, "PC8"), (41, "PC9"), (42, "PA8"), (43, "PA9"), (44, "PA10"), (45, "PA11"), (46, "PA12"), (47, "VSS_3"),
    (48, "VDD_3"), (49, "PA13"), (50, "PA14"), (51, "PA15"), (52, "PC10"), (53, "PC11"), (54, "PC12"), (55, "PD2"),
    (56, "PB3"), (57, "PB4"), (58, "PB5"), (59, "PB6"), (60, "PB7"), (61, "PB8-BOOT0"), (62, "PB9"), (63, "VSS_4"),
    (64, "VDD_4"),
]
_POWER = {"VBAT", "VDD_1", "VDD_2", "VDD_3", "VDD_4", "VDDA", "VREF+"}
_GND = {"VSS_1", "VSS_2", "VSS_3", "VSS_4", "VSSA"}
_REQUIRED_IO = {"PF0-OSC_IN", "PF1-OSC_OUT", "PG10-NRST", "PB8-BOOT0", "PA13", "PA14"}
tmpl("STM32G474RET6", "U",
     [(n, nm, PWR if nm in _POWER else GNDP if nm in _GND else BI) for n, nm in MCU_PADS],
     "Package_QFP:LQFP-64_10x10mm_P0.5mm",
     Meta("MCU Cortex-M4F 170 MHz, 512 kB flash, FDCAN, USB-FS, LQFP-64", "ST", "STM32G474RET6", "confident",
          optional={nm for _, nm in MCU_PADS if nm not in _POWER | _GND | _REQUIRED_IO},
          ground_pins=_GND, is_ic=True))
