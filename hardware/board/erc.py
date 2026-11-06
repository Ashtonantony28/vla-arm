"""Own electrical-rule checks on the SKiDL circuit (independent of SKiDL's built-in ERC).

check(board) -> list of Finding(level, rule, message). level "error" fails the tests.
Rules:
  R1 required pins connected, NC pins left open, no single-pin nets
  R2 no shorted power nets: rail anchors (test points) of different rails on different nets, no
     two-terminal part with both ends on one net, IC ground pins on GND and supply pins not on GND
  R3 every IC has local decoupling: for each IC supply net, at least one cap (supply -> GND) tagged as
     decoupling that IC per supply pin on that net
  R4 every INPUT pin is on a net with something that can drive it
  R5 MCU pin assignment == netlist; AF-mapped assignments are backed by an AF fact
"""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from skidl import POWER, Pin

from .pinmap import AF_FACTS, ASSIGNMENTS

T = Pin.types


@dataclass
class Finding:
    level: str
    rule: str
    msg: str


def _net(pin):
    try:
        n = pin.net
    except Exception:
        return None
    if n is None or len(n.pins) == 0:
        return None
    return n


def _group(pin):
    n = _net(pin)
    return None if n is None else frozenset(id(x) for x in n.pins)


def check(board) -> list[Finding]:
    out: list[Finding] = []
    parts = board.circuit.parts
    meta = board.meta

    # ---------------------------------------------------------------- R1
    for p in parts:
        m = meta[p.ref]["meta"]
        for pin in p.pins:
            n = _net(pin)
            if pin.func == T.NOCONNECT:
                if n is not None:
                    out.append(Finding("error", "R1", f"{p.ref}.{pin.name} is NC but connected to {n.name}"))
                continue
            if pin.name in m.optional:
                continue
            if n is None:
                out.append(Finding("error", "R1", f"{p.ref}.{pin.name} unconnected"))
    for n in board.circuit.get_nets():
        if len(n.pins) < 2:
            out.append(Finding("error", "R1", f"net {n.name} has only {len(n.pins)} pin(s)"))

    # ---------------------------------------------------------------- R2
    # connectivity groups (merged SKiDL nets share their pin list)
    rail_grp = {}
    for p in parts:
        rail = meta[p.ref]["rail"]
        if rail:
            rail_grp.setdefault(rail, set()).add(_group(p.pins[0]))
    for r, s in rail_grp.items():
        if len(s) != 1:
            out.append(Finding("error", "R2", f"rail {r} anchors on several nets"))
    inv = defaultdict(list)
    for r, s in rail_grp.items():
        inv[next(iter(s))].append(r)
    for g, rs in inv.items():
        if len(rs) > 1:
            out.append(Finding("error", "R2", f"rails {sorted(rs)} shorted together"))
    for r, s in rail_grp.items():
        n = board.nets.get(r)
        if n is None or _group(n.pins[0]) not in s:
            out.append(Finding("error", "R2", f"rail anchor {r} is not on the net named {r}"))
    for p in parts:
        if len(p.pins) == 2:
            a, b = (_net(x) for x in p.pins)
            if a is not None and b is not None and _group(p.pins[0]) == _group(p.pins[1]) \
                    and not meta[p.ref]["key"].startswith("JUMPER"):
                out.append(Finding("error", "R2", f"{p.ref} has both pins on {a.name}"))
    for p in parts:
        m = meta[p.ref]["meta"]
        if not m.is_ic:
            continue
        for pin in p.pins:
            n = _net(pin)
            if n is None:
                continue
            if pin.name in m.ground_pins and n.name != "GND":
                out.append(Finding("error", "R2", f"{p.ref}.{pin.name} ground pin on {n.name}"))
            if pin.func == T.PWRIN and pin.name not in m.ground_pins and n.name == "GND":
                out.append(Finding("error", "R2", f"{p.ref}.{pin.name} supply pin tied to GND"))

    # ---------------------------------------------------------------- R3
    caps = []
    for p in parts:
        if p.ref.startswith("C") and len(p.pins) == 2:
            a, b = (_net(x) for x in p.pins)
            if a is None or b is None:
                continue
            if b.name == "GND":
                caps.append((p.ref, a.name, meta[p.ref]["decouples"]))
            elif a.name == "GND":
                caps.append((p.ref, b.name, meta[p.ref]["decouples"]))
            else:  # e.g. LM74700 VCAP-ANODE pump cap
                caps.append((p.ref, a.name + "|" + b.name, meta[p.ref]["decouples"]))
    for p in parts:
        m = meta[p.ref]["meta"]
        if not m.is_ic:
            continue
        need = defaultdict(int)
        for pin in p.pins:
            n = _net(pin)
            if pin.func == T.PWRIN and pin.name not in m.ground_pins and n is not None:
                need[n.name] += 1
        if not need:
            out.append(Finding("error", "R3", f"{p.ref} has no supply pins?"))
        for net, k in need.items():
            have = [ref for ref, nn, dec in caps if nn == net and p.ref in dec]
            if len(have) < k:
                out.append(Finding("error", "R3", f"{p.ref}: {len(have)} decoupling cap(s) on {net}, need >= {k}"))

    # ---------------------------------------------------------------- R4
    for p in parts:
        for pin in p.pins:
            if pin.func != T.INPUT:
                continue
            n = _net(pin)
            if n is None:
                continue
            drivers = [q for q in n.pins if q is not pin and q.func != T.INPUT]
            if not drivers and getattr(n, "drive", 0) < POWER:
                out.append(Finding("error", "R4", f"{p.ref}.{pin.name} on {n.name} has no driver"))

    # ---------------------------------------------------------------- R5
    mcu = board.part("U4")
    seen = set()
    for a in ASSIGNMENTS:
        if a.pin in seen:
            out.append(Finding("error", "R5", f"MCU pin {a.pin} assigned twice"))
        seen.add(a.pin)
        n = _net(mcu[a.pin])
        if n is None or n.name != a.signal:
            out.append(Finding("error", "R5", f"MCU {a.pin} expected on {a.signal}, got {n.name if n else None}"))
        if a.af.startswith("AF") and AF_FACTS.get((a.pin, a.function)) != a.af:
            out.append(Finding("error", "R5", f"{a.pin} {a.function} {a.af} not backed by an AF fact"))
        if a.confidence != "high":
            out.append(Finding("warning", "R5", f"UNVERIFIED mapping {a.pin} = {a.function} ({a.af}): {a.note}"))
    return out
