"""Console entry points: ``armctl-ping`` and ``armctl-teleop``."""

from __future__ import annotations

import argparse
import logging
import sys
import threading

from . import protocol as P
from .calibration import Calibration, LeaderMap
from .client import ArmClient
from .mockboard import MockBoard, MockTransport
from .safety import SafetyShim
from .teleop import TeleopLoop


def _open(port: str, baud: int, **kw) -> ArmClient:
    if port == "mock":
        return ArmClient(MockTransport(MockBoard()), **kw)
    return ArmClient.open_serial(port, baud, **kw)


def ping_main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PING the arm board and print one STATE.")
    ap.add_argument("port", help="serial port (e.g. /dev/ttyACM0) or 'mock'")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("-n", "--count", type=int, default=1)
    a = ap.parse_args(argv)
    cli = _open(a.port, a.baud)
    try:
        for _ in range(a.count):
            pong = cli.ping(timeout=0.2)
            print(f"PONG fw=0x{pong.fw_version:04X} uptime={pong.uptime_ms} ms proto={pong.proto_ver}")
        st = cli.wait_for_state(timeout=1.0)
        print(f"mode={P.Mode(st.mode).name} faults={P.Fault(st.fault_flags)!r} bus={st.bus_mv} mV")
        for name, j in zip(P.JOINT_NAMES, st.joints):
            print(f"  {name:15s} {j.pos_mrad:+6d} mrad  {j.vel_mrad_s:+6d} mrad/s  {j.current_ma:5d} mA  {j.temp_c} C")
        return 0
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        cli.close()


def teleop_main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Leader-follower teleop.")
    ap.add_argument("--leader", required=True, help="leader serial port or 'mock'")
    ap.add_argument("--follower", required=True, help="follower serial port or 'mock'")
    ap.add_argument("--leader-cal", help="leader calibration YAML (incl. leader_to_follower)")
    ap.add_argument("--follower-cal", help="follower calibration YAML, pushed before start")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO if a.verbose else logging.WARNING)

    mapping = LeaderMap()
    leader = _open(a.leader, a.baud)
    follower = _open(a.follower, a.baud)
    try:
        if a.leader_cal:
            lcal = Calibration.load(a.leader_cal)
            lcal.push(leader)
            mapping = lcal.leader_to_follower or mapping
        if a.follower_cal:
            Calibration.load(a.follower_cal).push(follower)
        loop = TeleopLoop(leader, follower, mapping, SafetyShim(), rate_hz=a.rate)
        stop = threading.Event()
        print("teleop running; Ctrl-C to stop")
        try:
            loop.run(a.duration, stop)
        except KeyboardInterrupt:
            pass
        s = loop.stats
        print(f"steps={s.steps} sent={s.sent} stale={s.stale_leader} modified={s.safety_modified} "
              f"rejected={s.safety_rejected} overruns={s.overruns}")
        return 0
    finally:
        for c in (follower, leader):
            try:
                c.torque_off()
            except Exception as e:
                print(f"torque_off failed: {e}", file=sys.stderr)
            c.close()
