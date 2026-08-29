from __future__ import annotations

import argparse
import logging
from pathlib import Path

from .modbus import EmulatorConfig, ModbusSafetyClient, PLCEmulator
from .scenarios import Scenario, bundled_scenario
from .simulator import ForkliftSimulation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Autonomous forklift safety simulation")
    parser.add_argument("--plc", choices=("emulated", "external"), default="emulated")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--scenario", default=None, help="bundled name or path to JSON scenario")
    parser.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    parser.add_argument("--headless", action="store_true", help="use PyBullet DIRECT mode")
    parser.add_argument("--record-gif", help="save an actual simulation GIF")
    parser.add_argument("--record-fps", type=int, default=10)
    parser.add_argument(
        "--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR")
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    scenario = None
    if args.scenario:
        candidate = Path(args.scenario)
        path = candidate if candidate.exists() else bundled_scenario(args.scenario)
        scenario = Scenario.load(path)

    emulator = None
    port = args.port
    if args.plc == "emulated":
        port = port or 1502
        emulator = PLCEmulator(EmulatorConfig(host=args.host, port=port, device_id=args.device_id))
        emulator.start()
    else:
        port = port or 502

    try:
        client = ModbusSafetyClient(args.host, port, args.device_id)
        simulation = ForkliftSimulation(
            client,
            gui=not args.headless,
            scenario=scenario,
            duration_s=args.duration,
            record_gif=args.record_gif,
            record_fps=args.record_fps,
        )
        simulation.run()
    except KeyboardInterrupt:
        return 130
    finally:
        if emulator:
            emulator.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
