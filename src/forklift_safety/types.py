from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag

PROTOCOL_VERSION = 0x0100
FULL_SPEED_MM_S = 1800
REDUCED_SPEED_MM_S = 400
WATCHDOG_TIMEOUT_S = 0.5


class SafetyState(IntEnum):
    BOOT = 0
    SAFE_STOP = 10
    RUN_FULL = 20
    RUN_REDUCED = 30
    PROTECTIVE_STOP = 40
    EMERGENCY_STOP = 50
    FAULT = 60
    COMMS_LOSS = 70


class FaultCode(IntEnum):
    NONE = 0
    ESTOP_ACTIVE = 10
    PROTECTIVE_FIELD = 20
    SCANNER_FAILURE = 30
    DRIVE_FAILURE = 40
    COMMUNICATION_TIMEOUT = 50
    PROTOCOL_MISMATCH = 60


class StatusBits(IntFlag):
    COMM_HEALTHY = 1 << 0
    INPUTS_HEALTHY = 1 << 1
    RESET_REQUIRED = 1 << 2
    WARNING_ACTIVE = 1 << 3
    PROTECTIVE_ACTIVE = 1 << 4
    ESTOP_ACTIVE = 1 << 5
    FAULT_LATCHED = 1 << 6


@dataclass(slots=True)
class SafetyInputs:
    protocol_version: int = PROTOCOL_VERSION
    heartbeat: int = 0
    commanded_speed_mm_s: int = FULL_SPEED_MM_S
    automatic_mode: bool = True
    reset_request: bool = False
    estop_ok: bool = True
    warning_field_clear: bool = True
    protective_field_clear: bool = True
    scanner_healthy: bool = True
    drive_healthy: bool = True
    drive_stopped: bool = True
    simulator_ready: bool = True
    fault_injection_mask: int = 0
    nearest_object_mm: int = 0xFFFF


@dataclass(slots=True)
class SafetyOutputs:
    drive_enable: bool = False
    safe_stop_request: bool = True
    speed_limit_mm_s: int = 0
    safety_state: SafetyState = SafetyState.BOOT
    fault_code: FaultCode = FaultCode.NONE
    heartbeat_ack: int = 0
    reset_permitted: bool = False
    status_word: int = int(StatusBits.RESET_REQUIRED)
    protocol_version: int = PROTOCOL_VERSION
