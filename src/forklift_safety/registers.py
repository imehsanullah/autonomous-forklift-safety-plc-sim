from __future__ import annotations

from dataclasses import dataclass

from .types import FaultCode, SafetyInputs, SafetyOutputs, SafetyState


@dataclass(frozen=True, slots=True)
class RegisterMap:
    base_address: int = 1024
    input_offset: int = 0
    input_count: int = 16
    output_offset: int = 100
    output_count: int = 12

    @property
    def input_address(self) -> int:
        return self.base_address + self.input_offset

    @property
    def output_address(self) -> int:
        return self.base_address + self.output_offset


DEFAULT_REGISTER_MAP = RegisterMap()


def _bool(value: int) -> bool:
    return value != 0


def _u16(value: int) -> int:
    return max(0, min(0xFFFF, int(value)))


def encode_inputs(inputs: SafetyInputs) -> list[int]:
    values = [0] * DEFAULT_REGISTER_MAP.input_count
    values[0] = _u16(inputs.protocol_version)
    values[1] = _u16(inputs.heartbeat)
    values[2] = _u16(inputs.commanded_speed_mm_s)
    values[3] = int(inputs.automatic_mode)
    values[4] = int(inputs.reset_request)
    values[5] = int(inputs.estop_ok)
    values[6] = int(inputs.warning_field_clear)
    values[7] = int(inputs.protective_field_clear)
    values[8] = int(inputs.scanner_healthy)
    values[9] = int(inputs.drive_healthy)
    values[10] = int(inputs.drive_stopped)
    values[11] = int(inputs.simulator_ready)
    values[12] = _u16(inputs.fault_injection_mask)
    values[13] = _u16(inputs.nearest_object_mm)
    return values


def decode_inputs(values: list[int]) -> SafetyInputs:
    if len(values) < DEFAULT_REGISTER_MAP.input_count:
        raise ValueError("incomplete Modbus input image")
    return SafetyInputs(
        protocol_version=values[0],
        heartbeat=values[1],
        commanded_speed_mm_s=values[2],
        automatic_mode=_bool(values[3]),
        reset_request=_bool(values[4]),
        estop_ok=_bool(values[5]),
        warning_field_clear=_bool(values[6]),
        protective_field_clear=_bool(values[7]),
        scanner_healthy=_bool(values[8]),
        drive_healthy=_bool(values[9]),
        drive_stopped=_bool(values[10]),
        simulator_ready=_bool(values[11]),
        fault_injection_mask=values[12],
        nearest_object_mm=values[13],
    )


def encode_outputs(outputs: SafetyOutputs) -> list[int]:
    values = [0] * DEFAULT_REGISTER_MAP.output_count
    values[0] = int(outputs.drive_enable)
    values[1] = int(outputs.safe_stop_request)
    values[2] = _u16(outputs.speed_limit_mm_s)
    values[3] = int(outputs.safety_state)
    values[4] = int(outputs.fault_code)
    values[5] = _u16(outputs.heartbeat_ack)
    values[6] = int(outputs.reset_permitted)
    values[7] = _u16(outputs.status_word)
    values[8] = _u16(outputs.protocol_version)
    return values


def decode_outputs(values: list[int]) -> SafetyOutputs:
    if len(values) < DEFAULT_REGISTER_MAP.output_count:
        raise ValueError("incomplete Modbus output image")
    try:
        state = SafetyState(values[3])
    except ValueError:
        state = SafetyState.FAULT
    try:
        fault = FaultCode(values[4])
    except ValueError:
        fault = FaultCode.PROTOCOL_MISMATCH
    return SafetyOutputs(
        drive_enable=_bool(values[0]),
        safe_stop_request=_bool(values[1]),
        speed_limit_mm_s=values[2],
        safety_state=state,
        fault_code=fault,
        heartbeat_ack=values[5],
        reset_permitted=_bool(values[6]),
        status_word=values[7],
        protocol_version=values[8],
    )
