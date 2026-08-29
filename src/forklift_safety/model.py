from __future__ import annotations

from .types import (
    FULL_SPEED_MM_S,
    PROTOCOL_VERSION,
    REDUCED_SPEED_MM_S,
    WATCHDOG_TIMEOUT_S,
    FaultCode,
    SafetyInputs,
    SafetyOutputs,
    SafetyState,
    StatusBits,
)

LATCHED_STATES = {
    SafetyState.SAFE_STOP,
    SafetyState.PROTECTIVE_STOP,
    SafetyState.EMERGENCY_STOP,
    SafetyState.FAULT,
    SafetyState.COMMS_LOSS,
}


class SafetyPLC:
    def __init__(self, watchdog_timeout_s: float = WATCHDOG_TIMEOUT_S) -> None:
        self.watchdog_timeout_s = watchdog_timeout_s
        self.state = SafetyState.BOOT
        self.fault_code = FaultCode.NONE
        self._last_heartbeat = 0
        self._watchdog_elapsed = 0.0
        self._previous_reset = False
        self._ever_communicated = False

    def scan(self, inputs: SafetyInputs, dt: float) -> SafetyOutputs:
        dt = max(0.0, float(dt))
        if inputs.heartbeat != self._last_heartbeat:
            self._last_heartbeat = inputs.heartbeat
            self._watchdog_elapsed = 0.0
            self._ever_communicated = True
        else:
            self._watchdog_elapsed += dt

        comm_healthy = self._ever_communicated and self._watchdog_elapsed < self.watchdog_timeout_s
        protocol_ok = inputs.protocol_version == PROTOCOL_VERSION
        inputs_healthy = inputs.scanner_healthy and inputs.drive_healthy and protocol_ok
        all_clear = (
            inputs.estop_ok
            and inputs.protective_field_clear
            and inputs.scanner_healthy
            and inputs.drive_healthy
            and protocol_ok
            and inputs.simulator_ready
            and inputs.automatic_mode
            and comm_healthy
        )
        reset_edge = inputs.reset_request and not self._previous_reset
        self._previous_reset = inputs.reset_request
        reset_permitted = all_clear and inputs.drive_stopped and self.state in LATCHED_STATES

        if not inputs.estop_ok:
            self.state = SafetyState.EMERGENCY_STOP
            self.fault_code = FaultCode.ESTOP_ACTIVE
        elif not protocol_ok:
            self.state = SafetyState.FAULT
            self.fault_code = FaultCode.PROTOCOL_MISMATCH
        elif not inputs.scanner_healthy:
            self.state = SafetyState.FAULT
            self.fault_code = FaultCode.SCANNER_FAILURE
        elif not inputs.drive_healthy:
            self.state = SafetyState.FAULT
            self.fault_code = FaultCode.DRIVE_FAILURE
        elif not comm_healthy:
            self.state = SafetyState.COMMS_LOSS
            self.fault_code = FaultCode.COMMUNICATION_TIMEOUT
        elif not inputs.protective_field_clear:
            self.state = SafetyState.PROTECTIVE_STOP
            self.fault_code = FaultCode.PROTECTIVE_FIELD
        elif self.state == SafetyState.BOOT:
            self.state = SafetyState.SAFE_STOP
            self.fault_code = FaultCode.NONE
        elif self.state in LATCHED_STATES:
            if reset_edge and reset_permitted:
                self.state = (
                    SafetyState.RUN_FULL if inputs.warning_field_clear else SafetyState.RUN_REDUCED
                )
                self.fault_code = FaultCode.NONE
        elif not inputs.automatic_mode:
            self.state = SafetyState.SAFE_STOP
            self.fault_code = FaultCode.NONE
        elif not inputs.warning_field_clear:
            self.state = SafetyState.RUN_REDUCED
            self.fault_code = FaultCode.NONE
        else:
            self.state = SafetyState.RUN_FULL
            self.fault_code = FaultCode.NONE

        drive_enable = self.state in {SafetyState.RUN_FULL, SafetyState.RUN_REDUCED}
        speed_limit = 0
        if self.state == SafetyState.RUN_FULL:
            speed_limit = min(inputs.commanded_speed_mm_s, FULL_SPEED_MM_S)
        elif self.state == SafetyState.RUN_REDUCED:
            speed_limit = min(inputs.commanded_speed_mm_s, REDUCED_SPEED_MM_S)

        status = StatusBits(0)
        if comm_healthy:
            status |= StatusBits.COMM_HEALTHY
        if inputs_healthy:
            status |= StatusBits.INPUTS_HEALTHY
        if self.state in LATCHED_STATES:
            status |= StatusBits.RESET_REQUIRED
        if not inputs.warning_field_clear:
            status |= StatusBits.WARNING_ACTIVE
        if not inputs.protective_field_clear:
            status |= StatusBits.PROTECTIVE_ACTIVE
        if not inputs.estop_ok:
            status |= StatusBits.ESTOP_ACTIVE
        if self.fault_code != FaultCode.NONE:
            status |= StatusBits.FAULT_LATCHED

        return SafetyOutputs(
            drive_enable=drive_enable,
            safe_stop_request=not drive_enable,
            speed_limit_mm_s=speed_limit,
            safety_state=self.state,
            fault_code=self.fault_code,
            heartbeat_ack=inputs.heartbeat,
            reset_permitted=reset_permitted,
            status_word=int(status),
        )
