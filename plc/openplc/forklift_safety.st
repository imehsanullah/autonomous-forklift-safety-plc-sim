FUNCTION_BLOCK FB_SafetyKernel
VAR_INPUT
    ProtocolVersion        : UINT;
    Heartbeat              : UINT;
    CommandedSpeed         : UINT;
    AutomaticMode          : BOOL;
    ResetRequest           : BOOL;
    EStopOK                : BOOL;
    WarningFieldClear      : BOOL;
    ProtectiveFieldClear   : BOOL;
    ScannerHealthy         : BOOL;
    DriveHealthy           : BOOL;
    DriveStopped           : BOOL;
    SimulatorReady         : BOOL;
END_VAR
VAR_OUTPUT
    DriveEnable            : BOOL;
    SafeStopRequest        : BOOL;
    SpeedLimit             : UINT;
    SafetyState            : UINT;
    FaultCode              : UINT;
    HeartbeatAck           : UINT;
    ResetPermitted         : BOOL;
    StatusWord             : UINT;
    OutputProtocolVersion  : UINT;
END_VAR
VAR
    State                  : UINT := 0;
    LatchedFault           : UINT := 0;
    LastHeartbeat          : UINT := 0;
    HasCommunicated        : BOOL := FALSE;
    HeartbeatChanged       : BOOL;
    CommunicationHealthy   : BOOL;
    InputsHealthy          : BOOL;
    AllClear               : BOOL;
    ResetEdge              : R_TRIG;
    HeartbeatWatchdog      : TON;
END_VAR
VAR CONSTANT
    PROTOCOL_V1            : UINT := 256;
    FULL_SPEED             : UINT := 1800;
    REDUCED_SPEED          : UINT := 400;

    STATE_BOOT             : UINT := 0;
    STATE_SAFE_STOP        : UINT := 10;
    STATE_RUN_FULL         : UINT := 20;
    STATE_RUN_REDUCED      : UINT := 30;
    STATE_PROTECTIVE_STOP  : UINT := 40;
    STATE_EMERGENCY_STOP   : UINT := 50;
    STATE_FAULT            : UINT := 60;
    STATE_COMMS_LOSS       : UINT := 70;

    FAULT_NONE             : UINT := 0;
    FAULT_ESTOP            : UINT := 10;
    FAULT_PROTECTIVE       : UINT := 20;
    FAULT_SCANNER          : UINT := 30;
    FAULT_DRIVE            : UINT := 40;
    FAULT_COMMS            : UINT := 50;
    FAULT_PROTOCOL         : UINT := 60;
END_VAR

(* A changing counter is proof of fresh simulator input. *)
HeartbeatChanged := Heartbeat <> LastHeartbeat;
IF HeartbeatChanged THEN
    LastHeartbeat := Heartbeat;
    HasCommunicated := TRUE;
END_IF;
HeartbeatWatchdog(
    IN := HasCommunicated AND NOT HeartbeatChanged,
    PT := T#500ms
);
CommunicationHealthy := HasCommunicated AND NOT HeartbeatWatchdog.Q;
InputsHealthy := ScannerHealthy AND DriveHealthy AND (ProtocolVersion = PROTOCOL_V1);
AllClear := EStopOK
    AND ProtectiveFieldClear
    AND ScannerHealthy
    AND DriveHealthy
    AND (ProtocolVersion = PROTOCOL_V1)
    AND SimulatorReady
    AND AutomaticMode
    AND CommunicationHealthy;

ResetEdge(CLK := ResetRequest);
ResetPermitted := AllClear
    AND DriveStopped
    AND ((State = STATE_SAFE_STOP)
      OR (State = STATE_PROTECTIVE_STOP)
      OR (State = STATE_EMERGENCY_STOP)
      OR (State = STATE_FAULT)
      OR (State = STATE_COMMS_LOSS));

(* Globally prioritized trips override every normal state transition. *)
IF NOT EStopOK THEN
    State := STATE_EMERGENCY_STOP;
    LatchedFault := FAULT_ESTOP;
ELSIF ProtocolVersion <> PROTOCOL_V1 THEN
    State := STATE_FAULT;
    LatchedFault := FAULT_PROTOCOL;
ELSIF NOT ScannerHealthy THEN
    State := STATE_FAULT;
    LatchedFault := FAULT_SCANNER;
ELSIF NOT DriveHealthy THEN
    State := STATE_FAULT;
    LatchedFault := FAULT_DRIVE;
ELSIF NOT CommunicationHealthy THEN
    State := STATE_COMMS_LOSS;
    LatchedFault := FAULT_COMMS;
ELSIF NOT ProtectiveFieldClear THEN
    State := STATE_PROTECTIVE_STOP;
    LatchedFault := FAULT_PROTECTIVE;
ELSIF State = STATE_BOOT THEN
    State := STATE_SAFE_STOP;
    LatchedFault := FAULT_NONE;
ELSIF (State = STATE_SAFE_STOP)
   OR (State = STATE_PROTECTIVE_STOP)
   OR (State = STATE_EMERGENCY_STOP)
   OR (State = STATE_FAULT)
   OR (State = STATE_COMMS_LOSS) THEN
    IF ResetEdge.Q AND ResetPermitted THEN
        IF WarningFieldClear THEN
            State := STATE_RUN_FULL;
        ELSE
            State := STATE_RUN_REDUCED;
        END_IF;
        LatchedFault := FAULT_NONE;
    END_IF;
ELSIF NOT AutomaticMode THEN
    State := STATE_SAFE_STOP;
    LatchedFault := FAULT_NONE;
ELSIF NOT WarningFieldClear THEN
    State := STATE_RUN_REDUCED;
    LatchedFault := FAULT_NONE;
ELSE
    State := STATE_RUN_FULL;
    LatchedFault := FAULT_NONE;
END_IF;

(* Fail-safe output defaults, overwritten only by RUN states. *)
DriveEnable := FALSE;
SafeStopRequest := TRUE;
SpeedLimit := 0;
IF State = STATE_RUN_FULL THEN
    DriveEnable := TRUE;
    SafeStopRequest := FALSE;
    SpeedLimit := MIN(CommandedSpeed, FULL_SPEED);
ELSIF State = STATE_RUN_REDUCED THEN
    DriveEnable := TRUE;
    SafeStopRequest := FALSE;
    SpeedLimit := MIN(CommandedSpeed, REDUCED_SPEED);
END_IF;

StatusWord := 0;
IF CommunicationHealthy THEN StatusWord := StatusWord + 1; END_IF;
IF InputsHealthy THEN StatusWord := StatusWord + 2; END_IF;
IF (State = STATE_SAFE_STOP) OR (State = STATE_PROTECTIVE_STOP)
   OR (State = STATE_EMERGENCY_STOP) OR (State = STATE_FAULT)
   OR (State = STATE_COMMS_LOSS) THEN
    StatusWord := StatusWord + 4;
END_IF;
IF NOT WarningFieldClear THEN StatusWord := StatusWord + 8; END_IF;
IF NOT ProtectiveFieldClear THEN StatusWord := StatusWord + 16; END_IF;
IF NOT EStopOK THEN StatusWord := StatusWord + 32; END_IF;
IF LatchedFault <> FAULT_NONE THEN StatusWord := StatusWord + 64; END_IF;

SafetyState := State;
FaultCode := LatchedFault;
HeartbeatAck := Heartbeat;
OutputProtocolVersion := PROTOCOL_V1;
END_FUNCTION_BLOCK


PROGRAM main
VAR
    (* Simulator inputs: %MW0..%MW15 / holding registers 1024..1039 *)
    InProtocolVersion      AT %MW0   : UINT;
    InHeartbeat            AT %MW1   : UINT;
    InCommandedSpeed       AT %MW2   : UINT;
    InAutomaticMode        AT %MW3   : UINT;
    InResetRequest         AT %MW4   : UINT;
    InEStopOK              AT %MW5   : UINT;
    InWarningFieldClear    AT %MW6   : UINT;
    InProtectiveFieldClear AT %MW7   : UINT;
    InScannerHealthy       AT %MW8   : UINT;
    InDriveHealthy         AT %MW9   : UINT;
    InDriveStopped         AT %MW10  : UINT;
    InSimulatorReady       AT %MW11  : UINT;
    InFaultInjectionMask   AT %MW12  : UINT; (* diagnostic only *)
    InNearestObjectMM      AT %MW13  : UINT; (* diagnostic only *)

    (* PLC outputs: %MW100..%MW111 / holding registers 1124..1135 *)
    OutDriveEnable         AT %MW100 : UINT;
    OutSafeStopRequest     AT %MW101 : UINT;
    OutSpeedLimit          AT %MW102 : UINT;
    OutSafetyState         AT %MW103 : UINT;
    OutFaultCode           AT %MW104 : UINT;
    OutHeartbeatAck        AT %MW105 : UINT;
    OutResetPermitted      AT %MW106 : UINT;
    OutStatusWord          AT %MW107 : UINT;
    OutProtocolVersion     AT %MW108 : UINT;

    Safety : FB_SafetyKernel;
END_VAR

Safety(
    ProtocolVersion := InProtocolVersion,
    Heartbeat := InHeartbeat,
    CommandedSpeed := InCommandedSpeed,
    AutomaticMode := InAutomaticMode <> 0,
    ResetRequest := InResetRequest <> 0,
    EStopOK := InEStopOK <> 0,
    WarningFieldClear := InWarningFieldClear <> 0,
    ProtectiveFieldClear := InProtectiveFieldClear <> 0,
    ScannerHealthy := InScannerHealthy <> 0,
    DriveHealthy := InDriveHealthy <> 0,
    DriveStopped := InDriveStopped <> 0,
    SimulatorReady := InSimulatorReady <> 0
);

IF Safety.DriveEnable THEN OutDriveEnable := 1; ELSE OutDriveEnable := 0; END_IF;
IF Safety.SafeStopRequest THEN OutSafeStopRequest := 1; ELSE OutSafeStopRequest := 0; END_IF;
OutSpeedLimit := Safety.SpeedLimit;
OutSafetyState := Safety.SafetyState;
OutFaultCode := Safety.FaultCode;
OutHeartbeatAck := Safety.HeartbeatAck;
IF Safety.ResetPermitted THEN OutResetPermitted := 1; ELSE OutResetPermitted := 0; END_IF;
OutStatusWord := Safety.StatusWord;
OutProtocolVersion := Safety.OutputProtocolVersion;
END_PROGRAM


CONFIGURATION Config0
    RESOURCE Res0 ON PLC
        TASK MainTask(INTERVAL := T#20ms, PRIORITY := 0);
        PROGRAM MainInstance WITH MainTask : main;
    END_RESOURCE
END_CONFIGURATION
