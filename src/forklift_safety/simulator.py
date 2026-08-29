from __future__ import annotations

import importlib
import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path

from .modbus import ModbusSafetyClient
from .scenarios import Scenario, ScenarioEvent
from .types import FaultCode, SafetyInputs, SafetyOutputs, SafetyState

LOG = logging.getLogger(__name__)


@dataclass(slots=True)
class InjectedFaults:
    pedestrian_zone: str = "clear"
    estop: bool = False
    scanner_failure: bool = False
    drive_failure: bool = False
    communication_loss: bool = False
    reset_pulse: bool = False
    paused: bool = False


@dataclass(slots=True)
class PedestrianPath:
    start: tuple[float, float]
    control_1: tuple[float, float]
    control_2: tuple[float, float]
    end: tuple[float, float]
    duration_s: float
    progress: float = 0.0


class ForkliftSimulation:
    physics_hz = 60.0
    io_hz = 20.0
    warning_length_m = 4.5
    warning_width_m = 3.0
    protective_length_m = 2.2
    protective_width_m = 2.0
    scanner_origin_offset_m = 2.35

    def __init__(
        self,
        client: ModbusSafetyClient,
        *,
        gui: bool = True,
        scenario: Scenario | None = None,
        duration_s: float | None = None,
        record_gif: str | Path | None = None,
        record_fps: int = 10,
    ) -> None:
        try:
            self.p = importlib.import_module("pybullet")
        except ImportError as exc:
            raise RuntimeError(
                "PyBullet is not installed. Run `uv sync --extra sim`, or see the "
                "Apple Silicon note in docs/getting-started.md."
            ) from exc
        self.client = client
        self.gui = gui
        self.scenario = scenario
        self.duration_s = (
            duration_s if duration_s is not None else (scenario.duration_s if scenario else None)
        )
        self.faults = InjectedFaults()
        self.inputs = SafetyInputs()
        self.outputs = SafetyOutputs()
        self.speed_m_s = 0.0
        self.commanded_speed_m_s = 1.0
        self.heartbeat = 0
        self.forklift_x = -8.0
        self.forklift_y = 0.0
        self.route_end_x = 4.8
        self.route_complete = False
        self.pedestrian_position = [0.0, 3.4, 0.95]
        self.pedestrian_heading = -math.pi / 2
        self.pedestrian_walk_speed_m_s = 1.15
        self.pedestrian_path: PedestrianPath | None = None
        self.pedestrian_walk_phase = 0.0
        self._pedestrian_curve_sign = 1.0
        self._last_io = 0.0
        self._last_state: tuple[SafetyState, FaultCode] | None = None
        self._status_item = -1
        self._help_item = -1
        self._bodies: dict[str, int] = {}
        self.record_gif = Path(record_gif) if record_gif else None
        self.record_fps = max(1, record_fps)
        self._record_accumulator = 0.0
        self._recorded_frames: list[object] = []

    def run(self) -> None:
        mode = self.p.GUI if self.gui else self.p.DIRECT
        connection = self.p.connect(mode)
        if connection < 0:
            raise RuntimeError("could not connect to PyBullet")
        if not self.client.connect():
            self.p.disconnect()
            raise ConnectionError("could not connect to PLC Modbus TCP server")
        try:
            self._build_world()
            previous = time.monotonic()
            elapsed = 0.0
            while self.p.isConnected():
                frame_started = time.monotonic()
                dt = min(0.1, frame_started - previous)
                previous = frame_started
                elapsed += dt
                self._handle_keyboard()
                self._apply_scenario(elapsed)
                self._update_pedestrian(dt)
                self._update_plant(dt)
                if frame_started - self._last_io >= 1.0 / self.io_hz:
                    self._exchange_with_plc()
                    self._last_io = frame_started
                self._update_visuals()
                self.p.stepSimulation()
                self._capture_gif_frame(dt)
                if self.duration_s is not None and elapsed >= self.duration_s:
                    break
                time.sleep(max(0.0, 1.0 / self.physics_hz - (time.monotonic() - frame_started)))
        finally:
            self._save_gif()
            self.client.close()
            if self.p.isConnected():
                self.p.disconnect()

    def _build_world(self) -> None:
        p = self.p
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / self.physics_hz)
        pybullet_data = importlib.import_module("pybullet_data")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(15, 55, -35, [0, 0, 0])

        self._box("floor", [12, 7, 0.05], [0, 0, -0.05], [0.18, 0.20, 0.22, 1])
        for x in (-5.5, 0.0, 5.5):
            for y in (-4.7, 4.7):
                self._rack([x, y, 1.3])

        self._box("forklift", [0.9, 0.65, 0.45], [self.forklift_x, 0, 0.45], [1, 0.55, 0, 1])
        self._box("mast", [0.10, 0.7, 1.1], [self.forklift_x + 0.75, 0, 1.1], [0.12, 0.12, 0.14, 1])
        self._box(
            "fork_l", [0.8, 0.08, 0.05], [self.forklift_x + 1.45, -0.35, 0.12], [0.2, 0.2, 0.22, 1]
        )
        self._box(
            "fork_r", [0.8, 0.08, 0.05], [self.forklift_x + 1.45, 0.35, 0.12], [0.2, 0.2, 0.22, 1]
        )
        self._load_pedestrian()

        self._box(
            "warning_zone",
            [self.warning_length_m / 2, self.warning_width_m / 2, 0.015],
            [
                self.forklift_x + self.scanner_origin_offset_m + self.warning_length_m / 2,
                0,
                0.025,
            ],
            [1.0, 0.65, 0.0, 0.25],
            collision=False,
        )
        self._box(
            "protective_zone",
            [self.protective_length_m / 2, self.protective_width_m / 2, 0.02],
            [
                self.forklift_x + self.scanner_origin_offset_m + self.protective_length_m / 2,
                0,
                0.055,
            ],
            [1.0, 0.05, 0.05, 0.33],
            collision=False,
        )
        self._help_item = p.addUserDebugText(
            "R reset | 1 warning | 2 protective | E E-stop | "
            "S scanner | D drive | C comms | Q quit",
            [-7.5, -5.8, 3.4],
            textColorRGB=[0.9, 0.9, 0.9],
            textSize=1.1,
        )

    def _box(
        self,
        name: str,
        half_extents: list[float],
        position: list[float],
        color: list[float],
        *,
        collision: bool = True,
    ) -> int:
        shape = self.p.createVisualShape(self.p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        collision_shape = (
            self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=half_extents)
            if collision
            else -1
        )
        body = self.p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=shape,
            basePosition=position,
        )
        self._bodies[name] = body
        return body

    def _load_pedestrian(self) -> None:
        position, orientation = self._pedestrian_pose()
        body = self.p.loadURDF(
            "humanoid/humanoid.urdf",
            position,
            orientation,
            useFixedBase=True,
            globalScaling=0.27,
        )
        self._bodies["pedestrian"] = body

        skin = [0.76, 0.53, 0.36, 1]
        high_visibility = [1.0, 0.35, 0.02, 1]
        trousers = [0.05, 0.12, 0.22, 1]
        boots = [0.04, 0.04, 0.05, 1]
        for link in (0, 1, 3, 6):
            self.p.changeVisualShape(body, link, rgbaColor=high_visibility)
        for link in (2, 4, 5, 7, 8):
            self.p.changeVisualShape(body, link, rgbaColor=skin)
        for link in (9, 10, 12, 13):
            self.p.changeVisualShape(body, link, rgbaColor=trousers)
        for link in (11, 14):
            self.p.changeVisualShape(body, link, rgbaColor=boots)

    def _rack(self, position: list[float]) -> None:
        x, y, z = position
        color = [0.15, 0.3, 0.5, 1]
        self._box(f"rack_{x}_{y}_post1", [0.08, 0.7, 1.3], [x - 1.2, y, z], color)
        self._box(f"rack_{x}_{y}_post2", [0.08, 0.7, 1.3], [x + 1.2, y, z], color)
        for dz in (-0.9, 0.0, 0.9):
            self._box(f"rack_{x}_{y}_{dz}", [1.3, 0.7, 0.06], [x, y, z + dz], color)

    def _handle_keyboard(self) -> None:
        if not self.gui:
            return
        keys = self.p.getKeyboardEvents()
        triggered = self.p.KEY_WAS_TRIGGERED
        if keys.get(ord("q"), 0) & triggered:
            self.p.disconnect()
        if keys.get(ord("r"), 0) & triggered:
            self.faults.reset_pulse = True
        if keys.get(ord("1"), 0) & triggered:
            self.faults.pedestrian_zone = (
                "clear" if self.faults.pedestrian_zone == "warning" else "warning"
            )
            self._begin_pedestrian_walk(self.faults.pedestrian_zone)
        if keys.get(ord("2"), 0) & triggered:
            self.faults.pedestrian_zone = (
                "clear" if self.faults.pedestrian_zone == "protective" else "protective"
            )
            self._begin_pedestrian_walk(self.faults.pedestrian_zone)
        for key, attr in (
            ("e", "estop"),
            ("s", "scanner_failure"),
            ("d", "drive_failure"),
            ("c", "communication_loss"),
            (" ", "paused"),
        ):
            if keys.get(ord(key), 0) & triggered:
                setattr(self.faults, attr, not getattr(self.faults, attr))

    def _apply_scenario(self, elapsed_s: float) -> None:
        if not self.scenario:
            return
        for event in self.scenario.due(elapsed_s):
            self.apply_event(event)

    def apply_event(self, event: ScenarioEvent) -> None:
        if event.action == "pedestrian_zone":
            if event.value not in {"clear", "warning", "protective"}:
                raise ValueError(f"invalid pedestrian zone: {event.value}")
            self.faults.pedestrian_zone = event.value
            self._begin_pedestrian_walk(event.value)
        elif event.action == "reset":
            self.faults.reset_pulse = bool(event.value)
        elif hasattr(self.faults, event.action):
            setattr(self.faults, event.action, bool(event.value))
        else:
            raise ValueError(f"unsupported scenario action: {event.action}")
        LOG.info("scenario %.1fs: %s=%s %s", event.at_s, event.action, event.value, event.note)

    def _update_plant(self, dt: float) -> None:
        target = 0.0
        if (
            self.outputs.drive_enable
            and not self.outputs.safe_stop_request
            and not self.faults.paused
        ):
            target = min(self.commanded_speed_m_s, self.outputs.speed_limit_mm_s / 1000.0)
        rate = 4.0 if target < self.speed_m_s else 1.2
        delta = max(-rate * dt, min(rate * dt, target - self.speed_m_s))
        self.speed_m_s += delta
        self.forklift_x += self.speed_m_s * dt
        if self.forklift_x >= self.route_end_x:
            self.forklift_x = self.route_end_x
            self.speed_m_s = 0.0
            self.commanded_speed_m_s = 0.0
            self.route_complete = True

    def _exchange_with_plc(self) -> None:
        warning_clear, protective_clear, nearest = self._scanner_reading()
        self.heartbeat = (self.heartbeat + 1) & 0xFFFF
        self.inputs = SafetyInputs(
            heartbeat=self.heartbeat,
            commanded_speed_mm_s=round(self.commanded_speed_m_s * 1000),
            automatic_mode=not self.faults.paused,
            reset_request=self.faults.reset_pulse,
            estop_ok=not self.faults.estop,
            warning_field_clear=warning_clear,
            protective_field_clear=protective_clear,
            scanner_healthy=not self.faults.scanner_failure,
            drive_healthy=not self.faults.drive_failure,
            drive_stopped=abs(self.speed_m_s) < 0.02,
            simulator_ready=True,
            fault_injection_mask=self._fault_mask(),
            nearest_object_mm=nearest,
        )
        try:
            self.outputs = self.client.exchange(
                self.inputs,
                write_inputs=not self.faults.communication_loss,
            )
        except (ConnectionError, OSError) as exc:
            LOG.error("PLC output channel unavailable: %s", exc)
            self.outputs = SafetyOutputs(
                safety_state=SafetyState.COMMS_LOSS,
                fault_code=FaultCode.COMMUNICATION_TIMEOUT,
            )
        finally:
            self.faults.reset_pulse = False
        current = (self.outputs.safety_state, self.outputs.fault_code)
        if current != self._last_state:
            LOG.info(
                "PLC state=%s fault=%s enable=%s limit=%d mm/s",
                self.outputs.safety_state.name,
                self.outputs.fault_code.name,
                self.outputs.drive_enable,
                self.outputs.speed_limit_mm_s,
            )
            self._last_state = current

    def _fault_mask(self) -> int:
        return (
            int(self.faults.estop)
            | int(self.faults.scanner_failure) << 1
            | int(self.faults.drive_failure) << 2
            | int(self.faults.communication_loss) << 3
        )

    def _pedestrian_target(self, zone: str) -> tuple[float, float]:
        scanner_x = self.forklift_x + self.scanner_origin_offset_m
        return {
            "clear": (min(self.pedestrian_position[0] + 1.0, 10.5), 3.4),
            "warning": (min(scanner_x + 3.5, 10.5), 1.25),
            "protective": (min(scanner_x + 1.8, 9.5), 0.25),
        }[zone]

    def _begin_pedestrian_walk(self, zone: str) -> None:
        start = (self.pedestrian_position[0], self.pedestrian_position[1])
        end = self._pedestrian_target(zone)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        direct_distance = math.hypot(dx, dy)
        if direct_distance < 0.05:
            self.pedestrian_path = None
            return

        normal_x = -dy / direct_distance
        normal_y = dx / direct_distance
        bend = min(0.9, direct_distance * 0.24) * self._pedestrian_curve_sign
        self._pedestrian_curve_sign *= -1
        tangent_length = min(1.0, direct_distance * 0.35)
        control_1 = (
            max(
                -10.5,
                min(10.5, start[0] + math.cos(self.pedestrian_heading) * tangent_length),
            ),
            max(
                -5.5,
                min(5.5, start[1] + math.sin(self.pedestrian_heading) * tangent_length),
            ),
        )
        control_2 = (
            max(
                -10.5,
                min(10.5, end[0] - dx / direct_distance * tangent_length * 0.65 + normal_x * bend),
            ),
            max(
                -5.5,
                min(5.5, end[1] - dy / direct_distance * tangent_length * 0.65 + normal_y * bend),
            ),
        )
        if zone == "protective":
            side = 1.0 if start[1] >= 0 else -1.0
            safe_front_x = self.forklift_x + 3.0
            control_1 = (max(control_1[0], safe_front_x), start[1])
            control_2 = (max(control_2[0], safe_front_x + 0.25), side * 1.15)
        estimated_length = direct_distance + abs(bend) * 0.55
        self.pedestrian_path = PedestrianPath(
            start=start,
            control_1=control_1,
            control_2=control_2,
            end=end,
            duration_s=max(0.1, estimated_length / self.pedestrian_walk_speed_m_s),
        )

    @staticmethod
    def _bezier_point(path: PedestrianPath, progress: float) -> tuple[float, float]:
        eased_progress = progress * progress * (3.0 - 2.0 * progress)
        inverse = 1.0 - eased_progress
        x = (
            inverse**3 * path.start[0]
            + 3 * inverse**2 * eased_progress * path.control_1[0]
            + 3 * inverse * eased_progress**2 * path.control_2[0]
            + eased_progress**3 * path.end[0]
        )
        y = (
            inverse**3 * path.start[1]
            + 3 * inverse**2 * eased_progress * path.control_1[1]
            + 3 * inverse * eased_progress**2 * path.control_2[1]
            + eased_progress**3 * path.end[1]
        )
        return x, y

    def _update_pedestrian(self, dt: float) -> None:
        path = self.pedestrian_path
        if path is None:
            return
        previous_x, previous_y = self.pedestrian_position[:2]
        path.progress = min(1.0, path.progress + max(0.0, dt) / path.duration_s)
        next_x, next_y = self._bezier_point(path, path.progress)
        self.pedestrian_position[0] = next_x
        self.pedestrian_position[1] = next_y

        movement_x = next_x - previous_x
        movement_y = next_y - previous_y
        if math.hypot(movement_x, movement_y) > 1e-6:
            desired_heading = math.atan2(movement_y, movement_x)
            heading_error = (desired_heading - self.pedestrian_heading + math.pi) % (
                2 * math.pi
            ) - math.pi
            max_turn = 3.0 * max(0.0, dt)
            self.pedestrian_heading += max(-max_turn, min(max_turn, heading_error))
            self.pedestrian_walk_phase += max(0.0, dt) * math.tau * 1.6
        if path.progress >= 1.0:
            self.pedestrian_path = None

    def _pedestrian_pose(self) -> tuple[list[float], list[float]]:
        orientation = self.p.getQuaternionFromEuler(
            [math.pi / 2, 0, self.pedestrian_heading + math.pi / 2]
        )
        walking = self.pedestrian_path is not None
        bob = 0.015 * abs(math.sin(self.pedestrian_walk_phase)) if walking else 0.0
        position = [
            self.pedestrian_position[0],
            self.pedestrian_position[1],
            self.pedestrian_position[2] + bob,
        ]
        return position, orientation

    def _animate_pedestrian_gait(self) -> None:
        body = self._bodies["pedestrian"]
        walking = self.pedestrian_path is not None
        swing = math.sin(self.pedestrian_walk_phase) if walking else 0.0
        opposite = -swing
        hip_angle = 0.45 * swing
        shoulder_angle = 0.38 * opposite
        self.p.resetJointStateMultiDof(body, 9, self.p.getQuaternionFromEuler([hip_angle, 0, 0]))
        self.p.resetJointStateMultiDof(body, 12, self.p.getQuaternionFromEuler([-hip_angle, 0, 0]))
        self.p.resetJointStateMultiDof(
            body, 3, self.p.getQuaternionFromEuler([shoulder_angle, 0, 0])
        )
        self.p.resetJointStateMultiDof(
            body, 6, self.p.getQuaternionFromEuler([-shoulder_angle, 0, 0])
        )
        self.p.resetJointState(body, 10, 0.35 * max(0.0, -swing))
        self.p.resetJointState(body, 13, 0.35 * max(0.0, swing))
        self.p.resetJointState(body, 4, 0.22 * max(0.0, swing))
        self.p.resetJointState(body, 7, 0.22 * max(0.0, -swing))

    def _capture_gif_frame(self, dt: float) -> None:
        if self.record_gif is None:
            return
        self._record_accumulator += max(0.0, dt)
        frame_interval = 1.0 / self.record_fps
        if self._record_accumulator < frame_interval:
            return
        self._record_accumulator %= frame_interval

        from PIL import Image, ImageDraw

        width, height = 560, 315
        separation = math.hypot(
            self.pedestrian_position[0] - self.forklift_x,
            self.pedestrian_position[1] - self.forklift_y,
        )
        camera_target = [
            (self.forklift_x + self.pedestrian_position[0]) / 2 + 0.6,
            (self.forklift_y + self.pedestrian_position[1]) / 2,
            0.9,
        ]
        view = self.p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=camera_target,
            distance=max(9.5, min(13.0, 9.0 + separation * 0.45)),
            yaw=55,
            pitch=-34,
            roll=0,
            upAxisIndex=2,
        )
        projection = self.p.computeProjectionMatrixFOV(
            fov=55,
            aspect=width / height,
            nearVal=0.1,
            farVal=100,
        )
        _, _, pixels, _, _ = self.p.getCameraImage(
            width,
            height,
            viewMatrix=view,
            projectionMatrix=projection,
            renderer=self.p.ER_TINY_RENDERER,
        )
        frame = Image.frombytes("RGBA", (width, height), bytes(pixels)).convert("RGB")
        draw = ImageDraw.Draw(frame)
        draw.rectangle((0, 0, width, 38), fill=(12, 17, 24))
        state_color = (
            (40, 220, 90)
            if self.outputs.safety_state == SafetyState.RUN_FULL
            else (255, 180, 20)
            if self.outputs.safety_state == SafetyState.RUN_REDUCED
            else (255, 65, 65)
        )
        draw.text(
            (10, 7),
            f"PLC {self.outputs.safety_state.name}  |  {self.outputs.fault_code.name}",
            fill=state_color,
        )
        draw.text(
            (10, 22),
            f"Forklift {self.speed_m_s:.2f} m/s  |  Human path: {self.faults.pedestrian_zone}",
            fill=(230, 235, 240),
        )
        self._recorded_frames.append(frame)

    def _save_gif(self) -> None:
        if self.record_gif is None or not self._recorded_frames:
            return
        self.record_gif.parent.mkdir(parents=True, exist_ok=True)
        first, *remaining = self._recorded_frames
        first.save(
            self.record_gif,
            save_all=True,
            append_images=remaining,
            duration=round(1000 / self.record_fps),
            loop=0,
            optimize=True,
        )
        LOG.info("saved GIF: %s", self.record_gif)

    def _scanner_reading(self) -> tuple[bool, bool, int]:
        pedestrian = self.pedestrian_position
        scanner_x = self.forklift_x + self.scanner_origin_offset_m
        dx = pedestrian[0] - scanner_x
        dy = pedestrian[1] - self.forklift_y
        in_warning = 0 <= dx <= self.warning_length_m and abs(dy) <= self.warning_width_m / 2
        in_protective = (
            0 <= dx <= self.protective_length_m and abs(dy) <= self.protective_width_m / 2
        )
        nearest_mm = min(0xFFFF, round(math.hypot(dx, dy) * 1000))
        return not in_warning, not in_protective, nearest_mm

    def _update_visuals(self) -> None:
        p = self.p
        x = self.forklift_x
        positions = {
            "forklift": [x, 0, 0.45],
            "mast": [x + 0.75, 0, 1.1],
            "fork_l": [x + 1.45, -0.35, 0.12],
            "fork_r": [x + 1.45, 0.35, 0.12],
            "warning_zone": [
                x + self.scanner_origin_offset_m + self.warning_length_m / 2,
                0,
                0.025,
            ],
            "protective_zone": [
                x + self.scanner_origin_offset_m + self.protective_length_m / 2,
                0,
                0.055,
            ],
        }
        for name, position in positions.items():
            p.resetBasePositionAndOrientation(self._bodies[name], position, [0, 0, 0, 1])
        pedestrian, pedestrian_orientation = self._pedestrian_pose()
        p.resetBasePositionAndOrientation(
            self._bodies["pedestrian"], pedestrian, pedestrian_orientation
        )
        self._animate_pedestrian_gait()

        color = {
            SafetyState.RUN_FULL: [0.1, 1.0, 0.2],
            SafetyState.RUN_REDUCED: [1.0, 0.7, 0.0],
        }.get(self.outputs.safety_state, [1.0, 0.1, 0.1])
        text = (
            f"PLC: {self.outputs.safety_state.name} | Fault: {self.outputs.fault_code.name} | "
            f"Speed: {self.speed_m_s:.2f} m/s | "
            f"Limit: {self.outputs.speed_limit_mm_s / 1000:.2f} m/s"
        )
        if self.route_complete:
            text += " | ROUTE COMPLETE"
        self._status_item = p.addUserDebugText(
            text,
            [x - 2.0, -2.8, 2.6],
            textColorRGB=color,
            textSize=1.2,
            replaceItemUniqueId=self._status_item,
        )
