"""High-level Dobot CR5 control used by BeaVR interfaces."""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .dashboard import DashboardRobot, DryRunDobotRobot

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DobotCR5State:
    timestamp_s: float
    joint_positions_rad: tuple[float, ...]
    tcp_pose: np.ndarray
    gripper_state: tuple[int, int, int] | None
    errors: tuple[int, ...]
    command_count: int
    dry_run: bool


class DobotCR5Control:
    """Conservative CR5 controller with a BeaVR-friendly API.

    The controller accepts absolute Cartesian targets in the robot base frame,
    but sends only bounded XYZ translation steps with ``RelMovLUser``. Target
    orientation is kept in the recording path and deliberately ignored until
    the Dobot Euler/tool-frame convention is verified for this setup.
    """

    def __init__(
        self,
        robot_host: str,
        robot_port: int = 29999,
        *,
        dry_run: bool = True,
        speed_percent: int = 10,
        accel_percent: int = 10,
        max_step_mm: float = 5.0,
        workspace_radius_mm: float = 100.0,
        deadband_mm: float = 0.5,
        gripper_force: int = 30,
        enable_gripper: bool = True,
        initial_pose_xyz_m: Sequence[float] = (0.35, 0.0, 0.35),
    ) -> None:
        if not dry_run and not robot_host:
            raise ValueError("robot_host is required when dry_run is false")
        if max_step_mm <= 0 or workspace_radius_mm <= 0 or deadband_mm < 0:
            raise ValueError("motion limits must be positive")
        if max_step_mm > workspace_radius_mm:
            raise ValueError("max_step_mm cannot exceed workspace_radius_mm")
        if not 1 <= int(speed_percent) <= 100 or not 1 <= int(accel_percent) <= 100:
            raise ValueError("speed_percent and accel_percent must be in [1, 100]")
        if not 20 <= int(gripper_force) <= 100:
            raise ValueError("gripper_force must be in [20, 100]")

        self.robot_host = robot_host
        self.robot_port = int(robot_port)
        self.dry_run = bool(dry_run)
        self.speed_percent = int(speed_percent)
        self.accel_percent = int(accel_percent)
        self.max_step_m = float(max_step_mm) / 1000.0
        self.workspace_radius_m = float(workspace_radius_mm) / 1000.0
        self.deadband_m = float(deadband_mm) / 1000.0
        self.gripper_force = int(gripper_force)
        self.enable_gripper = bool(enable_gripper)

        initial_pose = np.eye(4, dtype=float)
        initial_pose[:3, 3] = np.asarray(initial_pose_xyz_m, dtype=float)
        if self.dry_run:
            self._robot = DryRunDobotRobot(initial_pose)
        else:
            self._robot = DashboardRobot(
                robot_host,
                self.robot_port,
                speed_percent=self.speed_percent,
                accel_percent=self.accel_percent,
            )

        self._lock = threading.RLock()
        self._connected = False
        self._origin_pose: np.ndarray | None = None
        self._latest_commanded_cartesian: np.ndarray | None = None
        self._latest_error: str | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def latest_error(self) -> str | None:
        return self._latest_error

    @property
    def command_count(self) -> int:
        return int(getattr(self._robot, "command_count", 0))

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._robot.connect()
            pose = self._robot.get_pose()
            self._origin_pose = np.asarray(pose, dtype=float).copy()
            self._connected = True
            self._latest_error = None
            logger.info(
                "Dobot CR5 connected in %s mode at %s:%s",
                "dry-run" if self.dry_run else "real",
                "memory" if self.dry_run else self.robot_host,
                self.robot_port,
            )

    def close(self) -> None:
        with self._lock:
            try:
                self._robot.close()
            finally:
                self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("DobotCR5Control is not connected")

    def get_arm_pose(self) -> np.ndarray:
        with self._lock:
            self._require_connected()
            return self._robot.get_pose()

    def get_arm_position(self) -> np.ndarray:
        with self._lock:
            self._require_connected()
            return np.asarray(self._robot.get_joint_angles(), dtype=np.float32)

    def get_arm_cartesian_coords(self) -> np.ndarray:
        pose = self.get_arm_pose()
        quat_xyzw = Rotation.from_matrix(pose[:3, :3]).as_quat()
        if quat_xyzw[3] < 0:
            quat_xyzw = -quat_xyzw
        return np.concatenate([pose[:3, 3], quat_xyzw]).astype(np.float32)

    def get_gripper_state(self) -> tuple[int, int, int] | None:
        if not self.enable_gripper:
            return None
        with self._lock:
            self._require_connected()
            try:
                return self._robot.read_gripper_state()
            except Exception as error:
                self._latest_error = f"{type(error).__name__}: {error}"
                logger.warning("Could not read Dobot gripper state: %s", error)
                return None

    def get_error_ids(self) -> tuple[int, ...]:
        with self._lock:
            self._require_connected()
            try:
                return tuple(int(value) for value in self._robot.error_ids())
            except Exception as error:
                self._latest_error = f"{type(error).__name__}: {error}"
                logger.warning("Could not read Dobot errors: %s", error)
                return ()

    def state(self, timestamp_s: float) -> DobotCR5State:
        with self._lock:
            self._require_connected()
            joint_positions = tuple(float(v) for v in self._robot.get_joint_angles())
            pose = self._robot.get_pose()
            try:
                gripper_state = self._robot.read_gripper_state() if self.enable_gripper else None
            except Exception as error:
                gripper_state = None
                self._latest_error = f"{type(error).__name__}: {error}"
            try:
                errors = tuple(int(value) for value in self._robot.error_ids())
            except Exception as error:
                errors = ()
                self._latest_error = f"{type(error).__name__}: {error}"

            return DobotCR5State(
                timestamp_s=timestamp_s,
                joint_positions_rad=joint_positions,
                tcp_pose=pose,
                gripper_state=gripper_state,
                errors=errors,
                command_count=self.command_count,
                dry_run=self.dry_run,
            )

    def move_arm_cartesian(self, cartesian_pose: Sequence[float]) -> bool:
        """Move one bounded step toward ``[x, y, z, qx, qy, qz, qw]``."""

        target = np.asarray(cartesian_pose, dtype=float).flatten()
        if target.shape[0] < 3 or not np.all(np.isfinite(target[:3])):
            raise ValueError("cartesian target must contain at least finite x/y/z")

        with self._lock:
            self._require_connected()
            if self._origin_pose is None:
                raise RuntimeError("Dobot session origin is not initialized")

            target_xyz = target[:3].copy()
            origin_offset = target_xyz - self._origin_pose[:3, 3]
            if float(np.linalg.norm(origin_offset)) > self.workspace_radius_m:
                raise RuntimeError(
                    "Dobot target exceeds session workspace radius "
                    f"{self.workspace_radius_m * 1000.0:.1f} mm"
                )

            current_pose = self._robot.get_pose()
            delta = target_xyz - current_pose[:3, 3]
            distance = float(np.linalg.norm(delta))
            self._latest_commanded_cartesian = target.copy()

            if distance <= self.deadband_m:
                return False

            step = delta * min(1.0, self.max_step_m / distance)
            self._robot.require_idle()
            self._robot.move_relative(step)
            self._robot.wait_motion_result()
            self._latest_error = None
            return True

    def set_gripper_open(self, open_gripper: bool) -> None:
        if not self.enable_gripper:
            return
        position = 1000 if open_gripper else 0
        with self._lock:
            self._require_connected()
            self._robot.require_idle()
            self._robot.set_gripper(position, self.gripper_force)
