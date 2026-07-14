"""Dobot CR-series Dashboard TCP protocol helpers.

The CR5 controller exposes a text Dashboard protocol on TCP 29999. Commands are
sent without newlines and successful responses start with ``0,`` and end in
``;``. This module intentionally keeps the same conservative command set used
by the bimanual cube project: status reads, bounded ``RelMovLUser`` point moves,
and explicit motion-result polling.
"""

from __future__ import annotations

import math
import socket
import time
from typing import Callable, Sequence

import numpy as np


class DashboardClient:
    """Exclusive TCP client for the Dobot Dashboard port."""

    def __init__(self, host: str, port: int = 29999, timeout: float = 5.0) -> None:
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self._sock: socket.socket | None = None

    def connect(self) -> None:
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except OSError as error:
            raise ConnectionError(
                f"Cannot connect to Dobot at {self.host}:{self.port}: {error}. "
                "Port 29999 may be occupied by another Dashboard/ROS client."
            ) from error

    def send(self, command: str) -> str:
        if self._sock is None:
            raise RuntimeError("DashboardClient is not connected")
        if not command or "\n" in command or "\r" in command:
            raise ValueError("Dashboard command must be non-empty and newline-free")

        self._sock.sendall(command.encode("ascii"))
        self._sock.settimeout(self.timeout)
        buffer = b""
        while not buffer.endswith(b";"):
            try:
                chunk = self._sock.recv(4096)
            except TimeoutError as error:
                if buffer:
                    raise ConnectionError(
                        f"Dobot returned an unterminated response for {command!r}: "
                        f"{buffer.decode(errors='replace').strip()!r}"
                    ) from error
                raise TimeoutError(f"Timed out waiting for Dobot response to {command!r}") from error
            if not chunk:
                raise ConnectionError(f"Dobot closed the connection after {command!r}")
            buffer += chunk
            if len(buffer) > 65536:
                raise ConnectionError("Dobot response exceeded 64 KiB")
        return buffer.decode("ascii", errors="replace").strip()

    def close(self) -> None:
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()


def require_success(command: str, reply: str) -> None:
    if not reply.startswith("0,"):
        raise RuntimeError(f"Dobot command rejected: {command} -> {reply}")


def parse_values(reply: str, command: str) -> list[float]:
    """Parse numeric values from a ``0,{...},Cmd();`` response."""

    require_success(command, reply)
    start = reply.find("{")
    end = reply.find("}", start + 1)
    if start < 0 or end < 0:
        raise RuntimeError(f"{command} reply has no value block: {reply!r}")

    payload = reply[start + 1 : end].strip()
    if payload.startswith("[") or payload.endswith("]"):
        if not (payload.startswith("[") and payload.endswith("]")):
            raise RuntimeError(f"{command} reply list is malformed: {reply!r}")
        payload = payload[1:-1].strip()
    if not payload:
        return []

    try:
        values = [float(value.strip()) for value in payload.split(",")]
    except ValueError as error:
        raise RuntimeError(f"{command} reply is not numeric: {reply!r}") from error
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"{command} reply contains non-finite values")
    return values


def parse_error_ids(reply: str) -> list[int]:
    """Parse ``GetErrorID`` without treating ambiguous payloads as no error."""

    start = reply.find("{")
    end = reply.find("}", start + 1)
    if start < 0 or end < 0 or not reply[start + 1 : end].strip():
        raise RuntimeError(f"GetErrorID() did not return an explicit error list: {reply!r}")

    values = parse_values(reply, "GetErrorID()")
    if any(not value.is_integer() for value in values):
        raise RuntimeError(f"GetErrorID() returned a non-integer error code: {reply!r}")
    return [int(value) for value in values]


def euler_deg_to_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """Return ``Rz * Ry * Rx`` for Dobot Dashboard pose Euler values."""

    rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
    rx_mat = np.array(
        [[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]],
        dtype=float,
    )
    ry_mat = np.array(
        [[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]],
        dtype=float,
    )
    rz_mat = np.array(
        [[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]],
        dtype=float,
    )
    return rz_mat @ ry_mat @ rx_mat


class DashboardRobot:
    """Small CR5 motion/status adapter over one Dashboard connection."""

    def __init__(
        self,
        host: str,
        port: int = 29999,
        *,
        timeout: float = 5.0,
        speed_percent: int = 10,
        accel_percent: int = 10,
        poll_interval_s: float = 0.05,
        motion_timeout_s: float = 30.0,
        settle_samples: int = 10,
        client_factory: Callable[..., DashboardClient] = DashboardClient,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if not 1 <= int(speed_percent) <= 100:
            raise ValueError("speed_percent must be in [1, 100]")
        if not 1 <= int(accel_percent) <= 100:
            raise ValueError("accel_percent must be in [1, 100]")
        if poll_interval_s < 0 or motion_timeout_s <= 0 or settle_samples < 3:
            raise ValueError("invalid motion polling configuration")

        self.client = client_factory(host, port, timeout)
        self.speed_percent = int(speed_percent)
        self.accel_percent = int(accel_percent)
        self.poll_interval_s = float(poll_interval_s)
        self.motion_timeout_s = float(motion_timeout_s)
        self.settle_samples = int(settle_samples)
        self.sleep = sleep
        self.command_count = 0

    def _send(self, command: str) -> str:
        reply = self.client.send(command)
        require_success(command, reply)
        return reply

    def connect(self) -> None:
        self.client.connect()
        try:
            for command in (
                "EnableRobot()",
                f"VelL({self.speed_percent})",
                f"AccL({self.accel_percent})",
                f"VelJ({self.speed_percent})",
                f"AccJ({self.accel_percent})",
                "SpeedFactor(100)",
            ):
                self._send(command)
            self.require_idle()
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        self.client.close()

    def error_ids(self) -> list[int]:
        return parse_error_ids(self._send("GetErrorID()"))

    def robot_mode(self) -> int:
        values = parse_values(self._send("RobotMode()"), "RobotMode()")
        if len(values) != 1 or not values[0].is_integer():
            raise RuntimeError(f"RobotMode() must return one integer, got {values!r}")
        return int(values[0])

    def require_idle(self) -> None:
        errors = self.error_ids()
        if errors:
            raise RuntimeError(f"Dobot controller errors: {errors}")
        mode = self.robot_mode()
        if mode != 5:
            raise RuntimeError(f"Dobot is not enabled and idle (RobotMode={mode})")

    def get_joint_angles(self) -> np.ndarray:
        values = parse_values(self._send("GetAngle()"), "GetAngle()")
        if len(values) != 6:
            raise RuntimeError(f"GetAngle() must return 6 values, got {values!r}")
        return np.radians(np.asarray(values, dtype=np.float32))

    def get_pose(self) -> np.ndarray:
        values = parse_values(self._send("GetPose()"), "GetPose()")
        if len(values) != 6:
            raise RuntimeError(f"GetPose() must return 6 values, got {values!r}")
        x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = values
        pose = np.eye(4, dtype=float)
        pose[:3, 3] = [x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0]
        pose[:3, :3] = euler_deg_to_matrix(rx_deg, ry_deg, rz_deg)
        return pose

    def move_relative(self, delta_xyz_m: Sequence[float]) -> None:
        delta = np.asarray(delta_xyz_m, dtype=float)
        if delta.shape != (3,) or not np.all(np.isfinite(delta)):
            raise ValueError("translation delta must contain three finite values")
        dx_mm, dy_mm, dz_mm = delta * 1000.0
        command = f"RelMovLUser({dx_mm:.3f},{dy_mm:.3f},{dz_mm:.3f},0.000,0.000,0.000)"
        self._send(command)
        self.command_count += 1

    def wait_motion_result(self) -> None:
        deadline = time.monotonic() + self.motion_timeout_s
        idle_samples = 0
        samples = 0
        while time.monotonic() < deadline:
            errors = self.error_ids()
            mode = self.robot_mode()
            if errors:
                raise RuntimeError(f"Dobot asynchronous motion error: mode={mode}, errors={errors}")
            samples += 1
            idle_samples = idle_samples + 1 if mode == 5 else 0
            if samples >= self.settle_samples and idle_samples >= 3:
                return
            self.sleep(self.poll_interval_s)
        raise TimeoutError("Timed out waiting for Dobot motion completion")

    def set_gripper(self, position: int, force: int) -> None:
        from .gripper import set_gripper

        set_gripper(self.client, position, force, sleep=self.sleep)
        errors = self.error_ids()
        if errors:
            raise RuntimeError(f"Dobot gripper errors: {errors}")
        self.command_count += 1

    def read_gripper_state(self) -> tuple[int, int, int]:
        from .gripper import read_gripper_state

        return read_gripper_state(self.client, sleep=self.sleep)


class DryRunDobotRobot:
    """In-memory CR5 used for dry runs and tests."""

    def __init__(self, initial_pose: np.ndarray | None = None) -> None:
        self.pose = np.eye(4, dtype=float) if initial_pose is None else np.asarray(initial_pose, dtype=float)
        self.joint_angles = np.zeros(6, dtype=np.float32)
        self.command_count = 0
        self.connected = False
        self.commands: list[tuple] = []
        self.gripper_position = 1000

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def require_idle(self) -> None:
        if not self.connected:
            raise RuntimeError("Dry-run Dobot is not connected")

    def error_ids(self) -> list[int]:
        return []

    def robot_mode(self) -> int:
        return 5

    def get_joint_angles(self) -> np.ndarray:
        return self.joint_angles.copy()

    def get_pose(self) -> np.ndarray:
        return self.pose.copy()

    def move_relative(self, delta_xyz_m: Sequence[float]) -> None:
        self.require_idle()
        delta = np.asarray(delta_xyz_m, dtype=float)
        if delta.shape != (3,) or not np.all(np.isfinite(delta)):
            raise ValueError("translation delta must contain three finite values")
        self.pose[:3, 3] += delta
        self.commands.append(("move_relative", delta.copy()))
        self.command_count += 1

    def wait_motion_result(self) -> None:
        self.require_idle()

    def set_gripper(self, position: int, force: int) -> None:
        self.require_idle()
        self.gripper_position = int(position)
        self.commands.append(("gripper", int(position), int(force)))
        self.command_count += 1

    def read_gripper_state(self) -> tuple[int, int, int]:
        return (1, 0, self.gripper_position)
