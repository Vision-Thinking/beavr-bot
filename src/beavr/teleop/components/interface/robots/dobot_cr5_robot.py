"""BeaVR robot interface for a Dobot CR5 over direct Dashboard TCP."""

from __future__ import annotations

import logging
import time
from typing import Any

import numpy as np

from beavr.dobot import DobotCR5Control
from beavr.teleop.common.network.handshake import HandshakeCoordinator
from beavr.teleop.common.network.publisher import ZMQPublisherManager
from beavr.teleop.common.network.subscriber import ZMQSubscriber
from beavr.teleop.common.network.utils import cleanup_zmq_resources
from beavr.teleop.components.detector.detector_types import SessionCommand
from beavr.teleop.components.interface.interface_base import RobotWrapper
from beavr.teleop.components.interface.interface_types import (
    CartesianState,
    CommandedCartesianState,
)
from beavr.teleop.components.operator.operator_types import CartesianTarget
from beavr.teleop.configs.constants import robots

logger = logging.getLogger(__name__)


class DobotCR5Robot(RobotWrapper):
    """Direct-TCP CR5 interface following BeaVR's XArm robot data flow."""

    def __init__(
        self,
        host: str,
        endeff_subscribe_port: int,
        reset_subscribe_port: int,
        teleoperation_state_port: int,
        robot_host: str,
        *,
        robot_port: int = 29999,
        is_right_arm: bool = True,
        endeff_publish_port: int,
        state_publish_port: int,
        home_subscribe_port: int,
        dry_run: bool = True,
        speed_percent: int = 10,
        accel_percent: int = 10,
        max_step_mm: float = 5.0,
        workspace_radius_mm: float = 100.0,
        deadband_mm: float = 0.5,
        gripper_force: int = 30,
        enable_gripper: bool = True,
        **_: Any,
    ) -> None:
        if not endeff_publish_port:
            raise ValueError("DobotCR5Robot requires endeff_publish_port")
        if not state_publish_port:
            raise ValueError("DobotCR5Robot requires state_publish_port")

        self._is_right_arm = bool(is_right_arm)
        self._data_frequency = robots.VR_FREQ
        self._controller = DobotCR5Control(
            robot_host=robot_host,
            robot_port=robot_port,
            dry_run=dry_run,
            speed_percent=speed_percent,
            accel_percent=accel_percent,
            max_step_mm=max_step_mm,
            workspace_radius_mm=workspace_radius_mm,
            deadband_mm=deadband_mm,
            gripper_force=gripper_force,
            enable_gripper=enable_gripper,
        )

        self._cartesian_coords_subscriber = ZMQSubscriber(
            host=host,
            port=endeff_subscribe_port,
            topic="endeff_coords",
            message_type=CartesianTarget,
        )
        self._reset_subscriber = ZMQSubscriber(
            host=host,
            port=reset_subscribe_port,
            topic="reset",
            message_type=SessionCommand,
        )
        self._home_subscriber = ZMQSubscriber(
            host=host,
            port=home_subscribe_port,
            topic="home",
            message_type=SessionCommand,
        )
        self._teleop_state_subscriber = ZMQSubscriber(
            host=host,
            port=teleoperation_state_port,
            topic="pause",
            message_type=SessionCommand,
        )

        self._publisher_manager = ZMQPublisherManager.get_instance()
        self._publisher_host = host
        self._endeff_publish_port = endeff_publish_port
        self._state_publish_port = state_publish_port

        self._latest_commanded_cartesian_position: np.ndarray | None = None
        self._latest_commanded_cartesian_timestamp = 0.0
        self._last_reset_timestamp: float | None = None
        self._last_home_timestamp: float | None = None
        self._last_pause_command = robots.RESUME

        self._handshake_coordinator = HandshakeCoordinator.get_instance()
        self._handshake_server_id = f"{self.name}_handshake"
        self._handshake_coordinator.start_server(
            subscriber_id=self._handshake_server_id,
            bind_host="*",
            port=robots.TELEOP_HANDSHAKE_PORT + (11 if self._is_right_arm else 12),
        )

    @property
    def name(self) -> str:
        side = robots.RIGHT if self._is_right_arm else robots.LEFT
        return f"{robots.ROBOT_NAME_DOBOT_CR5}_{side}"

    @property
    def recorder_functions(self):
        return {
            "joint_states": self.get_joint_state,
            "dobot_cartesian_states": self.get_robot_actual_cartesian_position,
            "commanded_cartesian_state": self.get_cartesian_commanded_position,
            "joint_angles_rad": self.get_joint_position,
            "gripper_state": self.get_gripper_state,
            "error_state": self.get_error_state,
        }

    @property
    def data_frequency(self) -> int:
        return self._data_frequency

    def get_joint_state(self):
        joint_position = self.get_joint_position()
        if joint_position is None:
            return None
        return {"joint_position": joint_position, "timestamp": time.time()}

    def get_joint_position(self):
        try:
            return list(np.asarray(self._controller.get_arm_position(), dtype=np.float32))
        except Exception as error:
            logger.warning("Failed to read Dobot joint position: %s", error)
            return None

    def get_joint_velocity(self):
        return None

    def get_joint_torque(self):
        return None

    def get_cartesian_position(self):
        try:
            return self._controller.get_arm_cartesian_coords()
        except Exception as error:
            logger.warning("Failed to read Dobot Cartesian position: %s", error)
            return None

    def get_pose(self):
        return self._controller.get_arm_pose()

    def reset(self):
        self.send_robot_pose()
        return True

    def home(self):
        # Homing a CR5 is a physical motion and is deliberately left to the operator.
        self.send_robot_pose()
        return True

    def move(self, input_angles):
        raise NotImplementedError("DobotCR5Robot does not support direct joint commands yet")

    def move_coords(self, cartesian_coords):
        return self._controller.move_arm_cartesian(cartesian_coords)

    def get_cartesian_commanded_position(self):
        if self._latest_commanded_cartesian_position is None:
            return None
        return CommandedCartesianState(
            commanded_cartesian_position=self._latest_commanded_cartesian_position.tolist(),
            timestamp_s=self._latest_commanded_cartesian_timestamp,
        )

    def get_robot_actual_cartesian_position(self):
        cartesian = self.get_cartesian_position()
        if cartesian is None:
            return None
        return CartesianState(
            position_m=tuple(float(v) for v in np.asarray(cartesian[:3], dtype=np.float32)),
            timestamp_s=time.time(),
        )

    def get_gripper_state(self):
        state = self._controller.get_gripper_state()
        if state is None:
            return None
        init_status, grip_status, position = state
        return {
            "init_status": int(init_status),
            "grip_status": int(grip_status),
            "position": int(position),
            "timestamp": time.time(),
        }

    def get_error_state(self):
        errors = self._controller.get_error_ids()
        if not errors and self._controller.latest_error is None:
            return None
        return {
            "error_ids": list(errors),
            "message": self._controller.latest_error,
            "timestamp": time.time(),
        }

    def send_robot_pose(self):
        try:
            pose = self._controller.get_arm_pose()
            h_matrix = tuple(tuple(float(x) for x in row) for row in pose)
            self._publisher_manager.publish(
                host=self._publisher_host,
                port=self._endeff_publish_port,
                topic="endeff_homo",
                data=CartesianState(timestamp_s=time.time(), h_matrix=h_matrix),
            )
        except Exception as error:
            logger.error("Failed to publish Dobot pose for %s: %s", self.name, error)

    def _check_new_session_command(
        self,
        subscriber: ZMQSubscriber[SessionCommand],
        last_timestamp: float | None,
        expected_command: str,
    ) -> tuple[bool, float | None]:
        command = subscriber.recv_keypoints()
        if command is None or command.command != expected_command:
            return False, last_timestamp
        if last_timestamp is not None and command.timestamp_s <= last_timestamp:
            return False, last_timestamp
        return True, command.timestamp_s

    def check_reset(self) -> bool:
        matched, timestamp_s = self._check_new_session_command(
            self._reset_subscriber, self._last_reset_timestamp, "reset"
        )
        self._last_reset_timestamp = timestamp_s
        return matched

    def check_home(self) -> bool:
        matched, timestamp_s = self._check_new_session_command(
            self._home_subscriber, self._last_home_timestamp, "home"
        )
        self._last_home_timestamp = timestamp_s
        return matched

    def get_teleop_state(self) -> int:
        command = self._teleop_state_subscriber.recv_keypoints()
        if command is not None:
            self._last_pause_command = command.command
        return robots.ARM_TELEOP_STOP if self._last_pause_command == robots.PAUSE else robots.ARM_TELEOP_CONT

    def publish_current_state(self):
        publish_time = time.time()
        current_state_dict = {}

        joint_states = self.get_joint_state()
        robot_cart = self.get_robot_actual_cartesian_position()
        commanded_cart = self.get_cartesian_commanded_position()
        joint_angles_rad = self.get_joint_position()
        gripper_state = self.get_gripper_state()
        error_state = self.get_error_state()

        if joint_states is not None:
            current_state_dict["joint_states"] = joint_states
        if robot_cart is not None:
            current_state_dict["dobot_cartesian_states"] = robot_cart.to_dict()
        if commanded_cart is not None:
            current_state_dict["commanded_cartesian_state"] = commanded_cart.to_dict()
        if joint_angles_rad is not None:
            current_state_dict["joint_angles_rad"] = joint_angles_rad
        if gripper_state is not None:
            current_state_dict["gripper_state"] = gripper_state
        if error_state is not None:
            current_state_dict["error_state"] = error_state

        current_state_dict["timestamp"] = publish_time
        current_state_dict["dobot"] = {
            "dry_run": self._controller.dry_run,
            "command_count": self._controller.command_count,
        }

        self._publisher_manager.publish(
            host=self._publisher_host,
            port=self._state_publish_port,
            topic=self.name,
            data=current_state_dict,
        )

    def stream(self):
        self._controller.connect()
        self.send_robot_pose()

        target_interval = 1.0 / self._data_frequency
        next_frame_time = time.time()

        while True:
            current_time = time.time()
            if current_time < next_frame_time:
                time.sleep(max(0.0, next_frame_time - current_time))
                continue

            next_frame_time = current_time + target_interval

            if self.check_home() or self.check_reset():
                self.send_robot_pose()

            msg = self._cartesian_coords_subscriber.recv_keypoints()
            if msg is not None:
                self._latest_commanded_cartesian_position = np.asarray(
                    [*msg.position_m, *msg.orientation_xyzw],
                    dtype=np.float32,
                )
                self._latest_commanded_cartesian_timestamp = msg.timestamp_s

            if (
                self.get_teleop_state() == robots.ARM_TELEOP_CONT
                and self._latest_commanded_cartesian_position is not None
            ):
                try:
                    self.move_coords(self._latest_commanded_cartesian_position)
                except Exception as error:
                    logger.error("Dobot CR5 motion rejected: %s", error)

            self.publish_current_state()

    def __del__(self):
        if hasattr(self, "_handshake_coordinator") and hasattr(self, "_handshake_server_id"):
            self._handshake_coordinator.stop_server(self._handshake_server_id)
        if hasattr(self, "_controller"):
            self._controller.close()
        cleanup_zmq_resources()
