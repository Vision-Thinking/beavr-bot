"""Teleoperation config for Dobot CR5 arms over direct Dashboard TCP."""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from typing import Any

from beavr.teleop.common.configs.loader import Laterality, log_laterality_configuration
from beavr.teleop.components.interface.robots.dobot_cr5_robot import DobotCR5Robot
from beavr.teleop.configs.constants import network, ports, robots
from beavr.teleop.configs.robots import TeleopRobotConfig
from beavr.teleop.configs.robots.shared_components import SharedComponentRegistry

logger = logging.getLogger(__name__)


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    return default if value is None else int(value)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    return default if value is None else float(value)


@dataclass
class DobotCR5RobotCfg:
    host: str = network.HOST_ADDRESS
    robot_host: str = field(default_factory=lambda: os.environ.get("DOBOT_ROBOT_HOST", network.RIGHT_DOBOT_CR5_IP))
    robot_port: int = field(default_factory=lambda: _env_int("DOBOT_ROBOT_PORT", 29999))
    is_right_arm: bool = True
    dry_run: bool = field(default_factory=lambda: _env_bool("DOBOT_DRY_RUN", True))
    speed_percent: int = field(default_factory=lambda: _env_int("DOBOT_SPEED_PERCENT", 10))
    accel_percent: int = field(default_factory=lambda: _env_int("DOBOT_ACCEL_PERCENT", 10))
    max_step_mm: float = field(default_factory=lambda: _env_float("DOBOT_MAX_STEP_MM", 5.0))
    workspace_radius_mm: float = field(default_factory=lambda: _env_float("DOBOT_WORKSPACE_RADIUS_MM", 100.0))
    deadband_mm: float = field(default_factory=lambda: _env_float("DOBOT_DEADBAND_MM", 0.5))
    gripper_force: int = field(default_factory=lambda: _env_int("DOBOT_GRIPPER_FORCE", 30))
    enable_gripper: bool = field(default_factory=lambda: _env_bool("DOBOT_ENABLE_GRIPPER", True))
    endeff_publish_port: int = ports.DOBOT_CR5_ENDEFF_PUBLISH_PORT
    endeff_subscribe_port: int = ports.DOBOT_CR5_ENDEFF_SUBSCRIBE_PORT
    reset_subscribe_port: int = ports.DOBOT_CR5_RESET_SUBSCRIBE_PORT
    state_publish_port: int = ports.DOBOT_CR5_STATE_PUBLISH_PORT
    home_subscribe_port: int = ports.DOBOT_CR5_HOME_SUBSCRIBE_PORT
    teleoperation_state_port: int = ports.DOBOT_CR5_TELEOPERATION_STATE_PORT
    hand_side: str = robots.RIGHT
    recorder_config: dict[str, Any] = field(
        default_factory=lambda: {
            "robot_identifier": robots.ROBOT_IDENTIFIER_RIGHT_DOBOT_CR5,
            "recorded_data": [
                robots.RECORDED_DATA_JOINT_STATES,
                robots.RECORDED_DATA_DOBOT_CARTESIAN_STATES,
                robots.RECORDED_DATA_COMMANDED_CARTESIAN_STATE,
                robots.RECORDED_DATA_JOINT_ANGLES_RAD,
            ],
        }
    )

    def __post_init__(self) -> None:
        all_ports = [
            self.robot_port,
            self.endeff_publish_port,
            self.endeff_subscribe_port,
            self.reset_subscribe_port,
            self.state_publish_port,
            self.home_subscribe_port,
            self.teleoperation_state_port,
        ]
        for port in all_ports:
            if not (1 <= int(port) <= 65535):
                raise ValueError(f"Port out of valid range (1-65535): {port}")
        if not self.dry_run and not self.robot_host:
            raise ValueError("robot_host is required when DOBOT_DRY_RUN=0")

    def build(self):
        return DobotCR5Robot(
            host=self.host,
            robot_host=self.robot_host,
            robot_port=self.robot_port,
            is_right_arm=self.is_right_arm,
            dry_run=self.dry_run,
            speed_percent=self.speed_percent,
            accel_percent=self.accel_percent,
            max_step_mm=self.max_step_mm,
            workspace_radius_mm=self.workspace_radius_mm,
            deadband_mm=self.deadband_mm,
            gripper_force=self.gripper_force,
            enable_gripper=self.enable_gripper,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            reset_subscribe_port=self.reset_subscribe_port,
            state_publish_port=self.state_publish_port,
            home_subscribe_port=self.home_subscribe_port,
            teleoperation_state_port=self.teleoperation_state_port,
        )


@dataclass
class DobotCR5OperatorCfg:
    host: str = network.HOST_ADDRESS
    transformed_keypoints_port: int = ports.KEYPOINT_TRANSFORM_PORT
    stream_configs: dict[str, Any] = field(
        default_factory=lambda: {
            "host": network.HOST_ADDRESS,
            "port": ports.CONTROL_STREAM_PORT,
        }
    )
    stream_oculus: bool = True
    endeff_publish_port: int = ports.DOBOT_CR5_ENDEFF_SUBSCRIBE_PORT
    endeff_subscribe_port: int = ports.DOBOT_CR5_ENDEFF_PUBLISH_PORT
    moving_average_limit: int = 3
    arm_resolution_port: int = ports.KEYPOINT_STREAM_PORT
    use_filter: bool = False
    teleoperation_state_port: int = ports.DOBOT_CR5_TELEOPERATION_STATE_PORT
    logging_config: dict[str, Any] = field(
        default_factory=lambda: {
            "enabled": False,
            "log_dir": "logs",
            "log_poses": True,
            "log_prefix": "dobot_cr5",
        }
    )
    hand_side: str = robots.RIGHT

    def build(self):
        if self.hand_side == robots.RIGHT:
            from beavr.teleop.components.operator.robots.dobot_cr5_operator import (
                DobotCR5RightOperator,
            )

            operator_class = DobotCR5RightOperator
        else:
            from beavr.teleop.components.operator.robots.dobot_cr5_operator import (
                DobotCR5LeftOperator,
            )

            operator_class = DobotCR5LeftOperator

        return operator_class(
            host=self.host,
            transformed_keypoints_port=self.transformed_keypoints_port,
            stream_configs=self.stream_configs,
            stream_oculus=self.stream_oculus,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            moving_average_limit=self.moving_average_limit,
            arm_resolution_port=self.arm_resolution_port,
            use_filter=self.use_filter,
            teleoperation_state_port=self.teleoperation_state_port,
            logging_config=self.logging_config,
        )


@dataclass
@TeleopRobotConfig.register_subclass(robots.ROBOT_NAME_DOBOT_CR5)
class DobotCR5Config:
    robot_name: str = robots.ROBOT_NAME_DOBOT_CR5
    laterality: Laterality = Laterality.RIGHT

    detector: list = field(default_factory=list)
    transforms: list = field(default_factory=list)
    visualizers: list = field(default_factory=list)
    robots: list = field(default_factory=list)
    operators: list = field(default_factory=list)

    def __post_init__(self) -> None:
        log_laterality_configuration(self.laterality, robots.ROBOT_NAME_DOBOT_CR5)
        self._configure_for_laterality()

    def _configure_for_laterality(self) -> None:
        self.detector = []
        if self.laterality == Laterality.BIMANUAL:
            self.detector.append(SharedComponentRegistry.get_bimanual_detector_config(host=network.HOST_ADDRESS))
        else:
            hand_side = robots.RIGHT if self.laterality == Laterality.RIGHT else robots.LEFT
            self.detector.append(
                SharedComponentRegistry.get_detector_config(
                    hand_side=hand_side,
                    host=network.HOST_ADDRESS,
                )
            )

        self.transforms = []
        if self.laterality in [Laterality.RIGHT, Laterality.BIMANUAL]:
            self.transforms.append(
                SharedComponentRegistry.get_transform_config(
                    hand_side=robots.RIGHT,
                    host=network.HOST_ADDRESS,
                    keypoint_sub_port=ports.KEYPOINT_STREAM_PORT,
                    moving_average_limit=3,
                )
            )
        if self.laterality in [Laterality.LEFT, Laterality.BIMANUAL]:
            self.transforms.append(
                SharedComponentRegistry.get_transform_config(
                    hand_side=robots.LEFT,
                    host=network.HOST_ADDRESS,
                    keypoint_sub_port=ports.KEYPOINT_STREAM_PORT,
                    moving_average_limit=3,
                )
            )

        self.visualizers = []
        self.robots = []
        self.operators = []

        if self.laterality in [Laterality.RIGHT, Laterality.BIMANUAL]:
            self.robots.append(
                DobotCR5RobotCfg(
                    host=network.HOST_ADDRESS,
                    robot_host=os.environ.get("DOBOT_ROBOT_HOST", network.RIGHT_DOBOT_CR5_IP),
                    is_right_arm=True,
                    hand_side=robots.RIGHT,
                )
            )
            self.operators.append(
                DobotCR5OperatorCfg(
                    host=network.HOST_ADDRESS,
                    transformed_keypoints_port=ports.KEYPOINT_TRANSFORM_PORT,
                    endeff_publish_port=ports.DOBOT_CR5_ENDEFF_SUBSCRIBE_PORT,
                    endeff_subscribe_port=ports.DOBOT_CR5_ENDEFF_PUBLISH_PORT,
                    hand_side=robots.RIGHT,
                    logging_config={
                        "enabled": False,
                        "log_dir": "logs",
                        "log_poses": True,
                        "log_prefix": "dobot_cr5_right",
                    },
                )
            )

        if self.laterality in [Laterality.LEFT, Laterality.BIMANUAL]:
            self.robots.append(
                DobotCR5RobotCfg(
                    host=network.HOST_ADDRESS,
                    robot_host=os.environ.get("DOBOT_LEFT_ROBOT_HOST", network.LEFT_DOBOT_CR5_IP),
                    is_right_arm=False,
                    endeff_publish_port=ports.DOBOT_CR5_ENDEFF_PUBLISH_PORT + 2,
                    endeff_subscribe_port=ports.DOBOT_CR5_ENDEFF_SUBSCRIBE_PORT + 2,
                    reset_subscribe_port=ports.DOBOT_CR5_RESET_SUBSCRIBE_PORT + 2,
                    state_publish_port=ports.DOBOT_CR5_STATE_PUBLISH_PORT + 1,
                    home_subscribe_port=ports.DOBOT_CR5_HOME_SUBSCRIBE_PORT + 2,
                    hand_side=robots.LEFT,
                    recorder_config={
                        "robot_identifier": robots.ROBOT_IDENTIFIER_LEFT_DOBOT_CR5,
                        "recorded_data": [
                            robots.RECORDED_DATA_JOINT_STATES,
                            robots.RECORDED_DATA_DOBOT_CARTESIAN_STATES,
                            robots.RECORDED_DATA_COMMANDED_CARTESIAN_STATE,
                            robots.RECORDED_DATA_JOINT_ANGLES_RAD,
                        ],
                    },
                )
            )
            self.operators.append(
                DobotCR5OperatorCfg(
                    host=network.HOST_ADDRESS,
                    transformed_keypoints_port=ports.LEFT_KEYPOINT_TRANSFORM_PORT,
                    endeff_publish_port=ports.DOBOT_CR5_ENDEFF_SUBSCRIBE_PORT + 2,
                    endeff_subscribe_port=ports.DOBOT_CR5_ENDEFF_PUBLISH_PORT + 2,
                    hand_side=robots.LEFT,
                    logging_config={
                        "enabled": False,
                        "log_dir": "logs",
                        "log_poses": True,
                        "log_prefix": "dobot_cr5_left",
                    },
                )
            )

    def build(self):
        return {
            "robot_name": self.robot_name,
            "detector": [detector.build() for detector in self.detector],
            "transforms": [item.build() for item in self.transforms],
            "visualizers": [item.build() for item in self.visualizers],
            "robots": [item.build() for item in self.robots],
            "operators": [item.build() for item in self.operators],
        }
