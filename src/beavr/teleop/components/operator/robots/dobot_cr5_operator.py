"""Quest/Oculus operator wrappers for Dobot CR5 arms."""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from beavr.teleop.components.operator.robots.xarm7_operator import XArmOperator
from beavr.teleop.configs.constants import robots

# Initial CR5 mapping follows the existing right XArm mapping. Keep these
# explicit so hand-eye/base calibration can be adjusted without touching the
# shared XArmOperator implementation.
H_R_V_RIGHT = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
H_T_V_RIGHT = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])

H_R_V_LEFT = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
H_T_V_LEFT = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])


class DobotCR5RightOperator(XArmOperator):
    def __init__(
        self,
        host: str,
        transformed_keypoints_port: int,
        stream_configs: Dict[str, Any],
        stream_oculus: bool,
        endeff_publish_port: int,
        endeff_subscribe_port: int,
        moving_average_limit: int,
        use_filter: bool = True,
        arm_resolution_port: Optional[int] = None,
        teleoperation_state_port: Optional[int] = None,
        logging_config: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(
            operator_name="dobot_cr5_right_operator",
            host=host,
            transformed_keypoints_port=transformed_keypoints_port,
            stream_configs=stream_configs,
            stream_oculus=stream_oculus,
            endeff_publish_port=endeff_publish_port,
            endeff_subscribe_port=endeff_subscribe_port,
            moving_average_limit=moving_average_limit,
            h_r_v=H_R_V_RIGHT,
            h_t_v=H_T_V_RIGHT,
            use_filter=use_filter,
            arm_resolution_port=arm_resolution_port,
            teleoperation_state_port=teleoperation_state_port,
            logging_config=logging_config,
            hand_side=robots.RIGHT,
        )


class DobotCR5LeftOperator(XArmOperator):
    def __init__(
        self,
        host: str,
        transformed_keypoints_port: int,
        stream_configs: Dict[str, Any],
        stream_oculus: bool,
        endeff_publish_port: int,
        endeff_subscribe_port: int,
        moving_average_limit: int,
        use_filter: bool = True,
        arm_resolution_port: Optional[int] = None,
        teleoperation_state_port: Optional[int] = None,
        logging_config: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(
            operator_name="dobot_cr5_left_operator",
            host=host,
            transformed_keypoints_port=transformed_keypoints_port,
            stream_configs=stream_configs,
            stream_oculus=stream_oculus,
            endeff_publish_port=endeff_publish_port,
            endeff_subscribe_port=endeff_subscribe_port,
            moving_average_limit=moving_average_limit,
            h_r_v=H_R_V_LEFT,
            h_t_v=H_T_V_LEFT,
            use_filter=use_filter,
            arm_resolution_port=arm_resolution_port,
            teleoperation_state_port=teleoperation_state_port,
            logging_config=logging_config,
            hand_side=robots.LEFT,
        )
