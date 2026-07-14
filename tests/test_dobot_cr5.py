import numpy as np
import pytest

from beavr.dobot.control import DobotCR5Control
from beavr.dobot.dashboard import DashboardRobot, parse_error_ids, parse_values


def test_parse_error_ids_accepts_literal_empty_list():
    assert parse_error_ids("0,{[]},GetErrorID();") == []


def test_parse_values_rejects_ambiguous_empty_error_payload():
    with pytest.raises(RuntimeError, match="explicit error list"):
        parse_error_ids("0,{},GetErrorID();")


class FakeDashboardClient:
    def __init__(self, host, port=29999, timeout=5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.connected = False
        self.commands = []

    def connect(self):
        self.connected = True

    def close(self):
        self.connected = False

    def send(self, command):
        self.commands.append(command)
        replies = {
            "EnableRobot()": "0,{},EnableRobot();",
            "VelL(10)": "0,{},VelL(10);",
            "AccL(10)": "0,{},AccL(10);",
            "VelJ(10)": "0,{},VelJ(10);",
            "AccJ(10)": "0,{},AccJ(10);",
            "SpeedFactor(100)": "0,{},SpeedFactor(100);",
            "GetErrorID()": "0,{[]},GetErrorID();",
            "RobotMode()": "0,{5},RobotMode();",
            "GetPose()": "0,{100.000,200.000,300.000,0.000,0.000,0.000},GetPose();",
            "RelMovLUser(5.000,0.000,-20.000,0.000,0.000,0.000)": (
                "0,{7936},RelMovLUser(5.000,0.000,-20.000,0.000,0.000,0.000);"
            ),
        }
        return replies[command]


def test_dashboard_robot_sends_relmovluser_in_millimetres():
    robot = DashboardRobot("127.0.0.1", client_factory=FakeDashboardClient)
    robot.connect()

    pose = robot.get_pose()
    robot.move_relative([0.005, 0.0, -0.02])

    assert pose[:3, 3] == pytest.approx([0.1, 0.2, 0.3])
    assert "RelMovLUser(5.000,0.000,-20.000,0.000,0.000,0.000)" in robot.client.commands


def test_dobot_cr5_control_dry_run_limits_each_step():
    control = DobotCR5Control("", dry_run=True, max_step_mm=5.0)
    control.connect()
    current = control.get_arm_cartesian_coords()
    target = current.copy()
    target[0] += 0.020

    moved = control.move_arm_cartesian(target)

    assert moved is True
    assert control.get_arm_pose()[0, 3] == pytest.approx(current[0] + 0.005)


def test_dobot_cr5_control_rejects_workspace_violation():
    control = DobotCR5Control("", dry_run=True, workspace_radius_mm=100.0)
    control.connect()
    current = control.get_arm_cartesian_coords()
    target = current.copy()
    target[:3] += np.array([0.101, 0.0, 0.0], dtype=np.float32)

    with pytest.raises(RuntimeError, match="workspace radius"):
        control.move_arm_cartesian(target)
