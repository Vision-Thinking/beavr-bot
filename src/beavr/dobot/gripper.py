"""DH-Robotics AG-95 gripper control through the Dobot Dashboard Tool485 path."""

from __future__ import annotations

import time
from typing import Callable

from .dashboard import DashboardClient, require_success

INIT_COMMAND_ADDR = 256
FORCE_ADDR = 257
POSITION_ADDR = 259
INIT_STATUS_ADDR = 512
GRIP_STATUS_ADDR = 513
CURRENT_POSITION_ADDR = 514

MIN_FORCE, MAX_FORCE = 20, 100
MIN_POSITION, MAX_POSITION = 0, 1000
INIT_COMMAND_VALUE = 165
INIT_STATUS_INITIALIZED = 1
INIT_TIMEOUT_S = 5.0
SESSION_ATTEMPTS = 2
SESSION_RETRY_DELAY_S = 0.25


def _parse_bracket_value(reply: str) -> str:
    start = reply.find("{")
    end = reply.find("}", start)
    if start < 0 or end < 0:
        raise ValueError(f"Cannot parse Dashboard reply: {reply!r}")
    return reply[start + 1 : end]


def _send_success(client: DashboardClient, command: str) -> str:
    reply = client.send(command)
    require_success(command, reply)
    return reply


def prepare_native_gripper(
    client: DashboardClient,
    *,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    _send_success(client, "SetToolPower(1)")
    sleep(0.8)
    _send_success(client, "SetToolMode(1)")
    _send_success(client, "SetTool485(115200,N,1,1)")
    reply = _send_success(client, "ModbusCreate(127.0.0.1,60000,1,1)")
    return int(_parse_bracket_value(reply))


def read_register(client: DashboardClient, idx: int, address: int) -> int:
    reply = _send_success(client, f"GetHoldRegs({idx},{address},1,U16)")
    return int(_parse_bracket_value(reply))


def write_register(client: DashboardClient, idx: int, address: int, value: int) -> None:
    _send_success(client, f"SetHoldRegs({idx},{address},1,{{{value}}},U16)")


def close_native_gripper(client: DashboardClient, idx: int) -> None:
    _send_success(client, f"ModbusClose({idx})")


def ensure_initialized(
    client: DashboardClient,
    idx: int,
    *,
    timeout_s: float = INIT_TIMEOUT_S,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    status = read_register(client, idx, INIT_STATUS_ADDR)
    if status == INIT_STATUS_INITIALIZED:
        return

    write_register(client, idx, INIT_COMMAND_ADDR, INIT_COMMAND_VALUE)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        sleep(0.3)
        status = read_register(client, idx, INIT_STATUS_ADDR)
        if status == INIT_STATUS_INITIALIZED:
            return
    raise RuntimeError(f"Gripper initialization timed out; final init_status={status}")


def _set_gripper_once(
    client: DashboardClient,
    position: int,
    force: int,
    *,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    idx = prepare_native_gripper(client, sleep=sleep)
    try:
        ensure_initialized(client, idx, sleep=sleep)
        write_register(client, idx, FORCE_ADDR, force)
        write_register(client, idx, POSITION_ADDR, position)
    finally:
        close_native_gripper(client, idx)


def set_gripper(
    client: DashboardClient,
    position: int,
    force: int,
    *,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    if not (MIN_POSITION <= int(position) <= MAX_POSITION):
        raise ValueError(f"position must be in [{MIN_POSITION}, {MAX_POSITION}], got {position}")
    if not (MIN_FORCE <= int(force) <= MAX_FORCE):
        raise ValueError(f"force must be in [{MIN_FORCE}, {MAX_FORCE}], got {force}")

    first_error = None
    for attempt in range(SESSION_ATTEMPTS):
        try:
            _set_gripper_once(client, int(position), int(force), sleep=sleep)
            return
        except RuntimeError as error:
            if attempt + 1 == SESSION_ATTEMPTS:
                raise RuntimeError(
                    f"Gripper Modbus session failed after retry; first error={first_error}; "
                    f"final error={error}"
                ) from error
            first_error = error
            sleep(SESSION_RETRY_DELAY_S)


def read_gripper_state(
    client: DashboardClient,
    *,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[int, int, int]:
    idx = prepare_native_gripper(client, sleep=sleep)
    try:
        init_status = read_register(client, idx, INIT_STATUS_ADDR)
        grip_status = read_register(client, idx, GRIP_STATUS_ADDR)
        position = read_register(client, idx, CURRENT_POSITION_ADDR)
    finally:
        close_native_gripper(client, idx)
    return init_status, grip_status, position
