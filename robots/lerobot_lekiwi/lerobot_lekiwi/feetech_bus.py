"""Narrow STS3215 bus implementation for the nine-motor LeKiwi chain."""

from __future__ import annotations

import platform
import select
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MethodType
from typing import Any


BAUDRATE = 1_000_000
MODEL_NUMBER = 777
RESOLUTION = 4096
SIGN_BITS = {"Homing_Offset": 11, "Goal_Velocity": 15, "Present_Velocity": 15}
REGISTERS = {
    "Return_Delay_Time": (7, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Velocity": (46, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Maximum_Acceleration": (85, 1),
}


@dataclass(frozen=True)
class MotorCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


class FeetechBus:
    """The minimal SDK surface LeKiwi needs; fixed to STS3215 protocol 0."""

    def __init__(
        self,
        port: str,
        motors: Mapping[str, int],
        calibration: Mapping[str, MotorCalibration] | None = None,
    ) -> None:
        if len(set(motors.values())) != len(motors):
            raise ValueError("LeKiwi motor IDs must be unique")
        self.port = str(port)
        self.motors = dict(motors)
        self.calibration = dict(calibration or {})
        self._connected = False
        self._sdk: Any = None
        self.port_handler: Any = None
        self.packet_handler: Any = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            raise RuntimeError(f"Feetech bus on {self.port} is already connected")
        import scservo_sdk as scs

        self._sdk = scs
        self.port_handler = scs.PortHandler(self.port)
        self.port_handler.setPacketTimeout = MethodType(_patched_set_packet_timeout, self.port_handler)
        self.packet_handler = scs.PacketHandler(0)
        if not self.port_handler.openPort():
            raise ConnectionError(f"Could not open LeKiwi serial port {self.port}")
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.port_handler.closePort()
            raise ConnectionError(f"Could not set {self.port} to {BAUDRATE} baud")
        self._connected = True
        try:
            self._check_motors()
        except Exception:
            self.port_handler.closePort()
            self._connected = False
            raise

    def disconnect(self, *, disable_torque: bool = True) -> None:
        if not self._connected:
            return
        if disable_torque:
            try:
                self.disable_torque(retries=5)
            except Exception:
                pass
        self.port_handler.closePort()
        self._connected = False

    def _check_motors(self) -> None:
        missing: list[str] = []
        wrong: list[str] = []
        for name, motor_id in self.motors.items():
            model, comm, error = self.packet_handler.ping(self.port_handler, motor_id)
            if comm != self._sdk.COMM_SUCCESS:
                missing.append(f"{name}(id={motor_id})")
            elif error != 0 or model != MODEL_NUMBER:
                wrong.append(f"{name}(id={motor_id}, model={model}, error={error})")
        if missing or wrong:
            details = []
            if missing:
                details.append("missing: " + ", ".join(missing))
            if wrong:
                details.append("unexpected: " + ", ".join(wrong))
            raise RuntimeError(f"LeKiwi motor check failed on {self.port}: {'; '.join(details)}")

    def configure(self, arm_motors: Sequence[str], base_motors: Sequence[str]) -> None:
        self.disable_torque()
        for motor in self.motors:
            self.write("Return_Delay_Time", motor, 0)
            self.write("Maximum_Acceleration", motor, 254)
            self.write("Acceleration", motor, 254)
        for motor in arm_motors:
            self.write("Operating_Mode", motor, 0)
            self.write("P_Coefficient", motor, 16)
            self.write("I_Coefficient", motor, 0)
            self.write("D_Coefficient", motor, 32)
        for motor in base_motors:
            self.write("Operating_Mode", motor, 1)
        self.enable_torque()

    def disable_torque(self, motors: Sequence[str] | None = None, *, retries: int = 0) -> None:
        for motor in motors or tuple(self.motors):
            self.write("Torque_Enable", motor, 0, retries=retries)
            self.write("Lock", motor, 0, retries=retries)

    def enable_torque(self, motors: Sequence[str] | None = None, *, retries: int = 0) -> None:
        for motor in motors or tuple(self.motors):
            self.write("Torque_Enable", motor, 1, retries=retries)
            self.write("Lock", motor, 1, retries=retries)

    def read(self, register: str, motor: str, *, retries: int = 0) -> int:
        self._require_connected()
        address, length = _register(register)
        motor_id = self.motors[motor]
        fn = {1: self.packet_handler.read1ByteTxRx, 2: self.packet_handler.read2ByteTxRx}[length]
        for _ in range(retries + 1):
            value, comm, error = fn(self.port_handler, motor_id, address)
            if comm == self._sdk.COMM_SUCCESS:
                break
        self._check_result(comm, error, f"read {register} from {motor}(id={motor_id})")
        return _decode_signed(register, int(value))

    def write(self, register: str, motor: str, value: int | float, *, retries: int = 0) -> None:
        self._require_connected()
        address, length = _register(register)
        motor_id = self.motors[motor]
        encoded = _encode_signed(register, int(round(value)))
        fn = {1: self.packet_handler.write1ByteTxRx, 2: self.packet_handler.write2ByteTxRx}[length]
        for _ in range(retries + 1):
            comm, error = fn(self.port_handler, motor_id, address, encoded)
            if comm == self._sdk.COMM_SUCCESS:
                break
        self._check_result(comm, error, f"write {register} to {motor}(id={motor_id})")

    def sync_read(self, register: str, motors: Sequence[str]) -> dict[str, int]:
        self._require_connected()
        names = list(motors)
        address, length = _register(register)
        reader = self._sdk.GroupSyncRead(self.port_handler, self.packet_handler, address, length)
        try:
            for name in names:
                if not reader.addParam(self.motors[name]):
                    raise RuntimeError(f"Could not add {name} to sync read")
            comm = reader.txRxPacket()
            if comm != self._sdk.COMM_SUCCESS:
                raise ConnectionError(
                    f"LeKiwi sync read {register} failed: {self.packet_handler.getTxRxResult(comm)}"
                )
            result = {}
            for name in names:
                motor_id = self.motors[name]
                if not reader.isAvailable(motor_id, address, length):
                    raise ConnectionError(f"LeKiwi sync read {register} returned no data for {name}")
                result[name] = _decode_signed(register, int(reader.getData(motor_id, address, length)))
            return result
        finally:
            reader.clearParam()

    def sync_write(
        self,
        register: str,
        values: Mapping[str, int | float],
        *,
        retries: int = 0,
    ) -> None:
        self._require_connected()
        address, length = _register(register)
        writer = self._sdk.GroupSyncWrite(self.port_handler, self.packet_handler, address, length)
        try:
            for name, value in values.items():
                encoded = _encode_signed(register, int(round(value)))
                if not writer.addParam(self.motors[name], _serialize(self._sdk, encoded, length)):
                    raise RuntimeError(f"Could not add {name} to sync write")
            for _ in range(max(0, int(retries)) + 1):
                comm = writer.txPacket()
                if comm == self._sdk.COMM_SUCCESS:
                    break
            if comm != self._sdk.COMM_SUCCESS:
                raise ConnectionError(
                    f"LeKiwi sync write {register} failed: {self.packet_handler.getTxRxResult(comm)}"
                )
        finally:
            writer.clearParam()

    def read_arm_degrees_and_percent(self, arm_motors: Sequence[str]) -> dict[str, float]:
        raw = self.sync_read("Present_Position", arm_motors)
        return {name: self._normalize(name, value) for name, value in raw.items()}

    def write_arm_degrees_and_percent(self, values: Mapping[str, float]) -> None:
        self.sync_write("Goal_Position", {name: self._unnormalize(name, value) for name, value in values.items()})

    def set_half_turn_homings(self, motors: Sequence[str]) -> dict[str, int]:
        for motor in motors:
            self.write("Homing_Offset", motor, 0)
            self.write("Min_Position_Limit", motor, 0)
            self.write("Max_Position_Limit", motor, RESOLUTION - 1)
        positions = self.sync_read("Present_Position", motors)
        offsets = {motor: position - (RESOLUTION - 1) // 2 for motor, position in positions.items()}
        for motor, offset in offsets.items():
            self.write("Homing_Offset", motor, offset)
        return offsets

    def record_ranges(self, motors: Sequence[str], *, poll_s: float = 0.02) -> tuple[dict[str, int], dict[str, int]]:
        names = list(motors)
        positions = self.sync_read("Present_Position", names)
        minimums = dict(positions)
        maximums = dict(positions)
        print("Move the listed joints through their full ranges, then press ENTER.")
        while not _enter_pressed():
            positions = self.sync_read("Present_Position", names)
            minimums = {name: min(minimums[name], positions[name]) for name in names}
            maximums = {name: max(maximums[name], positions[name]) for name in names}
            status = "  ".join(f"{name}: {minimums[name]}..{maximums[name]}" for name in names)
            print("\r" + status[:180].ljust(180), end="", flush=True)
            time.sleep(poll_s)
        print()
        unmoved = [name for name in names if minimums[name] == maximums[name]]
        if unmoved:
            raise ValueError(f"No range was recorded for: {', '.join(unmoved)}")
        return minimums, maximums

    def write_calibration(self, calibration: Mapping[str, MotorCalibration]) -> None:
        self.calibration = dict(calibration)
        for name, item in self.calibration.items():
            self.write("Homing_Offset", name, item.homing_offset)
            self.write("Min_Position_Limit", name, item.range_min)
            self.write("Max_Position_Limit", name, item.range_max)

    def _normalize(self, motor: str, raw: int) -> float:
        item = self.calibration.get(motor)
        if item is None or item.range_min == item.range_max:
            raise RuntimeError(f"Missing or invalid LeKiwi calibration for {motor}")
        if motor == "arm_gripper":
            bounded = min(item.range_max, max(item.range_min, raw))
            percent = (bounded - item.range_min) / (item.range_max - item.range_min) * 100.0
            return 100.0 - percent if item.drive_mode else percent
        midpoint = (item.range_min + item.range_max) / 2.0
        return (raw - midpoint) * 360.0 / (RESOLUTION - 1)

    def _unnormalize(self, motor: str, value: float) -> int:
        item = self.calibration.get(motor)
        if item is None or item.range_min == item.range_max:
            raise RuntimeError(f"Missing or invalid LeKiwi calibration for {motor}")
        if motor == "arm_gripper":
            percent = min(100.0, max(0.0, float(value)))
            if item.drive_mode:
                percent = 100.0 - percent
            return int(percent / 100.0 * (item.range_max - item.range_min) + item.range_min)
        midpoint = (item.range_min + item.range_max) / 2.0
        raw = int(float(value) * (RESOLUTION - 1) / 360.0 + midpoint)
        return min(item.range_max, max(item.range_min, raw))

    def _check_result(self, comm: int, error: int, operation: str) -> None:
        if comm != self._sdk.COMM_SUCCESS:
            raise ConnectionError(f"LeKiwi failed to {operation}: {self.packet_handler.getTxRxResult(comm)}")
        if error != 0:
            raise RuntimeError(f"LeKiwi failed to {operation}: {self.packet_handler.getRxPacketError(error)}")

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError(f"Feetech bus on {self.port} is not connected")


def _register(name: str) -> tuple[int, int]:
    try:
        return REGISTERS[name]
    except KeyError as exc:
        raise KeyError(f"Unsupported STS3215 register: {name}") from exc


def _encode_signed(register: str, value: int) -> int:
    sign_bit = SIGN_BITS.get(register)
    if sign_bit is None:
        return value
    magnitude = abs(value)
    limit = (1 << sign_bit) - 1
    if magnitude > limit:
        raise ValueError(f"{register} magnitude {magnitude} exceeds {limit}")
    return magnitude | ((1 << sign_bit) if value < 0 else 0)


def _decode_signed(register: str, value: int) -> int:
    sign_bit = SIGN_BITS.get(register)
    if sign_bit is None:
        return value
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


def _serialize(sdk: Any, value: int, length: int) -> list[int]:
    if length == 1:
        return [value]
    if length == 2:
        return [sdk.SCS_LOBYTE(value), sdk.SCS_HIBYTE(value)]
    raise ValueError(f"Unsupported register length: {length}")


def _patched_set_packet_timeout(port_handler: Any, packet_length: int) -> None:
    port_handler.packet_start_time = port_handler.getCurrentTime()
    port_handler.packet_timeout = (
        port_handler.tx_time_per_byte * packet_length
        + port_handler.tx_time_per_byte * 3.0
        + 50
    )


def _enter_pressed() -> bool:
    if platform.system() == "Windows":
        import msvcrt

        return bool(msvcrt.kbhit() and msvcrt.getch() in (b"\r", b"\n"))
    return bool(select.select([sys.stdin], [], [], 0)[0]) and sys.stdin.readline().strip() == ""
