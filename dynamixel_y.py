import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from functools import wraps
from typing import Any

import colorama
import dynamixel_sdk
import serial.tools.list_ports  # pip install pyserial

# Part number: YM080-230-R051-RH
# https://emanual.robotis.com/docs/en/dxl/y/ym080-230-r051-rh/

# Configure USB latency setting
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting

# Use DYNAMIXEL Wizard 2.0 to factory reset and then configure:
# - Baud Rate = 1 Mbps
# - Max Position Limit = 1347440720
# - Min Position Limit = -1347440720


@dataclass(frozen=True)
class _Register:
    name: str
    address: int
    size: int


_OPERATING_MODE = _Register("OPERATING_MODE", 33, 1)

_VELCOITY_LIMIT = _Register("_VELCOITY_LIMIT", 72, 4)

_PROFILE_VELCOITY = _Register("_PROFILE_VELCOITY", 244, 4)

_CONTROLLER_STATE = _Register("_CONTROLLER_STATE", 152, 1)

_TORQUE_ENABLE = _Register("TORQUE_ENABLE", 512, 1)

_GOAL_VELOCITY = _Register("GOAL_VELOCITY", 528, 4)

_GOAL_POSITION = _Register("GOAL_POSITION", 532, 4)

_PRESENT_CURRENT = _Register("PRESENT_CURRENT", 546, 2)

_PRESENT_VELOCITY = _Register("PRESENT_VELOCITY", 548, 4)

_PRESENT_POSITION = _Register("PRESENT_POSITION", 552, 4)

_PRESENT_INPUT_VOLTAGE = _Register("PRESENT_INPUT_VOLTAGE", 568, 2)

_PRESENT_INVERTER_TEMPERATURE = _Register("PRESENT_INVERTER_TEMPERATURE", 570, 1)

_PRESENT_MOTOR_TEMPERATURE = _Register("PRESENT_MOTOR_TEMPERATURE", 571, 1)


class _OperatingMode(Enum):
    VELOCITY = 1
    POSITION = 3


_debug_callback = None


def set_debug_callback(callback: Callable[[str], None] | None) -> None:
    global _debug_callback
    _debug_callback = callback


def _debug_print(message: Any) -> None:
    if _debug_callback:
        _debug_callback(str(message))


def _retry(func: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(func)
    def wrapper(*args, **kwargs):
        timeout = time.perf_counter() + 1

        while True:
            try:
                return func(*args, **kwargs)

            except Exception as exception:
                if time.perf_counter() >= timeout:
                    raise

                _debug_print(exception)

    return wrapper


class DynamixelY:
    __CRPMS_PER_DEGREES_PER_SECOND = 1 / 0.06  # CRPM = 0.01 rev/min

    __PULSES_PER_DEGREE = 524_288 / 360

    def __init__(self, port_name: str) -> None:
        self.__ID = 1

        self.__port = dynamixel_sdk.PortHandler(port_name)

        self.__packet = dynamixel_sdk.PacketHandler(2.0)

        if not self.__port.openPort():
            raise Exception("Unable to open port")

        if not self.__port.setBaudRate(1_000_000):
            raise Exception("Unable to set baud rate")

        _, result, error = self.__packet.ping(self.__port, self.__ID)

        self.__check_response(f"Ping failed for {self.__port.getPortName()} at {self.__port.getBaudRate()} baud", result, error)

        self.__torque_enable()

    def close(self) -> None:
        self.__torque_disable()

        self.__port.closePort()

    def get_position(self) -> float:
        return self.__read(_PRESENT_POSITION) / DynamixelY.__PULSES_PER_DEGREE

    def set_position(self, degrees: float, degrees_per_second: float | None = None, block: bool = True) -> float:
        if not degrees_per_second:
            self.__write(_PROFILE_VELCOITY, self.__read(_VELCOITY_LIMIT))
        else:
            self.__write(_PROFILE_VELCOITY, round(abs(degrees_per_second) * DynamixelY.__CRPMS_PER_DEGREES_PER_SECOND))

        self.__set_operating_mode(_OperatingMode.POSITION)

        goal_position = round(degrees * DynamixelY.__PULSES_PER_DEGREE)

        degrees = goal_position / DynamixelY.__PULSES_PER_DEGREE

        self.__write(_GOAL_POSITION, goal_position)

        if not block:
            return degrees

        while True:
            present_position = self.__read(_PRESENT_POSITION)

            _debug_print(f"{present_position / DynamixelY.__PULSES_PER_DEGREE} deg")

            if present_position == goal_position:
                break

        return degrees

    def get_velocity(self) -> float:
        return self.__read(_PRESENT_VELOCITY) / DynamixelY.__CRPMS_PER_DEGREES_PER_SECOND

    def set_velocity(self, degrees_per_second: float, block: bool = True) -> float:
        self.__set_operating_mode(_OperatingMode.VELOCITY)

        goal_velocity = round(degrees_per_second * DynamixelY.__CRPMS_PER_DEGREES_PER_SECOND)

        degrees_per_second = goal_velocity / DynamixelY.__CRPMS_PER_DEGREES_PER_SECOND

        self.__write(_GOAL_VELOCITY, goal_velocity)

        if not block:
            return degrees_per_second

        velocity_increased = goal_velocity > self.__read(_PRESENT_VELOCITY)

        while True:
            present_velocity = self.__read(_PRESENT_VELOCITY)

            _debug_print(f"{present_velocity / DynamixelY.__CRPMS_PER_DEGREES_PER_SECOND} deg/s")

            if velocity_increased:
                if present_velocity >= goal_velocity:
                    break
            else:
                if present_velocity <= goal_velocity:
                    break

        return degrees_per_second

    def get_current(self) -> float:
        return self.__read(_PRESENT_CURRENT) / 100

    def get_voltage(self) -> float:
        return self.__read(_PRESENT_INPUT_VOLTAGE) / 10

    def get_inverter_temperature(self) -> int:
        return self.__read(_PRESENT_INVERTER_TEMPERATURE)

    def get_motor_temperature(self) -> int:
        return self.__read(_PRESENT_MOTOR_TEMPERATURE)

    def __torque_enable(self) -> None:
        self.__write(_TORQUE_ENABLE, 1)

        self.__wait_for_processing_torque()

    def __torque_disable(self) -> None:
        self.__write(_TORQUE_ENABLE, 0)

        self.__wait_for_processing_torque()

    def __wait_for_processing_torque(self) -> None:
        PROCESSING_TORQUE_ON = 4
        PROCESSING_TORQUE_OFF = 6

        while self.__read(_CONTROLLER_STATE) in (PROCESSING_TORQUE_ON, PROCESSING_TORQUE_OFF):
            pass

    def __set_operating_mode(self, mode: _OperatingMode) -> None:
        if self.__read(_OPERATING_MODE) != mode.value:
            self.__torque_disable()

            self.__write(_OPERATING_MODE, mode.value)

        if self.__read(_TORQUE_ENABLE) == 0:
            self.__torque_enable()

    @_retry
    def __read(self, register: _Register) -> int:
        match register.size:
            case 1:
                read = self.__packet.read1ByteTxRx
            case 2:
                read = self.__packet.read2ByteTxRx
            case 4:
                read = self.__packet.read4ByteTxRx

        value, result, error = read(self.__port, self.__ID, register.address)

        self.__check_response(f"Read failed for {register.name}", result, error)

        max_unsigned_plus_one = 1 << (register.size * 8)

        return value if value < (max_unsigned_plus_one // 2) else value - max_unsigned_plus_one  # convert to signed 8/16/32-bit

    @_retry
    def __write(self, register: _Register, value: int) -> None:
        match register.size:
            case 1:
                write = self.__packet.write1ByteTxRx
            case 2:
                write = self.__packet.write2ByteTxRx
            case 4:
                write = self.__packet.write4ByteTxRx

        result, error = write(self.__port, self.__ID, register.address, value)

        self.__check_response(f"Write failed for {register.name}, {value}", result, error)

    def __check_response(self, message: str, result: int, error: int) -> None:
        if result != dynamixel_sdk.COMM_SUCCESS:
            raise Exception(f"{message}. {self.__packet.getTxRxResult(result)}")

        if error != 0:
            raise Exception(f"{message}. {self.__packet.getRxPacketError(error)}")


def scan() -> str:
    port_names = [p.name for p in serial.tools.list_ports.comports()]

    for port_name in port_names:
        try:
            DynamixelY(port_name).close()

            return port_name

        except Exception:
            pass

    raise Exception("DYNAMIXEL-Y not found")


def _main() -> None:
    colorama.init(autoreset=True)

    set_debug_callback(lambda message: print(f"{colorama.Fore.LIGHTBLACK_EX}{message}"))

    dynamixel_y = DynamixelY(scan())

    @dataclass(frozen=True)
    class Command:
        name: str
        method: Callable[[float], None]

    commands = (
        Command("Position", dynamixel_y.set_position),
        Command("Velocity", dynamixel_y.set_velocity),
    )

    while True:
        position = dynamixel_y.get_position()
        velocity = dynamixel_y.get_velocity()
        current = dynamixel_y.get_current()
        voltage = dynamixel_y.get_voltage()
        inverter_temperature = dynamixel_y.get_inverter_temperature()
        motor_temperature = dynamixel_y.get_motor_temperature()

        print(f"{position:.3f} deg, {velocity:.3f} deg/s, {current:.2f} A, {voltage:.1f} V, {inverter_temperature} degC, {motor_temperature} degC")

        print("Select command")

        for index, command in enumerate(commands):
            print(f"{index}. {command.name}")

        try:
            command = commands[int(input())]

            print("Enter value")

            command.method(float(input()))

            print(f"{colorama.Fore.GREEN}Complete")

        except Exception as exception:
            print(f"{colorama.Fore.RED}{exception}")
            continue


if __name__ == "__main__":
    _main()
