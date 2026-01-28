import csv
import time
from typing import Any, Callable

import matplotlib.pyplot as plt
import numpy as np
import pyvisa  # pip install pyvisa-py
from dynamixel_y import DynamixelY


class Keysight5322A:
    def __init__(self, gate_time: float) -> None:
        self.__session = pyvisa.ResourceManager("@py").open_resource("TCPIP::192.168.1.53::5025::SOCKET")
        self.__session.read_termination = "\n"

        self.__session.write("*RST")
        self.__session.write("INP1:FILT ON")
        self.__session.write(f"SENS:FREQ:GATE:TIME {gate_time}")

    def trigger(self) -> None:
        self.__session.write("INIT:IMM")

    def fetch(self) -> float:
        return float(self.__session.query("FETCH?").strip())


def retry(function: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
    while True:
        try:
            return function(*args, **kwargs)
        except Exception as error:
            print(f"Retry after: {error}")


FILE_NAME = "test_velocity.csv"

if input("Collect data? (Y/N)") in ("y", "Y"):
    GATE_TIME = 30

    keysight = Keysight5322A(GATE_TIME)

    dynamixel_y = DynamixelY(DynamixelY.scan())

    with open(FILE_NAME, mode="w", newline="") as file:
        csv_writer = csv.writer(file)

        csv_writer.writerow(("Goal velocity (deg/s)", "Present velocity (deg/s)", "Measured velocity (deg/s)"))

        for abs_goal_velocity in np.arange(90, 180, 0.6):  # goal velocity must be in increments of 0.01 RPM (0.6 degrees per second)
            for sign in (-1, 1):
                goal_velocity = abs_goal_velocity * sign

                for trial in range(1):
                    print(f"goal_velocity = {goal_velocity} deg/s, trial = {trial}")

                    goal_velocity = retry(dynamixel_y.set_velocity, goal_velocity)

                    keysight.trigger()

                    timeout = time.perf_counter() + GATE_TIME

                    accumulator = 0
                    number_of_samples = 0

                    while time.perf_counter() < timeout:
                        accumulator += retry(dynamixel_y.get_velocity)
                        number_of_samples += 1

                    present_velocity = accumulator / number_of_samples

                    measured_velocity = sign * 2 * keysight.fetch()  # 180 PPR optical encoder

                    csv_writer.writerow((goal_velocity, present_velocity, measured_velocity))

                    file.flush()

    retry(dynamixel_y.set_velocity, 0)
    retry(dynamixel_y.set_position, 0, False)


data = np.genfromtxt(FILE_NAME, delimiter=",", skip_header=1)

data = data[data[:, 0].argsort()]

goal_velocity = data[:, 0]
present_velocity = data[:, 1]
measured_velocity = data[:, 2]

goal_error = 1e6 * ((measured_velocity - goal_velocity) / goal_velocity)
present_error = 1e6 * ((present_velocity - measured_velocity) / measured_velocity)

plt.plot(goal_velocity, goal_error, "x", label="Goal")
plt.plot(goal_velocity, present_error, "x", label="Present")

plt.title("YM080-230-R051-RH velocity accuracy")
plt.xlabel("Velocity (deg/s)")
plt.ylabel("Error (ppm)")
plt.legend()
plt.grid()

plt.show()
