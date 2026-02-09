import csv
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from dynamixel_y import DynamixelY, scan
from keysight_5322A import Keysight5322A

SERIAL_NUMBER = "SNNX13B3010044"

FILE_NAME = f"{Path(__file__).stem}_{SERIAL_NUMBER}.csv"

if input("Collect data? (Y/N)").lower() == "y":
    GATE_TIME = 30

    keysight = Keysight5322A(GATE_TIME)

    dynamixel_y = DynamixelY(scan())

    with open(FILE_NAME, mode="w", newline="") as file:
        csv_writer = csv.writer(file)

        csv_writer.writerow(("Goal velocity (deg/s)", "Present velocity (deg/s)", "Measured velocity (deg/s)"))

        for abs_goal_velocity in np.arange(90, 230, 0.6):  # goal velocity must be in increments of 0.01 RPM (0.6 deg/s)
            for sign in (-1, 1):
                goal_velocity = abs_goal_velocity * sign

                print(f"{goal_velocity} deg/s")

                goal_velocity = dynamixel_y.set_velocity(goal_velocity)

                keysight.trigger()

                timeout = time.perf_counter() + GATE_TIME

                accumulator = 0
                number_of_samples = 0

                while time.perf_counter() < timeout:
                    accumulator += dynamixel_y.get_velocity()
                    number_of_samples += 1

                present_velocity = accumulator / number_of_samples

                measured_velocity = sign * 2 * keysight.fetch()  # 180 PPR optical encoder

                csv_writer.writerow((goal_velocity, present_velocity, measured_velocity))

                file.flush()

    dynamixel_y.set_velocity(0)
    dynamixel_y.set_position(0, block=False)


data = np.genfromtxt(FILE_NAME, delimiter=",", skip_header=1)

data = data[data[:, 0].argsort()]

goal_velocity = data[:, 0]
present_velocity = data[:, 1]
measured_velocity = data[:, 2]

goal_error = 1e6 * ((measured_velocity - goal_velocity) / goal_velocity)
present_error = 1e6 * ((present_velocity - measured_velocity) / measured_velocity)

plt.plot(goal_velocity, goal_error, "x", label="Goal velocity")
plt.plot(goal_velocity, present_error, "x", label="Present velocity")

plt.title(f"Velocity error vs. goal velocity ({SERIAL_NUMBER})")
plt.xlabel("Goal velocity (°/s)")
plt.ylabel("Velocity error (ppm)")
plt.legend()
plt.grid()

plt.show()
