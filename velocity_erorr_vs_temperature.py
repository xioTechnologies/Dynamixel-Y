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

    plt.ion()

    fig, ax_error = plt.subplots()

    ABS_GOAL_VELOCITY = 180

    (positive_error_line,) = ax_error.plot([], label=f"Goal velocity = +{ABS_GOAL_VELOCITY}°/s")
    (negative_error_line,) = ax_error.plot([], label=f"Goal velocity = -{ABS_GOAL_VELOCITY}°/s")

    ax_error.set_title(f"Velocity error and temperature vs. time ({SERIAL_NUMBER})")
    ax_error.set_xlabel("Time (seconds)")
    ax_error.set_ylabel("Velocity error (ppm)")
    ax_error.legend(loc="upper left")
    ax_error.grid()

    ax_temperature = ax_error.twinx()

    (temp_line,) = ax_temperature.plot([], [], color="k", label="Motor temperature")

    ax_temperature.set_ylabel("Motor temperature (°C)")
    ax_temperature.legend(loc="upper right")

    positive_seconds_list = []
    positive_error_list = []
    negative_seconds_list = []
    negative_error_list = []

    temperature_seconds_list = []
    temperature_list = []

    with open(FILE_NAME, mode="w", newline="") as file:
        csv_writer = csv.writer(file)

        csv_writer.writerow(("Time (seconds)", "Goal velocity (deg/s)", "Measured velocity (deg/s)", "Motor temperature (degC)"))

        start = time.perf_counter()

        while plt.fignum_exists(fig.number):
            for sign in (-1, 1):
                seconds = time.perf_counter() - start

                motor_temperature = dynamixel_y.get_motor_temperature()

                goal_velocity = dynamixel_y.set_velocity(sign * ABS_GOAL_VELOCITY)

                keysight.trigger()

                time.sleep(GATE_TIME)

                measured_velocity = sign * 2 * keysight.fetch()  # 180 PPR optical encoder

                csv_writer.writerow((seconds, goal_velocity, measured_velocity, motor_temperature))

                file.flush()

                error = 1e6 * ((measured_velocity - goal_velocity) / goal_velocity)

                if sign > 0:
                    positive_seconds_list.append(seconds)
                    positive_error_list.append(error)

                    positive_error_line.set_data(positive_seconds_list, positive_error_list)
                else:
                    negative_seconds_list.append(seconds)
                    negative_error_list.append(error)

                    negative_error_line.set_data(negative_seconds_list, negative_error_list)

                temperature_seconds_list.append(seconds)
                temperature_list.append(motor_temperature)

                temp_line.set_data(temperature_seconds_list, temperature_list)

                ax_error.relim()
                ax_error.autoscale_view()

                ax_temperature.relim()
                ax_temperature.autoscale_view()

                fig.canvas.flush_events()

    dynamixel_y.set_velocity(0)
    dynamixel_y.set_position(0, block=False)


data = np.genfromtxt(FILE_NAME, delimiter=",", skip_header=1)

goal_velocity = data[:, 1]
measured_velocity = data[:, 2]
temperature = data[:, 3]

error = 1e6 * ((measured_velocity - goal_velocity) / goal_velocity)

positive_indices = goal_velocity >= 0
negative_indices = np.logical_not(positive_indices)

plt.plot(temperature[positive_indices], error[positive_indices], "x", label=f"Goal velocity = {goal_velocity[positive_indices][0]}°/s")
plt.plot(temperature[negative_indices], error[negative_indices], "x", label=f"Goal velocity = {goal_velocity[negative_indices][0]}°/s")

plt.title(f"Velocity error vs. temperature ({SERIAL_NUMBER})")
plt.xlabel("Motor temperature (°C)")
plt.ylabel("Velocity error (ppm)")
plt.legend()
plt.grid()

plt.show()
