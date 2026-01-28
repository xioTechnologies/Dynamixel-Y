import time

from dynamixel_y import DynamixelY

dynamixel_y = DynamixelY(DynamixelY.scan())

number_of_samples = 0
last_print = time.perf_counter()

while True:
    try:
        _ = dynamixel_y.get_velocity()
    except Exception as error:
        print(error)

    number_of_samples += 1
    now = time.perf_counter()

    if (now - last_print) < 1:
        continue

    print(f"{number_of_samples / (now - last_print):.1f} samples/s")

    number_of_samples = 0
    last_print = now
