import math
from typing import cast

import pyenki


def log(thymio: pyenki.Thymio2, time: float) -> None:
    events = thymio.prox_comm_events
    if not events:
        return
    print(f"At time {time:.1f}, Thymio {thymio.name} received msgs")
    for e in events:
        print(
            f"- value: {e.rx_value}, payloads: {e.payloads}, intensities: {e.intensities}"
        )


def main() -> None:
    dt = 0.1
    world = pyenki.World()
    for i, (x, theta, tx) in enumerate(
            zip((100, 115, 130), (0, math.pi, 0), (111, 222, 333),
                strict=True)):
        thymio = pyenki.Thymio2()
        thymio.name = f"#{i}"
        thymio.position = (x, 100)
        thymio.angle = theta
        world.add_object(thymio)
        thymio.prox_comm_enabled = True
        thymio.prox_comm_tx = tx
    print("Start Simulation")
    for i in range(15):
        world.step(dt)
        for robot in world.robots:
            log(cast('pyenki.Thymio2', robot), dt * i)
    print("End Simulation")


if __name__ == '__main__':
    main()
