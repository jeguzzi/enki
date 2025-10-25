import math
import sys

import pyenki


# We sublass `pyenki.Thymio2`. This way, the world update step will automatically call
# also the Thymio `controlStep`.
class ControlledThymio2(pyenki.Thymio2):

    # This is the method we have to overwrite
    def controlStep(self, dt: float) -> None:
        # Check if there is an obstacle in front of us
        value = self.prox_values[2]
        if value > 3000:
            # A lot of light is reflected => there is an obstacle,
            # so we stop and switch the LED to red
            speed = 0.0
            self.set_led_top(red=1.0)
        else:
            speed = 10.0
            self.set_led_top(green=1.0)
        self.motor_left_target = speed
        self.motor_right_target = speed


def setup() -> pyenki.World:
    # We create an unbounded world
    world = pyenki.World()
    # We add a Thymio at the origin,
    # which can optionally use Aseba-like units and types.
    thymio = ControlledThymio2()
    thymio.position = (0, 0)
    thymio.angle = 0
    world.add_object(thymio)
    # and a wall a bit in forward, in front of the Thymio.
    wall = pyenki.RectangularObject(l1=10,
                                    l2=50,
                                    height=5,
                                    mass=1,
                                    color=pyenki.Color(0.5, 0.3, 0.3))
    wall.position = (30, 0)
    world.add_object(wall)
    return world


def run(world: pyenki.World,
        gui: bool = False,
        T: float = 10,
        dt: float = 0.1,
        orthographic: bool = False) -> None:

    if gui:
        # We can either run a simulation [in real-time] inside a Qt application
        world.run_in_viewer(camera_position=(0, 0),
                            camera_altitude=70.0,
                            camera_yaw=0.0,
                            camera_pitch=-math.pi / 2,
                            walls_height=10,
                            camera_is_ortho=orthographic)
    else:
        # or we can write our own loop that run the simulaion as fast as possible.
        steps = int(T // dt)
        for _ in range(steps):
            world.step(dt)


if __name__ == '__main__':
    world = setup()
    run(world,
        gui='--gui' in sys.argv,
        orthographic='--orthographic' in sys.argv)
