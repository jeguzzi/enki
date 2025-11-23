import math
import sys
from typing import TYPE_CHECKING, SupportsFloat

import pyenki

if TYPE_CHECKING:
    from pyenki.viewer import CameraConfig


# The world update step will call `control_step` automatically
class ControlledThymio2(pyenki.Thymio2):

    # This is the method we have to overwrite
    # to implement a controller.
    def control_step(self, time_step: SupportsFloat) -> None:
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
        self.left_wheel_target_speed = speed
        self.right_wheel_target_speed = speed


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
    wall = pyenki.PhysicalObject(lx=10,
                                 ly=50,
                                 height=5,
                                 mass=1,
                                 color=pyenki.Color(0.5, 0.3, 0.3))
    wall.position = (30, 0)
    world.add_object(wall)
    return world


def main(duration: float = 10, dt: float = 0.1) -> None:
    gui = '--gui' in sys.argv
    video = '--video' in sys.argv
    ortho = '--ortho' in sys.argv
    world = setup()
    camera_config: CameraConfig = dict(camera_position=(0, 0),
                                       camera_altitude=70.0,
                                       camera_yaw=0.0,
                                       camera_pitch=-math.pi / 2,
                                       camera_is_ortho=ortho)
    if gui:
        import pyenki.viewer

        # We run a simulation [in real-time] inside a Qt application
        pyenki.viewer.run_in_viewer(world,
                                    time_step=0.1,
                                    walls_height=10,
                                    duration=duration,
                                    **camera_config)
    elif video:
        import pyenki.video
        import pyenki.viewer

        # We generate a video
        v = pyenki.video.make_video(world,
                                    time_step=0.1,
                                    walls_height=10,
                                    duration=2,
                                    width=1280,
                                    height=720,
                                    **camera_config)
        v.write_videofile('hello_thymio.mp4', fps=30)
        pyenki.viewer.cleanup()
    else:
        # We write our own loop to run the simulation as fast as possible.
        steps = int(duration // dt)
        for _ in range(steps):
            world.step(dt)


if __name__ == '__main__':
    main()
