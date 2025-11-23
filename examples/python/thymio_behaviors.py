import math
import sys

import pyenki
import pyenki.video
import pyenki.viewer
from pyenki.behaviors import (ThymioAccBehavior, ThymioExplorerBehavior,
                              ThymioFollowerBehavior)


def make_world() -> pyenki.World:
    world = pyenki.World(radius=50, seed=4)
    rng = world.random_generator
    behaviors = (ThymioAccBehavior, ThymioExplorerBehavior,
                 ThymioFollowerBehavior)
    for _ in range(7):
        thymio = pyenki.Thymio2()
        thymio.position = (rng.uniform(-30, 30), rng.uniform(-30, 30))
        thymio.angle = rng.uniform(0, 2 * math.pi)
        thymio.control_step_callback = rng.choice(behaviors)()  # type: ignore[arg-type]
        world.add_object(thymio)
    world.run(2, 0.033)
    return world


def main() -> None:
    pyenki.viewer.init()
    world = make_world()
    camera_config: pyenki.viewer.CameraConfig = dict(camera_altitude=55,
                                                     camera_pitch=-0.8,
                                                     camera_yaw=0.7,
                                                     camera_position=(-50,
                                                                      -50))
    if '--video' in sys.argv:
        v = pyenki.video.make_video(world,
                                    time_step=0.033,
                                    duration=60,
                                    width=1280,
                                    height=720,
                                    factor=2,
                                    **camera_config)
        v.write_videofile('thymio_behaviors.mp4', fps=30)
    else:
        pyenki.viewer.run_in_viewer(world,
                                    time_step=0.02,
                                    duration=-1,
                                    **camera_config)
    pyenki.viewer.cleanup()


if __name__ == '__main__':
    main()
