import pathlib as pl
import sys

import numpy as np
import PIL.Image
import pyenki
import pyenki.video
import pyenki.viewer
from pyenki.adapters import make_controller_from_thymio_behavior
from thymio_behaviors import LineFollowingBehavior


def make_world() -> pyenki.World:
    path = pl.Path(__file__).parent / 'line_following_ground.png'
    im = np.asarray(PIL.Image.open(path).convert('RGBA'))
    ground_texture = pyenki.World.GroundTexture(im)
    world = pyenki.World(lx=200, ly=200, ground_texture=ground_texture)
    thymio = pyenki.Thymio2()
    thymio.position = (110, 100)
    thymio.angle = 0
    thymio.control_step_callback = make_controller_from_thymio_behavior(
        thymio, LineFollowingBehavior())
    world.add_object(thymio)
    return world


def main() -> None:
    pyenki.viewer.init()
    world = make_world()
    camera_config: pyenki.viewer.CameraConfig = dict(camera_altitude=105,
                                                     camera_pitch=-1.15,
                                                     camera_yaw=0.0,
                                                     camera_position=(50, 100))
    if '--video' in sys.argv:
        v = pyenki.video.make_video(world,
                                    time_step=0.033,
                                    duration=60,
                                    width=1280,
                                    height=720,
                                    factor=2,
                                    **camera_config)
        v.write_videofile('thymio_line_following.mp4', fps=30)
    else:
        pyenki.viewer.run_in_viewer(world,
                                    time_step=0.02,
                                    duration=-1,
                                    **camera_config)
    pyenki.viewer.cleanup()


if __name__ == '__main__':
    main()
