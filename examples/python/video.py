import pyenki
import pyenki.viewer
from pyenki.video import make_video

pyenki.viewer.init()
world = pyenki.World()
thymio = pyenki.Thymio2()
thymio.left_wheel_target_speed = 10
thymio.right_wheel_target_speed = -6
thymio.set_led_circle(-1, 1.0)
thymio.set_led_buttons(0, 1.0)
thymio.set_led_buttons(2, 1.0)
thymio.set_led_top(1.0, 0.0, 0.3)
world.add_object(thymio)

v = make_video(world,
               time_step=0.033,
               duration=20,
               camera_position=(0, -20),
               camera_altitude=20,
               camera_pitch=-0.7,
               camera_yaw=1.5,
               width=1280,
               height=720)

v.write_videofile('world.mp4', fps=30)
pyenki.viewer.cleanup()
print('saved world.mp4')
