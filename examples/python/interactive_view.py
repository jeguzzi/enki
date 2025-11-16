import math

import pyenki
from pyenki import viewer

world = pyenki.World(radius=100)
thymio = pyenki.Thymio2()
thymio.position = (0, 0)
thymio.left_wheel_target_speed = -10
thymio.right_wheel_target_speed = 10
world.add_object(thymio)

viewer.init()
view = viewer.WorldView(world=world,
                        update_world=True,
                        camera_position=(0, 0),
                        camera_altitude=80,
                        camera_pitch=-math.pi / 2,
                        camera_yaw=math.pi / 2,
                        camera_is_ortho=False)
view.show()
viewer.run()
viewer.cleanup()
