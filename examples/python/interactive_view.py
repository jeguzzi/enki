import math

import pyenki

world = pyenki.World(100)
thymio = pyenki.Thymio2()
thymio.position = (0, 0)
thymio.left_wheel_target_speed = -10
thymio.right_wheel_target_speed = 10
world.add_object(thymio)

pyenki.init_ui()
view = pyenki.WorldView(world,
                        update_world=True,
                        camera_position=(0, 0),
                        camera_altitude=80,
                        camera_pitch=-math.pi / 2,
                        camera_yaw=math.pi / 2,
                        camera_is_ortho=True)
view.show()
pyenki.run_ui()
pyenki.cleanup_ui()
