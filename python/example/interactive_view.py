import math

from PyQt5 import QtCore

import pyenki

world = pyenki.World(100)
thymio = pyenki.Thymio2()
thymio.position = (0, 0)
thymio.motor_left_target = -10
thymio.motor_right_target = 10
world.add_object(thymio)

# Check if a QtApplication is running
if not QtCore.QCoreApplication.instance():
    print('No QtApplication active')
else:
    # Create a view --- which will also run ``world.step`` --- and display it
    view = pyenki.WorldView(world, run_world_update=True, cam_position=(0, -140),
                            cam_altitude=80, cam_pitch=-0.6, cam_yaw=math.pi / 2)
    view.show()
