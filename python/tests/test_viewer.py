import pyenki
import pyenki.viewer
from PySide6.QtWidgets import QGridLayout, QWidget
import numpy as np

def rotate(world, time_step):
    for obj in world.objects:
        obj.angle += time_step * 1


pyenki.viewer.init()

gt = np.zeros((10, 10, 4), dtype=np.uint8)
gt[..., 3] = 255
gt[:5, :5, 0] = 255  # blue
gt[:5, :5, 1] = 255  # green
gt[:5, 5:, 1] = 255  # green
gt[5:, :5, 2] = 255  # red

worlds = []

world = pyenki.World()
worlds.append(world)

world = pyenki.World(radius=30)
worlds.append(world)

world = pyenki.World(radius=30, ground_texture=pyenki.World.GroundTexture(gt))
worlds.append(world)

world = pyenki.World(width=40, height=40)
worlds.append(world)

world = pyenki.World(width=40, height=40)
worlds.append(world)

world = pyenki.World(width=40, height=40, ground_texture=pyenki.World.GroundTexture(gt))
worlds.append(world)

world = pyenki.World(radius=30)
robot = pyenki.EPuck(camera=False)
robot.set_led_ring(True)
world.add_object(robot)
worlds.append(world)

world = pyenki.World(radius=30)
robot = pyenki.Thymio2()
robot.set_led_top(1.0, 0.5, 0.0)
world.add_object(robot)
worlds.append(world)

world = pyenki.World(radius=30)
robot = pyenki.Marxbot()
world.add_object(robot)
worlds.append(world)

world = pyenki.World(radius=30)
obj = pyenki.PhysicalObject(radius=10,
                            height=10,
                            mass=-1,
                            color=pyenki.Color.blue)
world.add_object(obj)
worlds.append(world)

world = pyenki.World(radius=30)
obj = pyenki.PhysicalObject(l1=20,
                            l2=10,
                            height=10,
                            mass=-1,
                            color=pyenki.Color.red)
world.add_object(obj)
worlds.append(world)

world = pyenki.World(radius=30)
part = pyenki.PhysicalObject.Part(shape=[(0.0, 0.0), (0.0, 10.0),
                                         (10.0, 10.0)],
                                  height=10)
obj = pyenki.PhysicalObject(parts=[part], mass=1, color=pyenki.Color.green)
world.add_object(obj)
worlds.append(world)

world = pyenki.World(radius=30)
parts = [
    pyenki.PhysicalObject.Part(shape=[(-5.0, 0.0), (10.0, 10.0),
                                      (10.0, -10.0)],
                               height=10),
    pyenki.PhysicalObject.Part(shape=[(5.0, 0.0), (-10.0, -10.0),
                                      (-10.0, 10.0)],
                               height=15)
]
obj = pyenki.PhysicalObject(parts=parts, mass=1, color=pyenki.Color.gray)
world.add_object(obj)
worlds.append(world)

world = pyenki.World(radius=30)
shape = [(0.0, 5.0), (10.0, 5.0), (10.0, -5.0)]
textures = [[pyenki.Color.red], [pyenki.Color.green], [pyenki.Color.blue]]
part = pyenki.PhysicalObject.Part(shape=shape, height=10, textures=textures)
obj = pyenki.PhysicalObject(parts=[part], mass=1, color=pyenki.Color.white)
world.add_object(obj)
worlds.append(world)

viewers = [
    pyenki.viewer.WorldView(world=world,
                            camera_position=(-20, -20),
                            camera_altitude=20) for world in worlds
]

window = QWidget()
grid = QGridLayout(window)
# window.resize(960, 320)

i = 0
j = 0

for viewer in viewers:
    viewer.point_camera(target_position=(0, 0), target_altitude=5)
    grid.addWidget(viewer, i, j)
    j += 1
    if j == 3:
        j = 0
        i += 1
    viewer.world.control_step_callback = rotate
    viewer.start_updating_world(time_step=0.03)

viewers[4].walls_height = 5

window.show()
pyenki.viewer.run()
pyenki.viewer.cleanup()
