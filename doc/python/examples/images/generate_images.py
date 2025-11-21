import pyenki
from pyenki.viewer import init, cleanup, save_image
from pyenki.viewer.camera import Camera

init()

world = pyenki.World()
thymio = pyenki.Thymio2()
world.add_object(thymio)
thymio.position = (14.1, 7.2)
thymio.angle = 4.0
thymio.set_led_top(0.5, 0.5, 0.0)
obj = pyenki.PhysicalObject(l1=5.0,
                            l2=5.0,
                            height=5.0,
                            mass=-1,
                            color=pyenki.Color(0.8, 0.3, 0))
world.add_object(obj)
camera = Camera(pitch=-0.6, yaw=1.8)
camera.move(target_position=(*(thymio.position * 0.5), thymio.height * 0.5),
            target_distance=18)
save_image(world, "thymio.png", **camera.config)

world = pyenki.World(radius=20, walls_color=pyenki.Color(0.5, 0.2, 0.0))
marxbot = pyenki.Marxbot()
world.add_object(marxbot)
camera = Camera(pitch=-0.7, yaw=1.0)
camera.move(target_position=(*marxbot.position, 0), target_distance=30)
save_image(world, "marxbot.png", **camera.config)

world = pyenki.World()
epuck = pyenki.EPuck()
epuck.position = (-8, 0)
epuck.set_led_ring(True)
world.add_object(epuck)
world.add_object(
    pyenki.PhysicalObject(radius=2.0,
                          height=5.0,
                          mass=-1,
                          color=pyenki.Color(0.3, 0.7, 0)))
camera = Camera(pitch=-0.6, yaw=2)
camera.move(target_position=(*(epuck.position * 0.5), 2), target_distance=12)
save_image(world, "epuck.png", **camera.config)

world = pyenki.World()
parts = [
    pyenki.PhysicalObject.Part(shape=[(0, 1), (0, 0.5), (2, 0.5), (2, 1)],
                               height=1.0),
    pyenki.PhysicalObject.Part(shape=[(0, -0.5), (0, -1), (2, -1), (2, -0.5)],
                               height=1.0),
    pyenki.PhysicalObject.Part(shape=[(0, 0.5), (0, -0.5), (0.5, -0.5),
                                      (0.5, 0.5)],
                               height=1.0),
]
c = pyenki.PhysicalObject(parts, mass=-1, color=pyenki.Color(0, 0.5, 0.5))
world.add_object(c)

triangle = pyenki.PhysicalObject(shape=[(0.0, 0.0), (1.0, -1.0), (1.0, 1.0)],
                                 height=1,
                                 mass=-1,
                                 color=pyenki.Color(0.5, 0.5, 0.0))
triangle.position = (5, 0)
world.add_object(triangle)

cylinder = pyenki.PhysicalObject(radius=1.0,
                                 height=1.0,
                                 mass=-1,
                                 color=pyenki.Color(0.5, 0.0, 0.5))
cylinder.position = (10, 0)
world.add_object(cylinder)

box = pyenki.PhysicalObject(l1=2.0,
                            l2=1.0,
                            height=1.0,
                            mass=-1,
                            color=pyenki.Color(0.2, 0.5, 0.7))
box.position = (15, 0)
world.add_object(box)

colorful_box_shape = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
colorful_box_colors = [[pyenki.Color.red], [pyenki.Color(0.5, 0.5, 0.0)],
                       [pyenki.Color.green], [pyenki.Color(0.0, 0.5, 0.5)]]
part = pyenki.PhysicalObject.Part(shape=colorful_box_shape,
                                  height=1,
                                  textures=colorful_box_colors)
colorful_box = pyenki.PhysicalObject(shape=colorful_box_shape,
                                     height=1,
                                     textures=colorful_box_colors,
                                     mass=-1,
                                     color=pyenki.Color.white)
# colorful_box = pyenki.PhysicalObject(parts=[part], mass=-1)
# colorful_box.color =  pyenki.Color.white
colorful_box.position = (20, 0)
world.add_object(colorful_box)
camera = Camera(pitch=-0.7, yaw=1.5)
camera.move(target_position=(10, 0, 1), target_distance=12)
save_image(world, "objects.png", **camera.config)

cleanup()
