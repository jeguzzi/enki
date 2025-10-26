import pyenki

pyenki.init_ui()
world = pyenki.World(radius=100)
epuck = pyenki.EPuck()
epuck.set_led_ring(True)
epuck.position = (0, -12)
world.add_object(epuck)
thymio = pyenki.Thymio2()
thymio.set_led_top(red=1)
thymio.position = (0, 0)
world.add_object(thymio)
marxbot = pyenki.Marxbot()
marxbot.position = (0, 20)
world.add_object(marxbot)
world.save_image("world.png",
                 camera_position=(30, -10),
                 camera_altitude=20,
                 camera_yaw=2.6,
                 camera_pitch=-0.487)
