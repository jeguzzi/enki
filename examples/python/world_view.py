import pyenki

world = pyenki.World(radius=100)
epuck = pyenki.EPuck(camera=False)
epuck.left_wheel_target_speed = 10.0
epuck.set_led_ring(True)
world.add_object(epuck)
# setup Qt: needs to be called before creating the first view
pyenki.init_ui()
viewer = pyenki.WorldView(world)
viewer.reset_camera()
viewer.show()
viewer.start_updating_world(0.1)
# executes the Qt runloop for a while
pyenki.run_ui(duration=10)
