import pyenki
import pyenki.viewer

world = pyenki.World(radius=40)
epuck = pyenki.EPuck(camera=False)
epuck.position = (20, 20)
epuck.left_wheel_target_speed = 10.0
epuck.set_led_ring(True)
world.add_object(epuck)
# setup Qt: needs to be called before creating the first view
pyenki.viewer.init()
viewer = pyenki.viewer.WorldView(world=world, walls_height=2)
viewer.reset_camera()
viewer.show()
viewer.start_updating_world(0.1)
# executes the Qt runloop for a while
pyenki.viewer.run(duration=4)
del viewer
pyenki.viewer.cleanup()
# pyenki.run_ui(duration=-1)
