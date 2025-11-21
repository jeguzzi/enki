import pyenki
import pyenki.viewer


def create_world() -> pyenki.World:
    world = pyenki.World(radius=40)
    epuck = pyenki.EPuck(camera=False)
    epuck.position = (20, 20)
    epuck.left_wheel_target_speed = 10.0
    epuck.set_led_ring(True)
    world.add_object(epuck)
    return world


def main() -> None:
    # Needs to be called before creating the first view
    world = create_world()
    pyenki.viewer.init()
    viewer = pyenki.viewer.WorldView(world=world, walls_height=2)
    viewer.reset_camera()
    viewer.show()
    viewer.start_updating_world(0.1)
    # executes the runloop for a while
    pyenki.viewer.run(duration=10)
    pyenki.viewer.cleanup()


if __name__ == '__main__':
    main()
