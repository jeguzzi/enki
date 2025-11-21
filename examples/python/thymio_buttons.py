import pyenki
import pyenki.viewer


def on_button_touch(thymio: pyenki.Thymio2,
                    button: pyenki.Thymio2.Button) -> None:
    if button == pyenki.Thymio2.Button.CENTER:
        thymio.set_led_top(0, 0, 0)
    if button == pyenki.Thymio2.Button.FORWARD:
        thymio.set_led_top(1, 0, 0)
    if button == pyenki.Thymio2.Button.BACKWARD:
        thymio.set_led_top(0, 1, 0)
    if button == pyenki.Thymio2.Button.LEFT:
        thymio.set_led_top(0, 0, 1)
    if button == pyenki.Thymio2.Button.RIGHT:
        thymio.set_led_top(1, 0, 1)


def main() -> None:
    world = pyenki.World()
    thymio = pyenki.Thymio2()
    thymio.button_touch_callback = on_button_touch
    world.add_object(thymio)
    pyenki.viewer.init()
    viewer = pyenki.viewer.WorldView(world=world)
    viewer.move_camera(target_distance=20,
                       target_position=(8, 0),
                       pitch=-1.2,
                       yaw=0)
    viewer.show()
    viewer.start_updating_world(0.1)
    pyenki.viewer.run()


if __name__ == '__main__':
    main()
