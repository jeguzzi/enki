import pyenki
import pyenki.viewer

from world_view import create_world


def main() -> None:

    # PySide6 requires the viewer implemented in Python
    assert not pyenki.viewer.use_native

    from pyenki.viewer.utils import setup_context
    from PySide6.QtCore import QCoreApplication, Qt
    from PySide6.QtWidgets import QApplication, QHBoxLayout, QWidget

    world = create_world()
    # replaces pyenki.viewer.init
    # needs to be called before creating any widget
    QCoreApplication.setAttribute(
        Qt.ApplicationAttribute.AA_ShareOpenGLContexts),
    setup_context()
    app = QApplication([])

    world = create_world()
    viewer_1 = pyenki.viewer.WorldView(world=world,
                                       camera_position=(-20, -20),
                                       camera_altitude=20)
    viewer_1.point_camera(target_position=(0, 0), target_altitude=5)
    viewer_2 = pyenki.viewer.WorldView(world=world,
                                       helpers=False,
                                       camera_altitude=30,
                                       camera_is_ortho=False)
    viewer_2.move_camera(target_position=(0, 0),
                         target_altitude=10,
                         yaw=-1,
                         pitch=-0.5)
    window = QWidget()
    hbox = QHBoxLayout(window)
    window.resize(960, 320)
    hbox.addWidget(viewer_1.pyside_widget)
    hbox.addWidget(viewer_2.pyside_widget)
    window.show()
    viewer_1.start_updating_world(0.1)
    app.exec()


if __name__ == '__main__':
    main()
