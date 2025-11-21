import pyenki
import pyenki.viewer

from world_view import create_world


def main() -> None:

    # PyQt6 requires the native viewer implemented in C++
    assert pyenki.viewer.use_native

    from PyQt6.QtCore import QCoreApplication, Qt
    from PyQt6.QtWidgets import QApplication, QHBoxLayout, QWidget

    # equivalent to pyenki.viewer.init()
    # needs to be called before creating the first widget
    QCoreApplication.setAttribute(
        Qt.ApplicationAttribute.AA_ShareOpenGLContexts),
    app = QApplication([])

    world = create_world()
    viewer_1 = pyenki.viewer.WorldView(world=world,
                                       camera_position=(-20, -20),
                                       camera_altitude=20)
    viewer_1.point_camera(target_position=(20, 20), target_altitude=5)
    viewer_2 = pyenki.viewer.WorldView(world=world,
                                       helpers=False,
                                       camera_altitude=30,
                                       camera_is_ortho=False)
    viewer_2.move_camera(target_position=(20, 20),
                         target_altitude=10,
                         yaw=-1,
                         pitch=-0.5)
    window = QWidget()
    hbox = QHBoxLayout(window)
    window.resize(960, 320)
    # Note that we get a PyQt compatible widget
    # with `pyqt_widget`
    hbox.addWidget(viewer_1.pyqt_widget)
    hbox.addWidget(viewer_2.pyqt_widget)
    window.show()
    viewer_1.start_updating_world(0.1)
    app.exec()


if __name__ == '__main__':
    main()
