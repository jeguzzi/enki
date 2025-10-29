import pyenki
from PyQt6.QtCore import QCoreApplication, Qt
from PyQt6.QtWidgets import QApplication, QHBoxLayout, QWidget

# replaces pyenki.init_ui(): needs to be called before creating any widget
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts),
app = QApplication([])

world = pyenki.World(radius=100)
epuck = pyenki.EPuck(camera=False)
epuck.left_wheel_target_speed = 10.0
epuck.set_led_ring(True)
world.add_object(epuck)
viewer_1 = pyenki.WorldView(world,
                            camera_position=(-20, -20),
                            camera_altitude=20)
viewer_1.point_camera(target_position=(0, 0), target_altitude=5)
viewer_2 = pyenki.WorldView(world,
                            helpers=False,
                            camera_altitude=30,
                            camera_is_ortho=True)
viewer_2.camera_is_ortho = True
window = QWidget()
hbox = QHBoxLayout(window)
window.resize(960, 320)
hbox.addWidget(viewer_1.widget)
hbox.addWidget(viewer_2.widget)
window.show()
viewer_1.start_updating_world(0.1)
app.exec()
