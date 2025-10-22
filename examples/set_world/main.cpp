#include <QApplication>
#include <QTimer>
#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <viewer/Viewer.h>

using namespace Enki;

void run(ViewerWidget & viewer, const Point & pos) {
  World world(120, Color(0.9, 0.9, 0.9));
  Thymio2 *thymio2 = new Thymio2();
  thymio2->pos = pos;
  world.addObject(thymio2);
  viewer.setWorld(&world);
  viewer.resetCamera();
  // viewer.show();
  // QTimer::singleShot(2000, [&viewer]() { qApp->quit(); });
  QTimer::singleShot(2000, [&viewer]() { viewer.setWorld(nullptr); qApp->quit(); });
  qApp->exec();
}

int main(int argc, char *argv[]) {
  EnkiApplication app(argc, argv);
  ViewerWidget viewer(nullptr);
  viewer.show();
  for (int i = 0; i < 3; ++i) {
    std::cout << "Run #" << i << " starts" << std::endl;
    run(viewer, Point(i * 10, 0));
    viewer.show();
  }
  qApp->exec();
}
