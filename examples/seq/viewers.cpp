#include <QApplication>
#include <QTimer>
#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <viewer/Viewer.h>

using namespace Enki;

void run() {
  World world(120, Color(0.9, 0.9, 0.9));
  EPuck *epuck = new EPuck();
  Thymio2 *thymio2 = new Thymio2();
  thymio2->pos = Point(30, 0);
  thymio2->leftSpeed = 3;
  world.addObject(epuck);
  world.addObject(thymio2);
  ViewerWidget viewer(&world);
  viewer.show();
  QTimer::singleShot(1000, []() { qApp->quit(); });
  qApp->exec();
}

int main(int argc, char *argv[]) {
  EnkiApplication app(argc, argv);
  for (int i = 0; i < 5; ++i) {
    std::cout << "Run #" << i << " starts" << std::endl;
    run();
  }
}
