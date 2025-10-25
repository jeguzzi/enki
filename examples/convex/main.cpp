#include <QApplication>
#include <QTimer>
#include <enki/PhysicalEngine.h>
#include <viewer/Viewer.h>

using namespace Enki;

int main(int argc, char *argv[]) {
  EnkiApplication app(argc, argv);
  World world(120, Color(0.9, 0.9, 0.9));
  auto c = std::make_unique<PhysicalObject>();
  std::vector<Vector> ps{Vector(0.0, 0.0), Vector(10.0, 0.0), Vector(10.0, 10.0),
                         Vector(0.0, 10.0)};
  Polygon p;
  p.assign(ps.begin(), ps.end());
  Textures textures{Texture{Color::red}, Texture{Color::green},
                    Texture{Color::blue}, Texture{Color::yellow}};
  PhysicalObject::Part part(p, 10, textures);
  PhysicalObject::Hull hull(part);
  c->setCustomHull(hull, 1.0);
  world.addObject(c.get());
  ViewerWidget viewer(&world);
  viewer.resetCamera();
  viewer.show();
  QTimer::singleShot(10000, []() { qApp->quit(); });
  qApp->exec();
}