#include <viewer/Viewer.h>
#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <enki/robots/e-puck/EPuck.h>
#include <QApplication>
#include <QtGui>
#include <QHBoxLayout>

using namespace Enki;

// http://qtnode.net/wiki?title=Qt_with_cmake
int main(int argc, char *argv[])
{
	std::cout << QT_VERSION_STR << std::endl;
	EnkiApplication app(argc, argv);
	// MyApp app(argc, argv);
	
	World * world = new World(120, Color(0.9, 0.9, 0.9));
	// Thymio2 thymio = Thymio2();
	// world.addObject(&thymio);
	EPuck * epuck = new EPuck();
	Thymio2 * thymio = new Thymio2();
	Thymio2 * thymio2 = new Thymio2();
	thymio2->pos = Point(30, 0);
	thymio2->leftSpeed = 3;
	thymio2->rightSpeed = 4;
	// thymio->setLedColor(Thymio2::TOP,Color(0.0,1.0,0.0,1.0));
	world->addObject(thymio);
	world->addObject(thymio2);
	world->addObject(epuck);
	ViewerWidget viewer(world);
	// viewer.show();
	World world1(120, Color(0.9, 0.9, 0.9));
	Thymio2 * thymio1 = new Thymio2();
	thymio1->leftSpeed = 3;
	thymio1->rightSpeed = 4;
	thymio1->setLedColor(Thymio2::TOP,Color(1.0,0.0,0.0,1.0));
	world1.addObject(thymio1);
	Thymio2 * thymio12 = new Thymio2();
	thymio12->pos = Point(30, 0);
	world1.addObject(thymio12);
	
	QWidget window;
	QHBoxLayout hbox = QHBoxLayout(&window);
    window.resize(960, 320);
    ViewerWidget viewer1(&world1, nullptr, 30, true, 1, 1);
	ViewerWidget viewer2(world, nullptr, 30, false);
	hbox.addWidget(&viewer);
	hbox.addWidget(&viewer1);
	hbox.addWidget(&viewer2);
	window.show();
	
	// viewer1.show();
	// viewer2.show();
	// ViewerWidget viewer_2(&world_1);
	// ViewerWidget viewer_1(&world);
	// viewer_2.show();		
	return app.exec();
}

