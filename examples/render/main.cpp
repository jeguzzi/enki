#include <viewer/Viewer.h>
#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>

using namespace Enki;

int main(int argc, char *argv[])
{
	// std::cout << QT_VERSION_STR << std::endl;
	// EnkiApplication app(argc, argv);
	EnkiApplication::init();
	{
	    World * world = new World(120, Color(0.9, 0.9, 0.9));
	    Thymio2 * thymio = new Thymio2();
	    world->addObject(thymio);
	    ViewerWidget viewer(world);
	    viewer.saveImage("render.png");
	}
	EnkiApplication::cleanup();
}

