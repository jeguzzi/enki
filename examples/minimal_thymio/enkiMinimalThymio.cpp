#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <iostream>

const double DT = 0.1;

void log(Enki::Thymio2 *thymio, int id, double time){
	std::vector<Enki::IRCommEvent> events = thymio->irComm.get_events();
	if (!events.size()) return;
	std::cout << "At time " << time << " Thymio" << id << " received msg \n";
	for(std::vector<Enki::IRCommEvent>::iterator it=events.begin(); it!=events.end(); it++)
	{
		std::cout << *it<< "\n";
	}
}

int main(int argc, char *argv[])
{
	// Create the world
	std::cout << "Initialize\n";
	Enki::World world(2000, 2000);
	Enki::Thymio2 *thymio_1 = new Enki::Thymio2();
	thymio_1->pos = Enki::Point(100, 100);
	thymio_1->angle = 0;
	Enki::Thymio2 *thymio_2 = new Enki::Thymio2();
	thymio_2->pos = Enki::Point(130, 100);
	thymio_2->angle = M_PI;
	// thymio_2->leftSpeed = -20;
	// thymio_2->rightSpeed = 20;
	Enki::Thymio2 *thymio_3 = new Enki::Thymio2();
	thymio_3->pos = Enki::Point(145, 100);
	thymio_3->angle = 0;

	// objects are garbage collected by the world on destruction
	world.addObject(thymio_1);
	world.addObject(thymio_2);
	world.addObject(thymio_3);

	thymio_1->irComm.set_enable(true);
	thymio_2->irComm.set_enable(true);
	thymio_3->irComm.set_enable(true);
	thymio_1->irComm.set_tx(111);
	thymio_2->irComm.set_tx(222);
	thymio_3->irComm.set_tx(333);

	std::cout << "Start Simulation\n\n";

	// Run for some times
	for (unsigned i=0; i<15; i++)
	{
		world.step(DT);
		double time = DT * i;
		log(thymio_1, 1, time);
		log(thymio_2, 2, time);
		log(thymio_3, 3, time);
	}

	std::cout << "\nEnd Simulation\n";
}
