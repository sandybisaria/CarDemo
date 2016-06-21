#include "Sim.hpp"

#include "CarPosInfo.hpp"

Sim::Sim()
	: mSceneMgr(0) {
}

Sim::~Sim() {
	for (std::vector<CarModel*>::iterator i = carModels.begin(); i != carModels.end(); i++)
		delete (*i);
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	// Init CarModels (load vdrift Cars first)
	CarModel* carModel = new CarModel(0, "360", mSceneMgr, this);
	carModel->load();
	carModels.push_back(carModel);

	// Now we can create the CarModels
	for (int i = 0; i < carModels.size(); i++) {
		CarModel* cm = carModels[i];
		cm->create();
	}
}

void Sim::update(float dt) {
	//TODO Try maintaining constant framerate?

	// Advance game logic (as in GAME::AdvanceGameLogic())
	//TODO Clear fluids for each car

	for (int i = 0; i < cars.size(); i++) {
		cars[i]->update(dt);
		//TODO Process input
	}

	// Update poses (as in App::newPoses())
	for (int i = 0; i < carModels.size(); i++) {
		CarModel* cm = carModels[i];

		if (i >= cars.size()) // Because the car models are created before the cars...?
			continue;

		Car* c = cars[i];

		CarPosInfo cpi;

		if (c) cpi.fromCar(c);
		cpi.hasNew = true;

		// Now from App::updatePoses()
		cm->update(cpi, dt);
	}
}

Car* Sim::loadCar(std::string carType, CONFIGFILE& cf, const MATHVECTOR<double, 3> pos, const QUATERNION<double> rot,
				  int id) {
	std::cout << "Loading from Sim" << std::endl;
	Car* car = new Car();
	if (!car->load(carType, cf, pos, rot, id)) {
		//TODO Error: Loading car
		return NULL;
	} else {
		cars.push_back(car);
		return car;
	}
}
