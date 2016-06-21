#include "Sim.hpp"

Sim::Sim()
	: mScene(0), mSceneMgr(0) {
}

Sim::~Sim() {
	delete mScene;

	for (std::vector<CarModel*>::iterator i = carModels.begin(); i != carModels.end(); i++)
		delete (*i);
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	mScene = new Scene(mSceneMgr);
	mScene->setupTerrain();

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
}

Car* Sim::loadCar(std::string carType, CONFIGFILE& cf, const MATHVECTOR<double, 3> pos, const QUATERNION<double> rot,
				  int id) {
	Car* car = new Car();

	if (!car->load(carType, cf, pos, rot, id)) {
		//TODO Error: Loading car
		return NULL;
	} else {
		cars.push_back(car);

		return car;
	}
}
