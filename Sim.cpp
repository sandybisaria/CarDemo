#include "Sim.hpp"

Sim::Sim()
	: mScene(0), mSceneMgr(0) {
}

Sim::~Sim() {
	delete mScene;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	mScene = new Scene(mSceneMgr);
	mScene->setupTerrain();
}

void Sim::update(float dt) {
}

Car* Sim::loadCar(std::string carType, CONFIGFILE& cf, const MATHVECTOR<float, 3> pos, const QUATERNION<float> rot,
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
