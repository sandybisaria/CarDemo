#pragma once

#include "Scene.hpp"
#include "CarModel.hpp"
class CarModel;

#include "vdrift/Car.hpp"
class Car;
#include "vdrift/configfile.h"
#include "vdrift/mathvector.h"
#include "vdrift/quaternion.h"

#include "OgreSceneManager.h"

class Sim {
public:
	Sim();
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);
	void update(float dt);

	// Load car
	Car* loadCar(std::string carType, CONFIGFILE& cf, const MATHVECTOR<float, 3> pos, const QUATERNION<float> rot,
				 int id);

private:
	Scene* mScene;

	std::vector<Car*> cars;

	Ogre::SceneManager* mSceneMgr;
};
