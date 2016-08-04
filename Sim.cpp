#include "Sim.hpp"
#include "App.hpp"

#include "ai/BasicController.hpp"

#include "vdrift/CollisionWorld.hpp"

Sim::Sim(App* app)
	: mSceneMgr(0), scene(0), mApp(app),
	  world(0), carInput(0),
	  debugDraw(NULL), enableDebug(false) {
}

Sim::~Sim() {
	for (std::vector<Car*>::iterator it = cars.begin();
		 it != cars.end(); it++) {
		delete (*it);
	}
	cars.clear();

	for (std::vector<BasicController*>::iterator it = controllers.begin();
		 it != controllers.end(); it++) {
		delete (*it);
	}
	controllers.clear();

	delete world;
	delete carInput;
	delete debugDraw;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	world = new CollisionWorld(this);

	numCars = 1;
	idCarToWatch = 0;
	idCarToControl = 0;

	for (unsigned int i = 0; i < numCars; i++) {
		Car* newCar = new Car(i);
		// A Cadillac driving with Ferrari's...
		newCar->setup((i == idCarToControl ? "CAD" : "360"), mSceneMgr, *world);

		cars.push_back(newCar);

		BasicController* bc = new BasicController(newCar);
		controllers.push_back(bc);

		//TODO Initial speed configuration would probably come from the scene
		bc->setSpeed(10);

		// For testing only (uncomment when needed)
		bc->setupDataCollection();
	}

	carInput = new CInput();

	// Debug drawing
	debugDraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), world->getDynamicsWorld());
	world->getDynamicsWorld()->setDebugDrawer(debugDraw);
	world->getDynamicsWorld()->getDebugDrawer()->setDebugMode(0);
}

void Sim::update(float dt) {
	// Update physics
	world->update(dt);

	// Update model
	for (int i = 0; i < numCars; i++) { cars[i]->update(); }

	// Update inputs
	for (int i = 0; i < numCars; i++) {
		const std::vector<double>* inputs;
		if (i == idCarToControl) {
			// SSS effects/factors all zero by default
			inputs = &localMap.processInput(carInput->getPlayerInputState(),
											cars[i]->getSpeedDir(), 0.0, 0.0);
		} else {
			inputs = &controllers[i]->updateInputs(dt);
		}

		cars[i]->handleInputs(*inputs);
	}

	if (debugDraw) {
		debugDraw->setDebugMode(enableDebug ? 1 : 0);
		debugDraw->step();
	}

	// In the future, may want to implement Stuntrally's looping
}

Ogre::Vector3 Sim::getCameraPosition() {
	if (clamp(idCarToWatch, 0, (int) cars.size()) != idCarToWatch) {
		return Ogre::Vector3::ZERO;
	}

	Ogre::Vector3 pos = cars[idCarToWatch]->getPosition();
	pos += 2 * Ogre::Vector3::UNIT_Y; // Lift camera above car
	pos -= 8 * Axes::vectorToOgre(
		cars[idCarToWatch]->getForwardVector()); // Move behind car

	return pos;
}

Ogre::Quaternion Sim::getCameraOrientation() {
	if (clamp(idCarToWatch, 0, (int) cars.size()) != idCarToWatch) {
		return Ogre::Quaternion::IDENTITY;
	}

	Ogre::Quaternion orient = cars[idCarToWatch]->getOrientation();

	Ogre::Vector3 downVector = Axes::vectorToOgre(cars[idCarToWatch]->getDownVector());
	orient = orient * Ogre::Quaternion(
		Ogre::Degree(270), downVector); // Rotate to face front of car

	// Turn right-side-up; not sure why we can simply rotate around the Z axis
	Ogre::Vector3 forwardVector = Ogre::Vector3::UNIT_Z;
	orient = orient * Ogre::Quaternion(Ogre::Degree(180), forwardVector);

	return orient;
}

TerrainSurface* Sim::getTerrainSurface(std::string name) {
	return mApp->getTerrainSurface(name);
}

void Sim::keyPressed(const OIS::KeyEvent& ke) {
	carInput->keyPressed(ke);

	switch (ke.key) {
	// Toggle debug drawing
	case OIS::KC_B:
		enableDebug = !enableDebug;
		break;

	// Testing turns
	case OIS::KC_1:
		controllers[0]->turn(true,  50, 45);
		break;
	case OIS::KC_2:
		controllers[0]->turn(false, 30);
		break;

	// Testing lane changes
	case OIS::KC_3:
		controllers[0]->laneChange(true,  3.7);
		break;
	case OIS::KC_4:
		controllers[0]->laneChange(false, 3.7);
		break;

	default:
		break;
	}
}

void Sim::keyReleased(const OIS::KeyEvent& ke) {
	carInput->keyReleased(ke);
}
