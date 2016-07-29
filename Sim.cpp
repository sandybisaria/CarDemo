#include "Sim.hpp"

Sim::Sim(App* app)
	: mSceneMgr(0), scene(0), mApp(app),
	  world(0), carInput(0),
	  frameRate(1.f / 60.f), targetTime(0),
	  debugDraw(NULL), enableDebug(false) {
}

Sim::~Sim() {
	delete world;
	delete carInput;
	delete debugDraw;

	for (std::vector<Car*>::iterator it = cars.begin(); it != cars.end(); it++) {
		delete (*it);
	}
	cars.clear();

	for (std::vector<BasicController*>::iterator it = controllers.begin(); it != controllers.end(); it++) {
		delete (*it);
	}
	controllers.clear();
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	world = new CollisionWorld();
	world->sim = this; // Maybe make more secure?

	numCars = 1; // Setting the number of cars
	carToWatch = 0; // ID of car to watch
	idCarToControl = 0;  // The ID of the car that the user can control

	for (int i = 0; i < numCars; i++) {
		Car* newCar = new Car(i);
		newCar->setup((i == idCarToControl ? "CAD" : "360"), mSceneMgr, *world); // A Cadillac driving with Ferrari's...

		cars.push_back(newCar);

		BasicController* bc = new BasicController(newCar);
		controllers.push_back(bc);

		//TODO Initial speed configuration would probably come from the scene
		bc->setSpeed(10);

		// For testing only (uncomment when needed)
//		bc->setupDataCollection();
	}

	carInput = new CInput(this);

	// Debug drawing
	debugDraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), world->getDynamicsWorld());
	world->getDynamicsWorld()->setDebugDrawer(debugDraw);
	world->getDynamicsWorld()->getDebugDrawer()->setDebugMode(1);
}

void Sim::update(float dt) {
//	// Was in the loop for fluids...
//	for (int i = 0; i < numCars; i++) { cars[i]->updatePreviousVelocity(); }

	// Update physics
	if (dt > 0) world->update(dt);

	// Update model
	for (int i = 0; i < numCars; i++) { cars[i]->update(); }

	// Update inputs
	for (int i = 0; i < numCars; i++) {
		const std::vector<double>* inputs;
		if (i == idCarToControl) {
			 inputs = &localMap.processInput(carInput->getPlayerInputState(), cars[i]->getSpeedDir(), 0.0, 0.0);
		} else {
			//TODO Add "basic AI" for the vehicles
			inputs = &controllers[i]->updateInputs(dt);
		}

		cars[i]->handleInputs(*inputs);
	}

	if (debugDraw) {
		debugDraw->setDebugMode(enableDebug ? 1 : 0);
		debugDraw->step();
	}

	//TODO May want to see how Stuntrally "actually" loops
}

Ogre::Vector3 Sim::getCameraPosition() {
	if (carToWatch >= cars.size()) return Ogre::Vector3::ZERO;

	Ogre::Vector3 pos = cars[carToWatch]->getPosition();
	pos += 2 * Ogre::Vector3::UNIT_Y; // Lift camera above car
	pos -= 8 * Axes::vectorToOgre(cars[carToWatch]->getForwardVector()); // Move behind car

	return pos;
}

Ogre::Quaternion Sim::getCameraOrientation() {
	if (carToWatch >= cars.size()) return Ogre::Quaternion::IDENTITY;

	Ogre::Quaternion orient = cars[carToWatch]->getOrientation();

	Ogre::Vector3 downVector = Axes::vectorToOgre(cars[carToWatch]->getDownVector());
	orient = orient * Ogre::Quaternion(Ogre::Degree(270), downVector); // Rotate to face front of car

	// Turn right-side-up; not sure why we can simply rotate around the Z axis
	Ogre::Vector3 forwardVector = /*Axes::vectorToOgre(cars[carToWatch]->getForwardVector());*/ Ogre::Vector3::UNIT_Z;
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
