#include "Sim.hpp"

Sim::Sim(App* app)
	: mSceneMgr(0), scene(0), mApp(app),
	  world(0), carInput(0),
	  numCars(2), // Setting the number of cars
	  idCarToControl(0), // The ID of the car that the user can control
	  carToWatch(0), // ID of car to watch
	  frameRate(1.f / 60.f), targetTime(0),
	  debugDraw(NULL) {
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

	for (int i = 0; i < numCars; i++) {
		Car* newCar = new Car(i);
		newCar->setup("360", mSceneMgr, *world);

		cars.push_back(newCar);

		BasicController* bc = new BasicController(newCar);
		controllers.push_back(bc);
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

//	if (debugDraw) {
//		debugDraw->setDebugMode(1);
//		debugDraw->step();
//	}

	//TODO How Stuntrally updates the game... however, this loop does not work
//	const float minFPS = 10.f; // Minimum acceptable fps
//	const unsigned int maxTicks = (int) (1.f / (minFPS * frameRate));
//	const float maxTime = 1.f / minFPS;
//	unsigned int curTicks = 0;
//
//	// Throw away wall clock time if needed to maintain framerate
//	if (dt > maxTime) dt = maxTime;
//
//	targetTime += dt;
//	double tickPeriod = frameRate;
//
//	// Increment game logic by as many tick periods necessary
//	while (targetTime > tickPeriod && curTicks < maxTicks) {
//		// Advance game logic
//		car->updatePreviousVelocity(); // Was in the loop for fluids...
//		if (tickPeriod > 0) world->update(tickPeriod); // Update physics
//
//		const std::vector<float>& inputs = localMap.processInput(carInput->getPlayerInputState(), car->getSpeedDir(),
//																 0.f, 0.f);
//		car->handleInputs(inputs);
//
//		curTicks++;
//		targetTime -= tickPeriod;
//	}
//	car->update(); // Update model
//
//	if (debugDraw) {
//		debugDraw->setDebugMode(1);
//		debugDraw->step();
//	}
}

Ogre::Vector3 Sim::getCameraPosition() {
	Ogre::Vector3 pos = cars[carToWatch]->getPosition();
	pos += 2 * Ogre::Vector3::UNIT_Y; // Lift camera above car
	pos -= 8 * Axes::vectorToOgre(cars[carToWatch]->getForwardVector()); // Move behind car

	return pos;
}

Ogre::Quaternion Sim::getCameraOrientation() {
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

	if (numCars < 2) return;
	switch (ke.key) {
	case OIS::KC_NUMPAD0:
		carToWatch = 0;
		break;
	case OIS::KC_NUMPAD1:
		carToWatch = 1;
		break;

//	case OIS::KC_1:
//		controllers[1]->turn(true, 50);
//		break;
//	case OIS::KC_2:
//		controllers[1]->turn(false, 50);
//		break;

	default:
		break;
	}
}

void Sim::keyReleased(const OIS::KeyEvent& ke) {
	carInput->keyReleased(ke);
}
