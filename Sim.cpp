#include "Sim.hpp"

Sim::Sim(App* app)
	: mSceneMgr(0), scene(0), mApp(app),
	  world(0), carInput(0),
	  numCars(1), // Setting the number of cars
	  idCarToControl(0), // The ID of the car that the user can control
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
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	world = new CollisionWorld();
	world->sim = this; // Maybe make more secure?

	for (int i = 0; i < numCars; i++) {
		Car* newCar = new Car(i);
		newCar->setup("360", mSceneMgr, *world);

		cars.push_back(newCar);

		BasicController bc(newCar);
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
		if (i == idCarToControl) {
			const std::vector<float>& inputs = localMap.processInput(carInput->getPlayerInputState(), cars[i]->getSpeedDir(),
																	 0.f, 0.f);
			cars[i]->handleInputs(inputs);
		} else {
			std::vector<float> inputs;
			inputs.resize(CarInput::ALL, 0);
			//TODO Add "basic AI" for the vehicles

			cars[i]->handleInputs(inputs);
		}
	}

	if (debugDraw) {
		debugDraw->setDebugMode(1);
		debugDraw->step();
	}

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
	return cars[0]->getPosition();
}

Ogre::Quaternion Sim::getCameraOrientation() {
	return cars[0]->getOrientation();
}

TerrainSurface* Sim::getTerrainSurface(std::string name) {
	return mApp->getTerrainSurface(name);
}

void Sim::keyPressed(const OIS::KeyEvent& ke) {
	carInput->keyPressed(ke);
}

void Sim::keyReleased(const OIS::KeyEvent& ke) {
	carInput->keyReleased(ke);
}
