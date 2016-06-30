#include "Sim.hpp"

Sim::Sim(App* app)
	: mSceneMgr(0), scene(0), mApp(app),
	  world(0), car(0), carInput(0),
	  frameRate(60), targetTime(0), frame(0),
	  debugDraw(NULL) {
}

Sim::~Sim() {
	delete world;
	delete carInput;
	delete debugDraw;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	world = new CollisionWorld();
	world->sim = this; //TODO Maybe make more secure?

	car = new Car(0);
	car->setup("360", mSceneMgr, *world);

	carInput = new CInput(this);

	// Debug drawing
	debugDraw = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), world->getDynamicsWorld());
	world->getDynamicsWorld()->setDebugDrawer(debugDraw);
	world->getDynamicsWorld()->getDebugDrawer()->setDebugMode(1);
}

void Sim::update(float dt) {
	car->updatePreviousVelocity(); // Was in the loop for fluids...
	if (dt > 0) world->update(dt); // Update physics
	car->update(dt); // Update model

	const std::vector<float>& inputs = localMap.processInput(carInput->getPlayerInputState(), car->getSpeedDir(),
															 0.f, 0.f);
	car->handleInputs(inputs, dt);

	if (debugDraw) {
		debugDraw->setDebugMode(1);
		debugDraw->step();
	}

	//TODO How Stuntrally updates the game... however, this does not work
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
//		frame++;
//
//		// Advance game logic
//		car->updatePreviousVelocity(); // Was in the loop for fluids...
//		if (dt > 0) world->update(tickPeriod); // Update physics
//		car->update(tickPeriod); // Update model
//		//TODO Handle inputs
//
//		curTicks++;
//		targetTime -= tickPeriod;
//	}
}

TerrainSurface* Sim::getTerrainSurface(std::string name) { return mApp->getTerrainSurface(name); }

void Sim::keyPressed(const OIS::KeyEvent& ke) {
	carInput->keyPressed(ke);
}

void Sim::keyReleased(const OIS::KeyEvent& ke) {
	carInput->keyReleased(ke);
}
