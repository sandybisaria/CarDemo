#include "Sim.hpp"

Sim::Sim()
	: mSceneMgr(0),
	  world(0), car(0),
	  frameRate(60), targetTime(0), frame(0) {
}

Sim::~Sim() {
	delete world;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	world = new CollisionWorld();
	world->sim = this; //TODO Maybe make more secure?

	car = new Car(0);
	car->setup("360", mSceneMgr, *world);
}

void Sim::update(float dt) {
	car->updatePreviousVelocity(); // Was in the loop for fluids...
	if (dt > 0) world->update(dt); // Update physics
	car->update(dt); // Update model
	return;

	const float minFPS = 10.f; // Minimum acceptable fps
	const unsigned int maxTicks = (int) (1.f / (minFPS * frameRate));
	const float maxTime = 1.f / minFPS;
	unsigned int curTicks = 0;

	// Throw away wall clock time if needed to maintain framerate
	if (dt > maxTime) dt = maxTime;

	targetTime += dt;
	double tickPeriod = frameRate;

	// Increment game logic by as many tick periods necessary
	while (targetTime > tickPeriod && curTicks < maxTicks) {
		frame++;

		// Advance game logic
		car->updatePreviousVelocity(); // Was in the loop for fluids...
		if (dt > 0) world->update(tickPeriod); // Update physics
		car->update(tickPeriod); // Update model
		//TODO Handle inputs

		curTicks++;
		targetTime -= tickPeriod;
	}
}
