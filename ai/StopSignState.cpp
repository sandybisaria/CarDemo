#include "States.hpp"

StopSignState::StopSignState(ControllerInterface *interface, double speed,
							 double angle)
	: BaseState(interface), initSpeed(speed), initAngle(angle) {
	currState = new ConstantState(mInterface, initSpeed, initAngle);
	stage = ST_DRIVE; lastStopSign = "";
}

BaseState* StopSignState::update(float dt) {
	if (stage == ST_DRIVE) {
		std::list<SceneObject*> sceneObjs = mInterface->getNearbySceneObjects();
		if (sceneObjs.size() == 1) {
			SceneObject* object = sceneObjs.back();
			if ((object->getType() == "StopSign") &&
				(object->getName() != lastStopSign)) {
				lastStopSign = object->getName();
				stage = ST_BRAKE;

				delete currState;
				currState = new ConstantState(mInterface, 0, initAngle);
				mActive = true; // Override other state behavior
			}
		}
	} else if (stage == ST_BRAKE) {
		if (fabs(mInterface->getCarSpeed()) < 0.1) {
			stage = ST_WAIT; countdown = 3; //TODO Should be a const member? Or should the StopSign itself tell how long to wait?
		}
	} else if (stage == ST_WAIT) {
		countdown -= dt;
		if (countdown <= 0) {
			stage = ST_DRIVE;
			currState = new ConstantState(mInterface, initSpeed, initAngle);
			mActive = false;
		}
	}

	return currState->update(dt);
}