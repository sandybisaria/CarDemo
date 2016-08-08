#include "States.hpp"

#include "../terrain/SceneObject.hpp"

StopSignState::StopSignState(ControllerInterface *interface, double speed,
							 double angle)
	: BaseState(interface), initSpeed(speed), initAngle(angle) {
	currState = new ConstantState(interface, initSpeed, initAngle);
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
			}
		}
	} else if (stage == ST_BRAKE) {
		if (fabs(mInterface->getCarSpeed()) < 0.1) {
			stage = ST_WAIT; countdown = 2;
		}
	} else if (stage == ST_WAIT) {
		countdown -= dt;
		if (countdown <= 0) {
			stage = ST_DRIVE;
			currState = new ConstantState(mInterface, initSpeed, initAngle);
		}
	}

	return currState->update(dt);
}