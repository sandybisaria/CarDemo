#include "States.hpp"

TrafficLightState::TrafficLightState(ControllerInterface *interface,
									 double speed, double angle)
	: BaseState(interface), initSpeed(speed), initAngle(angle), lastLight(0) {
	currState = new ConstantState(mInterface, initSpeed, initAngle);
	lastStatus = TL_GREEN;
}

BaseState* TrafficLightState::update(float dt) {
	if (lastLight == NULL) {
		std::list<SceneObject*> sceneObjs = mInterface->getNearbySceneObjects();
		if (sceneObjs.size() == 1) {
			SceneObject* object = sceneObjs.back();
			if (object->getType() == "TrafficLight") {
				lastLight = (TrafficLight*) object;
				checkLight();
			}
		}
	} else { checkLight(); }

	return currState->update(dt);
}

void TrafficLightState::checkLight() {
	TrafficLightStatus status = lastLight->getStatus();
	if (status != lastStatus) {
		if (status == TL_GREEN) {
			delete currState;
			currState = new ConstantState(mInterface, initSpeed, initAngle);

			lastLight = NULL; // Don't need to remember the light
		}
		else if (lastStatus == TL_GREEN) {
			delete currState;
			currState = new ConstantState(mInterface, 0, initAngle);
		}

		lastStatus = status;
	} else if (status == TL_GREEN) {
		// Undo storing the light...
		lastLight = NULL;
	}
}