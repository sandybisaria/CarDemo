#include "States.hpp"

CompositeState::CompositeState(ControllerInterface *interface, double speed,
							   double angle)
	: BaseState(interface) {
	ssState = new StopSignState(mInterface, speed, angle);
	tlState = new TrafficLightState(mInterface, speed, angle);
}

BaseState* CompositeState::update(float dt) {
	if (ssState->isActive()) {
		// Right now, only the StopSignState touches the mActive bool
		tlState->update(dt);
		ssState->update(dt);
	} else {
		ssState->update(dt);
		tlState->update(dt);
	}

	return NULL;
}