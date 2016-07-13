#include "States.hpp"

WaypointState::WaypointState(ControllerInterface *interface, MathVector<double, 2> waypoint, double radius)
	: mInterface(interface), mWaypoint(waypoint), mRadius(radius) {
}

BaseState* WaypointState::update(float dt) {
	MathVector<double, 2> vecToPoint = mWaypoint - mInterface->getCarPosition();

	if (vecToPoint.magnitude() < mRadius) {
		return new ConstantState(mInterface, mInterface->getCarSpeed(), 0);
	} else {
		double angle = mInterface->getAngle(vecToPoint.normalized(), mInterface->getCarDirection());
		mInterface->setTargetAngle(angle);

		return NULL;
	}
}