#include "CarDynamics.hpp"

void CarDynamics::alignWithGround() {
	updateWheelTransform();
	updateWheelContacts();
}

double CarDynamics::getSpeedDir() const {
	// Car body vel in local car dir?
	double vel = body.getVelocity().dot(getForwardVector());
	return fabs(vel);
}

Quaternion<double> CarDynamics::getWheelOrientation(WheelPosition wp) const {
	Quaternion<double> sideRot;
	if (wp == FRONT_RIGHT || wp == REAR_RIGHT)
		sideRot.rotate(M_PI, 0, 0, 1);

	return chassisRotation * getWheelSteeringAndSuspensionOrientation(wp) * wheels[wp].getOrientation() * sideRot;
}

MathVector<double, 3> CarDynamics::getWheelPosition(WheelPosition wp) const {
	MathVector<double, 3> localPos = getLocalWheelPosition(wp, suspension[wp].getDisplacementPercent());
	chassisRotation.rotateVector(localPos);
	return localPos + chassisPosition;
}

MathVector<double, 3> CarDynamics::getWheelPosition(WheelPosition wp, double displacementPercent) const {
	MathVector<double, 3> localPos = getLocalWheelPosition(wp, displacementPercent);
	chassisRotation.rotateVector(localPos);
	return localPos + chassisPosition;
}

MathVector<double, 3> CarDynamics::getLocalWheelPosition(WheelPosition wp, double displacementPercent) const {
	MathVector<double, 3> wheelExt = wheels[wp].getExtendedPosition();
	MathVector<double, 3> hinge = suspension[wp].getHinge();
	MathVector<double, 3> relWheelExt = wheelExt - hinge;
	MathVector<double, 3> up(0, 0, 1);
	MathVector<double, 3> rotAxis = up.cross(relWheelExt.normalized());

	double hingeRadius = relWheelExt.magnitude();
	double travel = suspension[wp].getTravel();
	double displacement = displacementPercent * travel;
	double displacementRad = displacement / hingeRadius;

	Quaternion<double> hingeRotate;
	hingeRotate.rotate(-displacementRad, rotAxis[0], rotAxis[1], rotAxis[2]);
	MathVector<double, 3> localWheelPos = relWheelExt;
	hingeRotate.rotateVector(localWheelPos);

	return localWheelPos + hinge;
}

// World-space position of wheel center when suspension is compressed by the displacement percent (1.0 = fully compressed)
MathVector<double, 3> CarDynamics::getWheelPositionAtDisplacement(WheelPosition wp, double displacementPercent) const {
	return localToWorld(getLocalWheelPosition(wp, displacementPercent));
}

// Orientation of wheel solely due to steering and suspension
Quaternion<double> CarDynamics::getWheelSteeringAndSuspensionOrientation(WheelPosition wp) const {
	Quaternion<double> steer;
	steer.rotate(-wheels[wp].getSteerAngle() * M_PI / 180.0, 0, 0, 1);

	Quaternion<double> camber;
	double camberRot = -suspension[wp].getCamber() * M_PI / 180.0;
	if (wp % 2 == 1) camberRot = -camberRot;
	camber.rotate(camberRot, 1, 0, 0);

	Quaternion<double> toe;
	double toeRot = suspension[wp].getToe() * M_PI / 180.0;
	if (wp % 2 == 0) toeRot = -toeRot;
	toe.rotate(toeRot, 0, 0, 1);

	return camber * toe * steer;
}

MathVector<double, 3> CarDynamics::localToWorld(const MathVector<double, 3>& local) const {
	MathVector<double, 3> position = local - centerOfMass;
	getBodyOrientation().rotateVector(position);
	return position + body.getPosition();
}

void CarDynamics::updateWheelTransform() {
	for (int i = 0; i < numWheels; i++) {
		WheelPosition wp; wp = WheelPosition(i);
		wheelPos[i] = getWheelPositionAtDisplacement(wp, suspension[i].getDisplacementPercent());
		wheelRots[i] = getBodyOrientation() * getWheelSteeringAndSuspensionOrientation(wp);
	}
}

void CarDynamics::updateWheelVelocity() {
	for (int i = 0; i < numWheels; i++) {
		wheelVels[i] = body.getVelocity(wheelPos[i] - body.getPosition());
	}
}

MathVector<double, 3> CarDynamics::getDownVector() const {
	MathVector<double, 3> v(0, 0, -1);
	getBodyOrientation().rotateVector(v);
	return v;
}

MathVector<double, 3> CarDynamics::getForwardVector() const {
	MathVector<double, 3> v(1, 0, 0);
	getBodyOrientation().rotateVector(v);
	return v;
}
