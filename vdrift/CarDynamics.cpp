#include "CarDynamics.hpp"

void CarDynamics::alignWithGround() {
	updateWheelTransform();
	updateWheelContacts();
}

MathVector<double, 3> CarDynamics::localToWorld(const MathVector<double, 3>& local) const {
	MathVector<double, 3> position = local - centerOfMass;
	body.getOrientation().rotateVector(position);
	return position + body.getPosition();
}

MathVector<double, 3> CarDynamics::getWheelPosition(WheelPosition wp, double displacementPercent) const {
	MathVector<double, 3> localPos = getLocalWheelPosition(wp, displacementPercent);
	chassisRotation.rotateVector(localPos);
	return localPos + chassisPosition;
}

MathVector<double, 3> CarDynamics::getLocalWheelPosition(WheelPosition wp, double displacementPercent) const {
	const MathVector<double, 3>& wheelExt = wheels[wp].getExtendedPosition();
	const MathVector<double, 3> hinge = suspension[wp].getHinge();
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
	double toeRot = -suspension[wp].getToe() * M_PI / 180.0;
	if (wp % 2 == 0) toeRot = -toeRot;
	toe.rotate(toeRot, 0, 0, 1);

	return camber * toe * steer;
}

void CarDynamics::updateWheelTransform() {
	for (int i = 0; i < numWheels; i++) {
		wheelPos[i] = getWheelPositionAtDisplacement(WheelPosition(i), suspension[i].getDisplacementPercent());
		wheelRots[i] = getBodyOrientation() * getWheelSteeringAndSuspensionOrientation(WheelPosition(i));
	}
}

void CarDynamics::updateWheelContacts() {
	MathVector<float, 3> rayDir = getDownVector();
	for (int i = 0; i < numWheels; i++) {
		//TODO Implement and retrieve CollisionContact
		MathVector<float, 3> rayStart = localToWorld(wheels[i].getExtendedPosition());
		rayStart = rayStart - rayDir * wheels[i].getRadius();
		float rayLen = 1.5;

		world->castRay(rayStart, rayDir, rayLen, chassis, this, i, false); // False because we have car collisions TODO Update with CollisionContact
	}
}

MathVector<double, 3> CarDynamics::getDownVector() const {
	MathVector<double, 3> v(0, 0, -1);
	getBodyOrientation().rotateVector(v);
	return v;
}