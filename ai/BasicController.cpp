#include "BasicController.hpp"

#include "States.hpp"

BasicController::BasicController(Car* car)
	: mCar(car) {
	reset();

	// PID constants
	kPSpeed = 7.65629; kISpeed = 0.00656; kDSpeed = 0.00020;
	kPAngle = 443.75; kIAngle = 1; kDAngle = 1; // Could be refined further; may also be correlated with speed

	currentState = new ConstantState(this, 20, 0);
}

BasicController::~BasicController() {
	delete currentState;
}

void BasicController::reset() {
	inputs.resize(CarInput::ALL, 0.0f);

	// Speed variables
	iSpeedAcc = 0;
	dLastESpeed = 0;

	// Angle variables
	iAngleAcc = 0;
	dLastEAngle = 0;

//	// Point variables
//	isTargetPointEnabled = false;
}

void BasicController::setTargetSpeed(double newSpeed) {
	targetSpeed = newSpeed;
}

void BasicController::setTargetAngle(double newAngle, bool resetDir) {
	targetAngle = newAngle;

	if (resetDir) initDir = mCar->getForwardVector();
}

//void BasicController::setTargetPoint(MathVector<double, 2> newPoint) {
//	targetPoint = newPoint;
//	isTargetPointEnabled = true;
//}

const std::vector<double>& BasicController::updateInputs(float dt) {
	currentState->update(dt);

//	updatePointTargeting();
	updateSpeed(dt);
	updateDirection(dt);

	return inputs;
}

//void BasicController::turn(bool isLeft, double turnRadius) {
//	const double diagDist = turnRadius * sqrt(2.0);
//
//	MathVector<double, 2> forwardVector = toFlatVector(mCar->getForwardVector());
//	MathVector<double, 2> pointDir;
//
//	const double angle = (45.0 * M_PI / 180.0) * (isLeft ? 1.0 : -1.0); // Angle signs are inverted
//	pointDir[0] = (forwardVector[0] * cos(angle) - forwardVector[1] * sin(angle)) * diagDist;
//	pointDir[1] = (forwardVector[0] * sin(angle) + forwardVector[1] * cos(angle)) * diagDist;
//
//	MathVector<double, 2> carPos = toFlatVector(Axes::ogreToMath(mCar->getPosition()), false);
//	MathVector<double, 2> turnPoint = carPos + pointDir;
//
//	setTargetPoint(turnPoint);
//}

double BasicController::getAngle(MathVector<double, 2> fromDir, MathVector<double, 2> toDir) {
	double angle = acos(fromDir.dot(toDir));

	MathVector<double, 3> cross = MathVector<double, 3>(toDir[0], toDir[1], 0).cross(MathVector<double, 3>(fromDir[0], fromDir[1], 0));
	if (cross.dot(MathVector<double, 3>(0, 0, 1)) > 0) { // > 0 because angle sign is inverted
		angle = -angle; // Determine direction of angle (i.e. to the left or to the right of initDir)
	}

	return angle;
}

// "Flat" in that height is ignored; normalized by default
MathVector<double, 2> BasicController::toFlatVector(MathVector<double, 3> vec, bool normalize) {
	MathVector<double, 2> res(vec[0], vec[1]);
	return normalize ? res.normalized() : res;
}

//void BasicController::updatePointTargeting() {
//	if (isTargetPointEnabled) {
//		MathVector<double, 2> carPos = toFlatVector(Axes::ogreToMath(mCar->getPosition()), false);
//		MathVector<double, 2> pointDir = targetPoint - carPos;
//
//		// Maybe radius could be configured
//		if (pointDir.magnitude() < 1) {
//			isTargetPointEnabled = false;
//			targetAngle = 0;
//
//			//TODO In the future, may want another way to indicate arrival (or none at all)
//			std::cout << "Reached point " << targetPoint << std::endl;
//		} else {
//			double angle = getAngle(pointDir.normalized(), toFlatVector(mCar->getForwardVector()));
//			setTargetAngle(angle);
//		}
//	}
//}

void BasicController::updateSpeed(float dt) {
	const double eSpeed = targetSpeed - mCar->getSpeedMPS();

	// Simple PID controller
	double thrBrkVal = 0;
	thrBrkVal += eSpeed * kPSpeed; // Proportional term

	thrBrkVal += kISpeed * iSpeedAcc; // Integral term
	iSpeedAcc += eSpeed * dt;

	if (dt != 0) {
		thrBrkVal += kDSpeed * ((eSpeed - dLastESpeed) / dt); // Derivative term
		dLastESpeed = eSpeed;
	}

	thrBrkVal = clamp(thrBrkVal, -1.0, 1.0); // Clamp from -1 to 1
	if (thrBrkVal < 0) {
		inputs[CarInput::THROTTLE] = 0;
		inputs[CarInput::BRAKE] = -thrBrkVal;
	} else {
		inputs[CarInput::THROTTLE] = thrBrkVal;
		inputs[CarInput::BRAKE] = 0;
	}
}

void BasicController::updateDirection(float dt) {
	const double angle = getAngle(toFlatVector(mCar->getForwardVector()), toFlatVector(initDir));
	if (isnan(angle)) return; // Abandon ship (maybe should find out where a nan might occur...)

	const double eAngle = targetAngle - angle;

	double steerVal = 0;
	steerVal += eAngle * kPAngle; // Proportional term

	steerVal += kIAngle * iAngleAcc; // Integral term
	iAngleAcc += eAngle * dt;

	if (dt != 0) {
		steerVal += kDAngle * ((eAngle - dLastEAngle) / dt); // Derivative term
		dLastEAngle = eAngle;
	}

	steerVal = clamp(steerVal, -1.0, 1.0);
	if (steerVal < 0) {
		inputs[CarInput::STEER_RIGHT] = 0;
		inputs[CarInput::STEER_LEFT] = -steerVal;
	} else {
		inputs[CarInput::STEER_RIGHT] = steerVal;
		inputs[CarInput::STEER_LEFT] = 0;
	}
}