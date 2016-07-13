#pragma once

class BasicController;

class BaseState {
public:
	virtual ~BaseState() { };
	virtual void update(float dt) { };
};

// Maintain a given velocity and direction
class ConstantState : public BaseState {
public:
	ConstantState(BasicController* controller, double speed, double angle);
	virtual ~ConstantState() { };

	virtual void update(float dt);

private:
	BasicController* mController;
	double mSpeed;
	double mAngle;
};