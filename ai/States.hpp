#pragma once

class ControllerInterface;

class BaseState {
public:
	virtual ~BaseState() { };
	virtual void update(float dt) { };
};

// Maintain a given velocity and direction
class ConstantState : public BaseState {
public:
	ConstantState(ControllerInterface* interface, double speed, double angle);
	virtual ~ConstantState();

	virtual void update(float dt);

private:
	ControllerInterface* mInterface;
	double mSpeed;
	double mAngle;
};