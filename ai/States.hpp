#pragma once

class BasicController;

class BaseState {
public:
	virtual ~BaseState() { };
	virtual void update(float dt) { };
};

// Constant velocity and direction
class IdleState : public BaseState {
public:
	IdleState(BasicController* controller);
	virtual ~IdleState() { };

	virtual void update(float dt);

private:
	BasicController* mController;
};