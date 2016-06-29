#pragma once

class Sim;

namespace PlayerActions {
	enum PlayerActions {
		THROTTLE, BRAKE,
		STEERING, HANDBRAKE,
//		BOOST, FLIP,
		SHIFT_UP, SHIFT_DOWN,
//		PREV_CAM, NEXT_CAM,
//		LAST_CHK, REWIND,
		NumPlayerActions
	};
}

// Based on Stuntrally's CInput class
class CInput {
private:
	Sim* mSim;

	float playerInputState[PlayerActions::NumPlayerActions];

public:
	CInput(Sim* sim);
};

