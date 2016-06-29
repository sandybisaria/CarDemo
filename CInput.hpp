#pragma once

#include "vdrift/CarConstants.hpp"

#include <OIS/OIS.h>

#include <map>

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
public:
	CInput(Sim* sim);

	float* getPlayerInputState() { return playerInputState; }

	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);

private:
	void loadInputDefaults();

	Sim* mSim;

	std::map< OIS::KeyCode, PlayerActions::PlayerActions > triggerInputMap;
	std::map< std::pair<OIS::KeyCode, OIS::KeyCode>, PlayerActions::PlayerActions > axisInputMap;

	float playerInputState[PlayerActions::NumPlayerActions];
};

