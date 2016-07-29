#pragma once

#include "vdrift/CarConstants.hpp"

#include <OIS/OIS.h>

#include <map>

namespace PlayerActions {
	// Certain actions from Stuntrally were uncommented due to lack of relevance
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

// Based on Stuntrally's CInput class (CInput means "car input")
// Receives and stores keyboard inputs
class CInput {
public:
	CInput();

	double* getPlayerInputState() { return playerInputState; }

	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);

private:
	void loadInputDefaults();

	std::map< OIS::KeyCode, PlayerActions::PlayerActions > triggerInputMap;
	std::map< std::pair<OIS::KeyCode, OIS::KeyCode>, PlayerActions::PlayerActions > axisInputMap;

	double playerInputState[PlayerActions::NumPlayerActions];
};

