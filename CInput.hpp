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

// Based on Stuntrally's CInput class, receiving input from the keyboard and storing it
//TODO CInput and CarControlMap may possibly be combined if it makes sense
class CInput {
public:
	CInput(Sim* sim);

	double* getPlayerInputState() { return playerInputState; }

	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);

private:
	void loadInputDefaults();

	Sim* mSim; //TODO Remove; not needed

	std::map< OIS::KeyCode, PlayerActions::PlayerActions > triggerInputMap;
	std::map< std::pair<OIS::KeyCode, OIS::KeyCode>, PlayerActions::PlayerActions > axisInputMap;

	double playerInputState[PlayerActions::NumPlayerActions];
};

