#include "CInput.hpp"

CInput::CInput() {
	for (int a = 0; a < PlayerActions::NumPlayerActions; a++) {
		playerInputState[a] = 0;
	}

	loadInputDefaults();
}

void CInput::keyPressed(const OIS::KeyEvent& ke) {
	// Store all trigger inputs
	std::map<OIS::KeyCode,
		PlayerActions::PlayerActions>::iterator i = triggerInputMap.begin();
	for (; i != triggerInputMap.end(); i++) {
		if (ke.key == i->first)	playerInputState[i->second] = 1;
	}

	// Store all "axis" inputs
	std::map<std::pair<OIS::KeyCode, OIS::KeyCode>,
		PlayerActions::PlayerActions>::iterator j = axisInputMap.begin();
	for (; j != axisInputMap.end(); j++) {
		const std::pair<OIS::KeyCode, OIS::KeyCode>& pair = j->first;
			 if (ke.key == pair.first)  playerInputState[j->second] -= 0.5;
		else if (ke.key == pair.second) playerInputState[j->second] += 0.5;
	}
}

void CInput::keyReleased(const OIS::KeyEvent& ke) {
	// Store all trigger inputs
	std::map<OIS::KeyCode,
		PlayerActions::PlayerActions>::iterator i = triggerInputMap.begin();
	for (; i != triggerInputMap.end(); i++) {
		if (ke.key == i->first)	playerInputState[i->second] = 0;
	}

	// Store all "axis" inputs
	std::map<std::pair<OIS::KeyCode, OIS::KeyCode>,
		PlayerActions::PlayerActions>::iterator j = axisInputMap.begin();
	for (; j != axisInputMap.end(); j++) {
		const std::pair<OIS::KeyCode, OIS::KeyCode>& pair = j->first;
			 if (ke.key == pair.first)  playerInputState[j->second] += 0.5;
		else if (ke.key == pair.second) playerInputState[j->second] -= 0.5;
	}
}

void CInput::loadInputDefaults() {
	triggerInputMap.insert(std::make_pair(
		OIS::KC_UP,    PlayerActions::THROTTLE));
	triggerInputMap.insert(std::make_pair(
		OIS::KC_DOWN,  PlayerActions::BRAKE));
	triggerInputMap.insert(std::make_pair(
		OIS::KC_SPACE, PlayerActions::HANDBRAKE));
	triggerInputMap.insert(std::make_pair(
		OIS::KC_X, 	   PlayerActions::SHIFT_UP));
	triggerInputMap.insert(std::make_pair(
		OIS::KC_Z, 	   PlayerActions::SHIFT_DOWN));

	// For an "axis" input, the neutral value is 0.5
	// The "left" button drops it to 0, the "right" button to 1
	axisInputMap.insert(std::make_pair(std::make_pair(
		OIS::KC_LEFT, OIS::KC_RIGHT), PlayerActions::STEERING));
	playerInputState[PlayerActions::STEERING] = 0.5;
}
