#include "CInput.hpp"

CInput::CInput(Sim* sim)
	: mSim(sim) {
	for (int a = 0; a < PlayerActions::NumPlayerActions; a++) {	playerInputState[a] = 0; }

	loadInputDefaults();
}

void CInput::keyPressed(const OIS::KeyEvent& ke) {
	for (std::map<OIS::KeyCode, PlayerActions::PlayerActions>::const_iterator i = triggerInputMap.begin();
		 i != triggerInputMap.end(); i++) {
		if (ke.key == i->first)	playerInputState[i->second] = 1;
	}

	for (std::map<std::pair<OIS::KeyCode, OIS::KeyCode>, PlayerActions::PlayerActions>::const_iterator i = axisInputMap.begin();
		 i != axisInputMap.end(); i++) {
		const std::pair<OIS::KeyCode, OIS::KeyCode>& pair = i->first;
			 if (ke.key == pair.first)  playerInputState[i->second] -= 0.5;
		else if (ke.key == pair.second) playerInputState[i->second] += 0.5;
	}
}

void CInput::keyReleased(const OIS::KeyEvent& ke) {
	for (std::map<OIS::KeyCode, PlayerActions::PlayerActions>::const_iterator i = triggerInputMap.begin();
		 i != triggerInputMap.end(); i++) {
		if (ke.key == i->first)	playerInputState[i->second] = 0;
	}

	for (std::map<std::pair<OIS::KeyCode, OIS::KeyCode>, PlayerActions::PlayerActions>::const_iterator i = axisInputMap.begin();
		 i != axisInputMap.end(); i++) {
		const std::pair<OIS::KeyCode, OIS::KeyCode>& pair = i->first;
			 if (ke.key == pair.first)  playerInputState[i->second] += 0.5;
		else if (ke.key == pair.second) playerInputState[i->second] -= 0.5;
	}
}

void CInput::loadInputDefaults() {
	triggerInputMap.insert(std::make_pair(OIS::KC_UP, 	 PlayerActions::THROTTLE));
	triggerInputMap.insert(std::make_pair(OIS::KC_DOWN,  PlayerActions::BRAKE));
	triggerInputMap.insert(std::make_pair(OIS::KC_SPACE, PlayerActions::HANDBRAKE));
	triggerInputMap.insert(std::make_pair(OIS::KC_Z, 	 PlayerActions::SHIFT_UP));
	triggerInputMap.insert(std::make_pair(OIS::KC_X, 	 PlayerActions::SHIFT_DOWN));

	axisInputMap.insert(std::make_pair(std::make_pair(OIS::KC_LEFT, OIS::KC_RIGHT), PlayerActions::STEERING));
	playerInputState[PlayerActions::STEERING] = 0.5;
}
