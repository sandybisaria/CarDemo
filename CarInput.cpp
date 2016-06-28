#include "CarInput.hpp"

CarInput::CarInput(Sim* sim)
	: mSim(sim) {
	for (int a = 0; a < PlayerActions::NumPlayerActions; a++) {
		playerInputState[a] = 0;
	}
}
