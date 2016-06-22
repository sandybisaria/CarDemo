#pragma once

#include "CarConstants.hpp"

#include <vector>

class CarControlMapLocal {
public:
	CarControlMapLocal() {
		for (int i = 0; i < 8; i++) {
			grUpOld[i] = grDnOld[i] = false;
		}
		reset();
	}

	void reset() {
		inputs.resize(CarInput::ALL, 0.0f);
	}

	// Speed sensitive steering (decrease steer angle range with higher speed)
	// carSpeed in m/s
	static float getSSSCoeff(float carSpeed, float sssVelFactor, float sssEffect) {
		float coeff = 1.0f;
		float carMPH = carSpeed * 2.23693629f;

		if (carMPH > 1.0f) {
			coeff = std::max(1.f - sssEffect, 1.f - sssVelFactor * carSpeed * 0.02f);
		}

		return coeff;
	}

	//TODO Implement processInputs, likely after reviewing CInput.h

private:
	std::vector<float> inputs; // Indexed by CarInput values

	// Shift (gear up/down)
	bool grUpOld[8], grDnOld[8];
};
