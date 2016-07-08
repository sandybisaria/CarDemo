#pragma once

#include "CarConstants.hpp"
#include <vector>
#include <cassert>

#include "../CInput.hpp"

// Class for mapping keyboard inputs to valid car control inputs
class CarControlMapLocal {
public:
	CarControlMapLocal() {
		grUpOld = grDnOld = false; //TODO Only one player
		reset();
	}

	void reset() { inputs.resize(CarInput::ALL, 0.0f); }

	// Speed sensitive steering (decrease steer angle range with higher speed)
	// carSpeed in m/s
	static double getSSSCoeff(double carSpeed, double sssVelFactor, double sssEffect) {
		double coeff = 1.0f;
		double carMPH = carSpeed * 2.23693629f;

		if (carMPH > 1.0f) { coeff = std::max(1.f - sssEffect, 1.f - sssVelFactor * carSpeed * 0.02f); }

		return coeff;
	}

	// Skipped player int, race countdowns params, performance test params
	const std::vector<double>& processInput(const double* channels, double carSpeed, double sssEffect,
											double sssVelFactor) {
		assert(inputs.size() == CarInput::ALL);

		// Throttle and brake
		double thr = channels[PlayerActions::THROTTLE], brk = channels[PlayerActions::BRAKE];
		//Assume oneAxisThrBrk (one axis for both) is false! Likely to be the default setting
		inputs[CarInput::THROTTLE] = thr;
		const double deadzone = 0.0001f; // Sensible dead-zone for braking
		inputs[CarInput::BRAKE] = brk < deadzone ? 0.f : brk;
		inputs[CarInput::HANDBRAKE] = channels[PlayerActions::HANDBRAKE];

		// Steering
		double val = channels[PlayerActions::STEERING] * 2.f - 1.f;
		if (sssEffect > 0.02f) val *= getSSSCoeff(fabs(carSpeed), sssVelFactor, sssEffect);
		inputs[CarInput::STEER_RIGHT] = val > 0.f ?  val : 0.f;
		inputs[CarInput::STEER_LEFT]  = val < 0.f ? -val : 0.f;

		// Shift
		bool grUp = (bool) channels[PlayerActions::SHIFT_UP], grDn = (bool) channels[PlayerActions::SHIFT_DOWN];
		inputs[CarInput::SHIFT_UP]   = grUp && !grUpOld;
		inputs[CarInput::SHIFT_DOWN] = grDn && !grDnOld;
		grUpOld = grUp;  grDnOld = grDn;

		return inputs;
	}

private:
	std::vector<double> inputs; // Indexed by CarInput values

	// Shift (gear up/down)
	bool grUpOld, grDnOld; //For only one player
};
