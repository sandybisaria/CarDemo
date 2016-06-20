#pragma once

#include <string>

const int MIN_WHEEL_COUNT = 2;
const int DEF_WHEEL_COUNT = 4;
const int MAX_WHEEL_COUNT = 8;

// When loading from .car files
const std::string WHEEL_TYPE[MAX_WHEEL_COUNT] = { "FL","FR","RL","RR","RL2","RR2","RL3","RR3" };

namespace CARINPUT {
	// "Actual" car inputs that the car uses
	enum CARINPUT {
		THROTTLE, BRAKE, BOOST, FLIP,
		HANDBRAKE, CLUTCH,
		STEER_LEFT, STEER_RIGHT,
		SHIFT_UP, SHIFT_DOWN,
		PREV_CAM, NEXT_CAM,
//		LAST_CHK, REWIND,
		ALL
	};
}

enum EPerfTest { PT_StartWait, PT_Accel, PT_Brake };
