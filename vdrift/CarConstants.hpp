#pragma once

#include <string>
#include <algorithm>

const int MIN_WHEEL_COUNT = 2;
const int DEF_WHEEL_COUNT = 4;
const int MAX_WHEEL_COUNT = 8;

// When loading from .car files
const std::string WHEEL_TYPE[MAX_WHEEL_COUNT] = { "FL","FR","RL","RR","RL2","RR2","RL3","RR3" };
enum WheelPosition {
	FRONT_LEFT,	FRONT_RIGHT,
	REAR_LEFT,	REAR_RIGHT,
	REAR2_LEFT, REAR2_RIGHT,
	REAR3_LEFT, REAR3_RIGHT
};

namespace CarInput {
	// "Actual" car inputs that the car uses
	enum CarInput {
		THROTTLE, BRAKE,
//		BOOST, FLIP,
		HANDBRAKE, CLUTCH,
		STEER_LEFT, STEER_RIGHT,
		SHIFT_UP, SHIFT_DOWN,
//		PREV_CAM, NEXT_CAM,
//		LAST_CHK, REWIND,
		ALL
	};
}

static void versionConvert(float& x, float& y, float& z) {
	float tx = x, ty = y, tz = z;
	x = ty;  y = -tx;  z = tz;
}

static void getWheelPosStr(int i, int numWheels, WheelPosition& wl, WheelPosition& wr, std::string& pos) {
	if (numWheels == 2) {
			   if (i == 0) { wl = wr = FRONT_LEFT;				pos = "front";
		} else if (i == 1) { wl = wr = FRONT_RIGHT;				pos = "rear";
		}
	} else {
			   if (i == 0) { wl = FRONT_LEFT; wr = FRONT_RIGHT; pos = "front";
		} else if (i == 1) { wl = REAR_LEFT;  wr = REAR_RIGHT;  pos = "rear";
		} else if (i == 2) { wl = REAR2_LEFT; wr = REAR2_RIGHT; pos = "rear2";
		} else if (i == 3) { wl = REAR3_LEFT; wr = REAR3_RIGHT; pos = "rear2";
		}
	}
}

static double clamp(double val, double min, double max) { return std::max(std::min(val, max), min); }
