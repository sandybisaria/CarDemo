#pragma once

#include <string>

enum WHEEL_POSITION
{
	FRONT_LEFT,	FRONT_RIGHT,
	REAR_LEFT,	REAR_RIGHT,
	REAR2_LEFT, REAR2_RIGHT,
	REAR3_LEFT, REAR3_RIGHT
};
static void getWheelPosStr(int i, int numWheels, WHEEL_POSITION& wl, WHEEL_POSITION& wr, std::string& pos) {
	if (numWheels == 2) {
		if (i==0){	wl = wr = FRONT_LEFT;   pos = "front";  } else
		if (i==1){	wl = wr = FRONT_RIGHT;  pos = "rear";   }
	} else {
		if (i==0){	wl = FRONT_LEFT;  wr = FRONT_RIGHT;  pos = "front";  } else
		if (i==1){	wl = REAR_LEFT;   wr = REAR_RIGHT;   pos = "rear";   } else
		if (i==2){	wl = REAR2_LEFT;  wr = REAR2_RIGHT;  pos = "rear2";  } else
		if (i==3){	wl = REAR3_LEFT;  wr = REAR3_RIGHT;  pos = "rear2";  }
	}
}

const int MIN_WHEELS = 2;
const int DEF_WHEELS = 4;
const int MAX_WHEELS = 8;
const int MAX_CARS = 16;
const int CarPosCnt = 8;  // size of poses queue

enum VehicleType
{
	V_Car, V_Spaceship, V_Sphere
};

namespace SURFACE  {
enum CARSURFACETYPE
{
	NONE, ASPHALT, GRASS, GRAVEL, CONCRETE, SAND, COBBLES
};  }

namespace CARINPUT  {
enum CARINPUT
{
	//actual car inputs that the car uses
	THROTTLE, 	BRAKE,  BOOST, FLIP,
  	HANDBRAKE, 	CLUTCH, //-
  	STEER_LEFT,	STEER_RIGHT,
 	SHIFT_UP, 	SHIFT_DOWN,
 	PREV_CAM, NEXT_CAM,
 	LAST_CHK, REWIND,
 	//ABS_TOGGLE, TCS_TOGGLE, START_ENGINE,
	ALL
};  }

enum EPerfTest {PT_StartWait, PT_Accel, PT_Brake};

const int Ncrashsounds = 12, Nwatersounds = 3;

#ifdef _WIN32
static bool isnan(float number)  {  return (number != number);  }
static bool isnan(double number) {  return (number != number);  }
#endif

const static char sCfgWh[MAX_WHEELS][4] = {"FL","FR","RL","RR","RL2","RR2","RL3","RR3"};  // .car config wheel names

const int PAR_BOOST = 2, PAR_THRUST = 4;  // max particles for boost and spc thrusters

// v2 .car files store arrays in different order..
static void convertV2to1(float& x, float& y, float& z) {
	float tx = x, ty = y, tz = z;
	x = ty;  y = -tx;  z = tz;
}

// Methods from Axes.h, BECAUSE THEY WON'T WORK IN THEIR OWN FILE FOR SOME UNKNOWN REASON
