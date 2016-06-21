#include "Car.hpp"

Car::Car()
	: mId(0), mCarModel(0) {
	setNumWheels(DEF_WHEELS);
}

Car::~Car() {

}

bool Car::load(std::string carType, CONFIGFILE& cf, const MATHVECTOR<double, 3> pos, const QUATERNION<double> rot,
		  	   int id) {
	int nw = 0;
	cf.GetParam("wheels", nw);
	if (nw >= MIN_WHEELS && nw <= MAX_WHEELS)
		setNumWheels(nw);

	mCarType = carType;
	mId = id;

	// Car collision parameters
	// com
	cd.comOfsL = 0.f;  cf.GetParam("collision.com_ofs_L", cd.comOfsL);  //|
	cd.comOfsH = 0.f;  cf.GetParam("collision.com_ofs_H", cd.comOfsH);
	// dim
	cd.collR   = 0.3f;  cf.GetParam("collision.radius", cd.collR);
	cd.collR2m = 0.6f;  cf.GetParam("collision.radius2mul", cd.collR2m);
	cd.collH   = 0.45f; cf.GetParam("collision.height", cd.collH);
	cd.collW   = 0.5f;  cf.GetParam("collision.width",  cd.collW);
	// ofs
	cd.collLofs = 0.f;  cf.GetParam("collision.offsetL", cd.collLofs);
	cd.collWofs = 0.f;  cf.GetParam("collision.offsetW", cd.collWofs);
	cd.collHofs = 0.f;  cf.GetParam("collision.offsetH", cd.collHofs);
	cd.collLofs -= cd.comOfsL;
	cd.collHofs -= cd.comOfsH;
	// L
	cd.collPosLFront = 1.9f; cf.GetParam("collision.posLfront", cd.collPosLFront);
	cd.collPosLBack = -1.9f; cf.GetParam("collision.posLrear",  cd.collPosLBack);
	// w
	cd.collFrWMul  = 0.2f;   cf.GetParam("collision.FrWmul",  cd.collFrWMul);
	cd.collFrHMul  = 1.0f;   cf.GetParam("collision.FrHmul",  cd.collFrHMul);
	cd.collTopWMul = 0.8f;   cf.GetParam("collision.TopWmul", cd.collTopWMul);
	// Top L pos
	cd.collTopFr    = 0.4f;  cf.GetParam("collision.TopFr",    cd.collTopFr);
	cd.collTopMid   =-0.3f;  cf.GetParam("collision.TopMid",   cd.collTopMid);
	cd.collTopBack  =-1.1f;  cf.GetParam("collision.TopBack",  cd.collTopBack);
	// Top h mul
	cd.collTopFrHm  = 0.2f;  cf.GetParam("collision.TopFrHm",  cd.collTopFrHm);
	cd.collTopMidHm = 0.4f;  cf.GetParam("collision.TopMidHm", cd.collTopMidHm);
	cd.collTopBackHm= 0.2f;  cf.GetParam("collision.TopBackHm",cd.collTopBackHm);

	cd.collFriction = 0.4f;  cf.GetParam("collision.friction",  cd.collFriction);
	cd.collFlTrigH = 0.f;   cf.GetParam("collision.fluidTrigH",cd.collFlTrigH);
	cd.collFlTrigH -= cd.comOfsH;

	if (!cd.load(cf))
		return false;

	MATHVECTOR<double, 3> initPos;
	QUATERNION<double> initRot; initRot = rot;

//	float stOfsY = 0.f;
//	cf.GetParam("collision.start-offsetY", stOfsY);
//		pos[2] += stOfsY -0.4/**/ + cd.com_ofs_H;

//	posAtStart = posLastCheck = pos;
//	rotAtStart = rotLastCheck = rot;
//	dmgLastCheck = 0.f;

	cd.init(pos, rot);
	//TODO Add ABS and TCS methods to CarDynamics

	return true;
}

void Car::setNumWheels(int n) {
	numWheels = n;
}
