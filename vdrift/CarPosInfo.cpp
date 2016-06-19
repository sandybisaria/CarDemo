#include "CarPosInfo.hpp"

CarPosInfo::CarPosInfo()
	: src(0) {
	setNumWheels(DEF_WHEEL_COUNT);
}

void CarPosInfo::setSource(InfoSource* src) {
	this->src = src;
}

void CarPosInfo::setNumWheels(int nw) {
	numWheels = nw;
	wheelPos.resize(nw);
	wheelRot.resize(nw);
}

void CarPosInfo::update() {
	pos = src->getPos();
	rot = src->getRot();
	wheelPos = src->getWheelPos();
}
