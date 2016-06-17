#include "CarPosInfo.hpp"

CarPosInfo::CarPosInfo()
	: src(0) {
}

void CarPosInfo::setSource(InfoSource* src) {
	this->src = src;
}

void CarPosInfo::update() {
	setPos(src->getPos());
}
