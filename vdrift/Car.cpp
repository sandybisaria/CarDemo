#include "Car.hpp"

Car::Car()
	: mId(0), mCarModel(0) {
	setNumWheels(DEF_WHEELS);
}

Car::~Car() {

}

void Car::setNumWheels(int n) {
	numWheels = n;
}
