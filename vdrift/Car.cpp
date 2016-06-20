#include "Car.hpp"

Car::Car() {
	setNumWheels(DEF_WHEELS);
}

Car::~Car() {

}

void Car::setNumWheels(int n) {
	numWheels = n;
}
