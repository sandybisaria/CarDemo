#include "Car.hpp"

Car::Car() {
	setNumWheels(4); // Assume a four-wheeled car
}

Car::~Car() {

}

void setup() {

}

void Car::setNumWheels(int n) {
	numWheels = n;
}
