#pragma once

#include <string>

const int MIN_WHEEL_COUNT = 2;
const int DEF_WHEEL_COUNT = 4;
const int MAX_WHEEL_COUNT = 8;

// When loading from .car files
const std::string WHEEL_TYPE[MAX_WHEEL_COUNT] = { "FL","FR","RL","RR","RL2","RR2","RL3","RR3" };
