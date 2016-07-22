#pragma once

#include "assert.h"

template <typename T>
class Optional {
public:
	Optional() : value(T()), isValid(false) { }
	Optional(const T val) : value(val), isValid(true) { }

	bool isInit() const { return isValid; }

	const T get() const { assert(isInit()); return value; }
		  T get()		{ assert(isInit()); return value; }

	T getOrDefault(T defaultVal) { return isInit() ? value : defaultVal; }

private:
	T value;
	bool isValid;
};