#pragma once

#include "../vdrift/MathVector.hpp"
#include "../vdrift/Quaternion.hpp"

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <cmath>

#define _USE_MATH_DEFINES

// vdrift to Ogre utility methods
struct Axes {
	static Ogre::Quaternion qFixCar, qFixWh;
	static void init();

	static void vectorToOgre(Ogre::Vector3& vOut, const MathVector<float, 3>& vIn);
	static Ogre::Vector3 vectorToOgre(const MathVector<float, 3>& vIn);

	static MathVector<float, 3> ogreToMath(const Ogre::Vector3 vIn);

	// For the car
	static Ogre::Quaternion flQuatToOgre(const Quaternion<float>& qIn);
	static Ogre::Quaternion doQuatToOgre(const Quaternion<double>& qIn);

	// For the wheels
	static Ogre::Quaternion flWhQuatToOgre(const Quaternion<float>& qIn);
	static Ogre::Quaternion doWhQuatToOgre(const Quaternion<double>& qIn);
};
