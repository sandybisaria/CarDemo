#pragma once

#include "vdrift/dbl.h"
#include "vdrift/mathvector.h"
#include "vdrift/quaternion.h"

#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace Axes {
	static Ogre::Quaternion qFixCar, qFixWh;

	static void init() {
		Ogre::Quaternion qr;  {
		QUATERNION<double> fix;  fix.Rotate(PI_d, 0, 1, 0);
		qr.w = fix.w();  qr.x = fix.x();  qr.y = fix.y();  qr.z = fix.z();  qFixCar = qr;  }
		QUATERNION<double> fix;  fix.Rotate(PI_d/2, 0, 1, 0);
		qr.w = fix.w();  qr.x = fix.x();  qr.y = fix.y();  qr.z = fix.z();  qFixWh = qr;
	}

	static void toOgre(Ogre::Vector3& vOut, const MATHVECTOR<float,3>& vIn) {
		vOut.x = vIn[0];  vOut.y = vIn[2];  vOut.z = -vIn[1];
	}

//	Ogre::Vector3 toOgre(const MATHVECTOR<float,3>& vIn) {
//		return Ogre::Vector3(vIn[0], vIn[2], -vIn[1]);
//	}

	// Car
//	Ogre::Quaternion toOgreF(const QUATERNION<float>& vIn) {
//		Ogre::Quaternion q(vIn[0], -vIn[3], vIn[1], vIn[2]);
//		return q * qFixCar;
//	}
	static Ogre::Quaternion toOgreD(const QUATERNION<double>& vIn) {
		Ogre::Quaternion q(vIn[0], -vIn[3], vIn[1], vIn[2]);
		return q * qFixCar;
	}

	//  wheels
//	Quaternion Axes::toOgreW(const QUATERNION<half>& vIn)
//	{
//		Quaternion q(vIn[0], -vIn[3], vIn[1], vIn[2]);
//		return q * qFixWh;
//	}
//	Ogre::Quaternion toOgreWF(const QUATERNION<float>& vIn) {
//		Ogre::Quaternion q(vIn[0], -vIn[3], vIn[1], vIn[2]);
//		return q * qFixWh;
//	}
	static Ogre::Quaternion toOgreWD(const QUATERNION<double>& vIn) {
		Ogre::Quaternion q(vIn[0], -vIn[3], vIn[1], vIn[2]);
		return q * qFixWh;
	}
}
