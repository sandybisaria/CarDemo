#include "SplineBase.hpp"

//---- TerrUtil methods
float TerrUtil::getAngle(float x, float y) {
	if (x == 0.f && y == 0.f) { return 0.f; }

	if (y == 0.f) { return (x < 0.f) ? (float) M_PI : 0.f; }

	return (y < 0.f) ? atan2f(-y, x) : (float) M_PI*2 - atan2f(y, x);
}

float TerrUtil::getAngleAt(Ogre::Terrain *terr, float x, float z, float s) {
	Ogre::Real y0 = 0;
	Ogre::Vector3 vx(x-s, y0, z), vz(x, y0, z-s);
	Ogre::Vector3 vX(x+s, y0, z), vZ(x, y0, z+s);

	vx.y = terr->getHeightAtWorldPosition(vx);  vX.y = terr->getHeightAtWorldPosition(vX);
	vz.y = terr->getHeightAtWorldPosition(vz);  vZ.y = terr->getHeightAtWorldPosition(vZ);

	Ogre::Vector3 v_x = vx-vX;  //v_x.normalise();
	Ogre::Vector3 v_z = vz-vZ;  //v_z.normalise();

	Ogre::Vector3 n = -v_x.crossProduct(v_z);  n.normalise();
	Ogre::Real a = Ogre::Math::ACos(n.y).valueDegrees();
	return a;
}

Ogre::Vector3 TerrUtil::getNormalAt(Ogre::Terrain *terr, float x, float z, float s) {
	Ogre::Real y0 = 0;
	Ogre::Vector3 vx(x-s, y0, z), vz(x, y0, z-s);
	Ogre::Vector3 vX(x+s, y0, z), vZ(x, y0, z+s);

	vx.y = terr->getHeightAtWorldPosition(vx);  vX.y = terr->getHeightAtWorldPosition(vX);
	vz.y = terr->getHeightAtWorldPosition(vz);  vZ.y = terr->getHeightAtWorldPosition(vZ);

	Ogre::Vector3 v_x = vx-vX;  v_x.normalise();
	Ogre::Vector3 v_z = vz-vZ;  v_z.normalise();

	Ogre::Vector3 n = -v_x.crossProduct(v_z);  n.normalise();
	return n;
}

//---- SplinePoint methods
SplinePoint::SplinePoint() {
	setDefault();
}

void SplinePoint::setDefault() {
	pos = Ogre::Vector3::ZERO; tan = Ogre::Vector3::ZERO;
	width = 7;
	mYaw = 0; mRoll = 0;
	aYaw = 0; aRoll = 0;
	aType = AT_Both;
	onTerr = 1;
	idMtr = 0;
	notReal = false;
}

//---- SplineBase methods
SplineBase::SplineBase()
	: isLooped(false) {
}

void SplineBase::setPos(int index, Ogre::Vector3 value) {
	mP.at(index).pos = value;
	recalculateTangents();
}

Ogre::Vector3 SplineBase::interpolate(int id, Ogre::Real t) const{
	int id1 = getNext(id);
	Ogre::Vector3 p1 = getPos(id), p2 = getPos(id1);

		 if (t == 0.f) return p1;
	else if (t == 1.f) return p2;

	Ogre::Vector3 n1 = mP.at(id).tan, n2 = mP.at(id1).tan;

	// Hermite polynomial
	Ogre::Real t2 = t*t, t3 = t2*t;
	Ogre::Vector4 tm(t3* 2 + t2*-3 + 1, t3*-2 + t2* 3, t3 + t2*-2 + t, t3 - t2);

	return Ogre::Vector3(
		tm.x*p1.x + tm.y*p2.x + tm.z*n1.x + tm.w*n2.x,
		tm.x*p1.y + tm.y*p2.y + tm.z*n1.y + tm.w*n2.y,
		tm.x*p1.z + tm.y*p2.z + tm.z*n1.z + tm.w*n2.z);
}

Ogre::Real SplineBase::interpolateWidth(int id, Ogre::Real t) const {
	int id1 = getNext(id);
	Ogre::Real p1 = mP.at(id).width, p2 = mP.at(id1).width;

	if (t == 0.f) return p1;
	else if (t == 1.f) return p2;

	Ogre::Real n1 = mP.at(id).wtan, n2 = mP.at(id1).wtan;

	Ogre::Real t2 = t*t, t3 = t2*t;
	Ogre::Vector4 tm(t3* 2 + t2*-3 + 1, t3*-2 + t2* 3, t3 + t2*-2 + t, t3 - t2);
	return tm.x*p1 + tm.y*p2 + tm.z*n1 + tm.w*n2;
}

void SplineBase::recalculateTangents() {
	// Catmull-Rom approach
	int num = mP.size();
	if (num < 2) { return; }

	for (int i = 0; i < num; ++i) {
		int next = getNext(i), prev = getPrev(i);
		mP.at(i).tan  = 0.5f * (mP.at(next).pos   - mP.at(prev).pos  );
		mP.at(i).wtan = 0.5f * (mP.at(next).width - mP.at(prev).width);
	}
}

Ogre::Real SplineBase::getSegLen(int seg) {
	const int lenQ = 5; // Quality determined by # of iterations

	Ogre::Real len = 0;
	Ogre::Vector3 p0;
	for (int i = 0; i <= lenQ; ++i) {
		Ogre::Vector3 p = interpolate(seg, Ogre::Real(i) / lenQ);
		if (i > 0) {
			Ogre::Vector3 l = p - p0;
			len += l.length();
		}
		p0 = p;
	}
	return len;
}

Ogre::Vector3 SplineBase::getLenDir(int seg, Ogre::Real l, Ogre::Real la) {
	Ogre::Vector3 vL0 = interpolate(seg, l);
	Ogre::Vector3 vL1 = interpolate(seg, la);
	return vL1 - vL0;
}

Ogre::Vector3 SplineBase::getRot(Ogre::Real aYaw, Ogre::Real aRoll) {
	Ogre::Real ay = aYaw * M_PI / 180.f, ar = aRoll * M_PI / 180.f;
	Ogre::Real cb = cosf(ar);
	return Ogre::Vector3(cosf(ay)*cb, sinf(ar), -sinf(ay)*cb);
}