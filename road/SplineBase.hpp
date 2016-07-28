#pragma once

#include <OgreVector3.h>
#include <Terrain/OgreTerrain.h>

#include <deque>

// Helper methods
class TerrUtil {
public:
	static float getAngle(float x, float y);
	static float getAngleAt(Ogre::Terrain* terr, float x, float z, float s);
	static Ogre::Vector3 getNormalAt(Ogre::Terrain* terr, float x, float z, float s);
};

enum AngType {AT_Manual = 0, AT_Auto, AT_Both, AT_ALL };

// Point/variable
class SplinePoint {
public:
	SplinePoint();
	void setDefault();

	Ogre::Vector3 pos, tan; // Position, computed tangent
	Ogre::Real width, wtan; // Width of road

	Ogre::Real mYaw, mRoll; // Manual angles (if not auto)
	Ogre::Real aYaw, aRoll; // Working angles (from auto)
	AngType aType;

	bool onTerr;
	int idMtr; // Material ID

	bool notReal; // True means only for decoration, or point move (not a real road to be driven on)
	bool hidden() { return idMtr == -1 && !notReal; } // Real yet hidden
};


// SplineBase - only interpolation
class SplineBase {
public:
	SplineBase();
	~SplineBase() { }

//---- Points
	void clear() { mP.clear(); }
	int getNumPoints() const { return (int) mP.size(); }

	int getPrev(int id) const {
		int s = getNumPoints();
		return isLooped ? (id - 1 + s) % s : std::max(0, id - 1);
	}
	int getNext(int id) const {
		int s = getNumPoints();
		return isLooped ? (id + 1) % s : std::min(s - 1, id + 1);
	}
	int getAdd(int id, int n) const {
		int s = getNumPoints();
		return isLooped ? (id + n + s) % s : std::min(s - 1, std::max(0, id + n));
	}

	SplinePoint& getPoint(int index) { return mP.at(index); }

//---- Position
	Ogre::Vector3 getPos(int index) const { return mP.at(index).pos; }
	void setPos(int index, Ogre::Vector3 value);

//---- Interpolation
	Ogre::Vector3 interpolate(int id, Ogre::Real t) const; // Get value at single spline segment (t = 0..1)
	Ogre::Real interpolateWidth(int id, Ogre::Real t) const; // Interpolate one-dimensional variables (t = 0..1)

	void recalculateTangents();

//---- Direction, length, rotation
	Ogre::Real getSegLen(int seg);
	Ogre::Vector3 getLenDir(int seg, Ogre::Real l, Ogre::Real la);
	static Ogre::Vector3 getRot(Ogre::Real aYaw, Ogre::Real aRoll);

protected:
	bool isLooped; // Is this a closed loop?

	std::deque<SplinePoint> mP; // Spline points
	static std::deque<SplinePoint> mPcopy; // Copy points
};