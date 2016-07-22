#pragma once

#include <OgreVector3.h>
#include <Terrain/OgreTerrain.h>

#include <deque>

// Helper methods
class TerUtil {
	static float getAngle(float x, float y);
	static float 		  getAngleAt(Ogre::Terrain* terr, float x, float z, float s);
	static Ogre::Vector3 getNormalAt(Ogre::Terrain* terr, float x, float z, float s);
};

enum AngType {AT_Manual = 0, AT_Auto, AT_Both, AT_ALL };

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
//	int cols;
//	int onPipe;
//	int loop;

//	Ogre::Real pipe;
	int idMtr; // Material ID

	bool notReal; // True means only for decoration, or point move (not a real road to be driven on)
	bool hidden() { return idMtr == -1 && !notReal; } // Real yet hidden
};

class SplineBase {
public:
	SplineBase();
	~SplineBase();

	void clear();
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

protected:
	bool isLooped; // Is this a closed loop?

	std::deque<SplinePoint> mP; // Spline points
	static std::deque<SplinePoint> mPcopy; // Copy points
};