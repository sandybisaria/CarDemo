#pragma once

#include "SplineBase.hpp"

#include <Terrain/OgreTerrain.h>
#include <OgreSceneManager.h>

struct RoadSeg {
	struct SegData {
		Ogre::SceneNode* node;
		Ogre::Entity* ent;
		Ogre::MeshPtr mesh;
		Ogre::String meshStr;

		SegData() : node(0), ent(0), meshStr("") { }
	};

	// Ignoring LOD (levels of detail)
	SegData road;
	Ogre::String roadMtrStr;
	int mtrId;

	bool empty;

	RoadSeg() : empty(true), mtrId(0) { }
};

enum insertPos { INS_Begin, INS_Before_Cur, INS_Cur, INS_End };

class Road : public SplineBase {
public:
	Road();
	void setDefault();

	void setup(Ogre::Terrain* terrain, Ogre::SceneManager* sceneMgr);

	bool loadFile(Ogre::String fileName);

	std::deque<RoadSeg> vSegs;

private:
	Ogre::SceneManager* mSceneMgr;
	Ogre::Terrain* mTerrain;

	Ogre::String roadMtr;

	void insert(insertPos ip);
	SplinePoint newP;

//---- Originally in SplineEdit
	Ogre::Real getTerrH(Ogre::Vector3 p);
};