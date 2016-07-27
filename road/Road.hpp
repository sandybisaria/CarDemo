#pragma once

#include "SplineBase.hpp"

#include <BulletCollision/CollisionShapes/btTriangleMesh.h>

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
	bool rebuildRoadGeometry();

	std::deque<RoadSeg> vSegs;

private:
	Ogre::SceneManager* mSceneMgr;
	Ogre::Terrain* mTerrain;

	Ogre::String roadMtr;

//---- Geometry
	Ogre::Real g_LenDim0; // Triangle dim in length
	Ogre::Real g_SkirtLen, g_SkirtH; // Skirt for hiding gaps

	void insert(insertPos ip);
	SplinePoint newP;

//---- Meshes
	std::vector<btTriangleMesh*> vbtTriMesh;

//---- Rebuilding road geometry
	struct DataRoad {
		int segs;
		int sMin, sMax;

		DataRoad() : segs(0), sMin(0), sMax(0) { }
	};

	void prepassRange(DataRoad& dr);
	void prepassAngles(DataRoad& dr);

	// Default LOD is 0
	struct DataLod0 {
		std::vector<int> v0_iL; // Length steps
		std::vector<Ogre::Real> v0_tc; // Texture coords
		std::vector<Ogre::Vector3> v0_N; // Normals
		std::vector<int> v0_Loop; // bool, inside loop

		void clear() {
			v0_iL.clear(); v0_tc.clear(); v0_N.clear(); v0_Loop.clear();
		}
	} DL0;

	// LOD; should be equivalent to DL0 in this case
	struct DataLod {
		std::vector<int> v_iL, v_iW; // Length and width steps for each seg
		std::vector<int> v_bMerge; // bool: 0 if merged, 1 if new

		std::vector<Ogre::Real> v_tc, v_len; // Total length
		std::vector<Ogre::Vector3> w_W; // Width dir

		std::vector<std::vector<int > > w_iWL; // Width steps per length point, for each seg
		std::vector<int> v_iwEq; // 1 if equal width steps at whole length, 0 has transition

		Ogre::Real tcLen; // Total texture coord length
		Ogre::Real sumLenMrg; // Total length to determine merging
		// Skipping mrgCnt;

		int lod, iLodDiv;
		Ogre::Real fLenDim;
		bool isLod0; // No isPace;

		DataLod()
			: tcLen(0.f), sumLenMrg(0.f),
			  lod(0), isLod0(true), iLodDiv(1), fLenDim(1.f) { }
	};

	void prepassLod(const DataRoad& dr, DataLod0& DL0, DataLod& dl, int lod);

	struct DataLodMesh {
		std::vector<Ogre::Vector4> clr;
		std::vector<Ogre::Vector3> pos, norm;
		std::vector<Ogre::Vector2> tcs;

		int iLmrg;

		DataLodMesh() : iLmrg(0) { };
		void clear() {
			iLmrg = 0;
			clr.clear(); pos.clear(); norm.clear(); tcs.clear();
		}
	};

//---- Segment building
	struct DataSeg {
		int seg, seg1, seg0;
		int idMtr;
		bool onTerr;
		// Skipped pipe and hasPlend and iwC and jfw's
	};

	void buildSeg(const DataRoad& dr, const DataLod0& DL0, DataLod& dl, DataLodMesh& dlm, DataSeg& ds, int segM);

//---- Destroy methods
	void destroyRoad();
	void destroySeg(int id);

//---- Originally in SplineEdit
	Ogre::Real getTerrH(Ogre::Vector3 p);
};