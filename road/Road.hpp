#pragma once

#include "SplineBase.hpp"

#include <BulletCollision/CollisionShapes/btTriangleMesh.h>

#include <Terrain/OgreTerrain.h>
#include <OgreSceneManager.h>

class Sim;

struct RoadSeg {
	struct SegData {
		Ogre::SceneNode* node;
		Ogre::Entity* ent;
		Ogre::MeshPtr mesh;
		Ogre::String meshStr;

		SegData() : node(0), ent(0), meshStr("") { }
	};

	// Single LOD
	SegData road;
	Ogre::String roadMtrStr;
	int mtrId;

	std::vector<Ogre::Vector3> lpos; // Points for LOD dist

	bool empty;

	RoadSeg() : empty(true), mtrId(0) { }
};

enum insertPos { INS_Begin, INS_Before_Cur, INS_Cur, INS_End };

class Road : public SplineBase {
public:
	Road(Sim* sim);
	void setDefault();

	void setup(Ogre::Terrain* terrain, Ogre::SceneManager* sceneMgr);
	bool loadFile(Ogre::String fileName);
	bool rebuildRoadGeometry();

	std::deque<RoadSeg> vSegs;

	int idStr;

private:
	Sim* mSim;

	Ogre::SceneManager* mSceneMgr;
	Ogre::Terrain* mTerrain;

	Ogre::String roadMtr;

//---- Geometry
	Ogre::Real g_LenDim0; // Triangle dim in length
	int g_iWidthDiv0; // Width divisions (a constant for road, except pipes..)
	Ogre::Real g_SkirtLen, g_SkirtH; // Skirt for hiding gaps

	Ogre::Real g_tcMul; // Texture coord multiplier (scale) per unit length

	Ogre::Real g_Height; // Geometry above terrain global (for each point)

	Ogre::Real g_MergeLen; // Length below which segments are merged
	Ogre::Real g_LodPntLen; // Length between LOD points

	void insert(insertPos ip);
	SplinePoint newP;

//---- Meshes
	void createMesh(Ogre::SubMesh* subMesh, Ogre::AxisAlignedBox& aaBox,
					const std::vector<Ogre::Vector3>& pos, const std::vector<Ogre::Vector3>& norm,
					const std::vector<Ogre::Vector2>& tcs, const std::vector<Ogre::uint16>& idx, Ogre::String sMtrName);

	void addMesh(Ogre::MeshPtr mesh, Ogre::String sMesh, const Ogre::AxisAlignedBox& aaBox, Ogre::Entity** pEnt,
				 Ogre::SceneNode** pNode, Ogre::String sEnd);

	std::vector<Ogre::uint16> idx, idxB; // Mesh indices

	std::vector<Ogre::Vector3> posBt; // For Bullet trimesh
	std::vector<btTriangleMesh*> vbtTriMesh;

	const std::vector<Ogre::Vector3>* atPos;

	int at_ilBt;

	// Add triangle to Bullet (with index check)
	void addTri(int f1, int f2, int f3, int i);

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
		std::vector<Ogre::Vector3> v_W; // Width dir

		std::vector<std::vector<int > > v_iWL; // Width steps per length point, for each seg
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
		// Skipped clr (color)
		std::vector<Ogre::Vector3> pos, norm, posLod;
		std::vector<Ogre::Vector2> tcs;

		int iLmrg;

		DataLodMesh() : iLmrg(0) { };
		void clear() {
			iLmrg = 0;
			pos.clear(); norm.clear(); tcs.clear();
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
	void createSegMeshes(const DataLod& dl, const DataLodMesh& dlm, DataSeg& ds, RoadSeg& rs);
	void createSegCollision(const DataLodMesh& dlm, const DataSeg& ds);

//---- Destroy methods
	void destroyRoad();
	void destroySeg(int id);

//---- Originally in SplineEdit
	Ogre::Real getTerrH(Ogre::Vector3 p);
};