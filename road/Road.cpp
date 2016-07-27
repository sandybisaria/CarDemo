#include <Ogre.h>
#include "Road.hpp"

#include "../tinyxml/tinyxml2.h"

Road::Road() {
	//Init some variables
	setDefault();
}

void Road::setDefault() {
	//Reset some variables
	g_LenDim0 = 1.f;
	g_SkirtLen = 1.f; g_SkirtH = 0.12f;
}

void Road::setup(Ogre::Terrain* terrain, Ogre::SceneManager* sceneMgr) {
	mTerrain = terrain;
	mSceneMgr = sceneMgr;
}

bool Road::loadFile(Ogre::String fileName) {
	clear();

	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError e = doc.LoadFile(fileName.c_str());
	if (e != tinyxml2::XML_SUCCESS) { return false; }

	tinyxml2::XMLElement* root = doc.RootElement();
	if (!root) { return false; }

	tinyxml2::XMLElement* n = NULL;
	const char* a = NULL;

	setDefault();

	// Load material
	n = root->FirstChildElement("mtr");
	if (n) {
		a = n->Attribute("road"); // Valid road material names can be found in surfaces.cfg
		if (a) { roadMtr = Ogre::String(a); }
	}
	// Skipping wall/pipe/col materials

	n = root->FirstChildElement("dim");
	if (n) {
		a = n->Attribute("lenDim"); if (a) { g_LenDim0 = Ogre::StringConverter::parseReal(a); }
		a = n->Attribute("skirtLen"); if (a) { g_SkirtLen = Ogre::StringConverter::parseReal(a); }
		a = n->Attribute("skirtH"); if (a) { g_SkirtH = Ogre::StringConverter::parseReal(a); }
	}

	// Load road points
	n = root->FirstChildElement("P");
	while (n) {
		a = n->Attribute("pos"); newP.pos = Ogre::StringConverter::parseVector3(a);

		a = n->Attribute("w"); newP.width = Ogre::StringConverter::parseReal(a);

		a = n->Attribute("onTerr"); newP.onTerr = !(a && a[0] == '0'); // Road assumed to be on terrain unless otherwise noted

		a = n->Attribute("a"); // Yaw of the road (really, what direction it's pointing
		if (a) { newP.mYaw = Ogre::StringConverter::parseReal(a); }
		else { newP.mYaw = 0.f; }

		a = n->Attribute("ar"); // Roll of the road (really, its bank angle)
		if (a) { newP.mRoll = Ogre::StringConverter::parseReal(a); }
		else { newP.mRoll = 0.f; }

		a = n->Attribute("aT"); // Angle type
		if (a) { newP.aType = (AngType) Ogre::StringConverter::parseInt(a); }
		else { newP.aType = AT_Both; }

		insert(INS_End);

		n = n->NextSiblingElement("P");
	}

	newP.setDefault();

	return true;
}

bool Road::rebuildRoadGeometry() {
	// Assume that UpdRot() wouldn't do anything

	DataRoad dr;
	prepassRange(dr);

	destroyRoad();
	for (int seg = 0; seg < dr.segs; seg++) {
		RoadSeg rs; rs.empty = true;
		vSegs.push_back(rs);
	}

	prepassAngles(dr);

	DL0.clear();

	// LOD (will try to operate under a single LOD (zero); thus no for-loop
	DataLod dl;
	prepassLod(dr, DL0, dl, 0); // LOD of zero TODO FINISH

	DataLodMesh dlm;

	int sNum = dr.sMax - dr.sMin, segM = dr.sMin;
	while (sNum > 0) {
		DataSeg ds;

		buildSeg(dr, DL0, dl, dlm, ds, segM);

		sNum--;
		segM++;
	}
}

void Road::insert(insertPos ip) {
	RoadSeg rs;
	SplinePoint pt = newP;

	if (pt.onTerr) { pt.pos.y = getTerrH(pt.pos); } //TODO: Ignoring g_Height (road offset) for now

	//TODO: Implement other insertPos as needed
	if (ip == INS_End) {
		mP.push_back(pt);
		vSegs.push_back(rs);
	}

	recalculateTangents();
}

void Road::prepassRange(DataRoad& dr) {
	dr.segs = getNumPoints();
	dr.sMin = 0; dr.sMax = getNumPoints();

	// Do nothing else, since this will only be called when all points are added (iDirtyId == -1)
}

void Road::prepassAngles(DataRoad &dr) {
	if (dr.segs < 3) return;
	for (int seg = 0; seg < dr.segs; seg++) {
		int seg0 = getPrev(seg);

		if (mP.at(seg).aType == AT_Manual) {
			mP.at(seg).aYaw = mP.at(seg).mYaw;
			mP.at(seg).aRoll = mP.at(seg).mRoll;
		} else {
			mP.at(seg).aRoll = 0.f;

			const Ogre::Real dist = 0.1f;
			Ogre::Vector3 vl = getLenDir(seg, 0.f, dist) + getLenDir(seg0, 1.f - dist, 1.f);
			Ogre::Vector3 vw = Ogre::Vector3(vl.z, 0.f, -vl.x);

			mP.at(seg).aYaw = TerrUtil::getAngle(vw.x, vw.z) * 180.f / M_PI;

			if (mP.at(seg).aType == AT_Both) {
				mP.at(seg).aYaw += mP.at(seg).mYaw;
				mP.at(seg).aRoll += mP.at(seg).mRoll;
			}
		}
	}
}

// If adding additional LODs, refer to Road_Prepass.cpp
const int ciLodDivs[1] = {1};

void Road::prepassLod(const DataRoad &dr, DataLod0 &DL0, DataLod &dl, int lod) {
	dl.lod = lod;
	dl.isLod0 = (lod == 0);

	int iLodDiv = ciLodDivs[lod];
	dl.fLenDim = iLodDiv * g_LenDim0;
	dl.tcLen = 0.f;
	int inLoop = 0;

	for (int seg = 0; seg < dr.segs; seg++) {
		int seg1 = getNext(seg), seg0 = getPrev(seg);

		//TODO NOT DONE
	}
}

void Road::buildSeg(const DataRoad &dr, const DataLod0 &DL0, DataLod &dl, DataLodMesh &dlm, DataSeg &ds, int segM) {
	int seg = (segM + dr.segs) % dr.segs;
	int seg1 = getNext(seg), seg0 = getPrev(seg), seg02 = getAdd(seg, -2);
	ds.seg = seg; ds.seg1 = seg1; ds.seg0 = seg0;

	ds.onTerr = mP.at(seg).onTerr && mP.at(seg1).onTerr;

	// Skipping jump front walls

	// Assuming merging segs is on (as it is for game rebuilds)
	bool bNew = (segM   == dr.sMin) || dl.v_bMerge[seg ];
	bool bNxt = (segM+1 == dr.sMax) || dl.v_bMerge[seg1];

	if (bNew) { dlm.clear(); }

	RoadSeg& rs = vSegs[seg];
	if (!rs.empty && dl.isLod0) { destroySeg(seg); }

	// Materials
	int mId = mP.at(seg).idMtr;
	ds.idMtr = std::max(0, mId); // So far, only allowing one road material...
	rs.roadMtrStr = roadMtr + (ds.onTerr ? "_ter" : "");

	bool useSkirt = ds.onTerr;
	Ogre::Real skLen = useSkirt ? g_SkirtLen : 0.f, skH = useSkirt ? g_SkirtH : 0.f;

	// Skip iwC

	int il = dl.v_iL[seg];
	int il0 = DL0.v0_iL[seg];
	Ogre::Real la = 1.f / il, la0 = 1.f / il0, l = -la0;

	// Angles
	Ogre::Real ay1 = mP[seg].aYaw,  ay2 = mP[seg1].aYaw,  ay21 = ay2 - ay1;
	Ogre::Real ar1 = mP[seg].aRoll, ar2 = mP[seg1].aRoll, ar21 = ar2 - ar1;
	const Ogre::Real asw = 180;
	while (ay21 > asw)  ay21 -= 2 * asw;  while (ay21 < -asw)  ay21 += 2*asw;
	while (ar21 > asw)  ar21 -= 2 * asw;  while (ar21 < -asw)  ar21 += 2*asw;

	Ogre::Real tcBeg  = (seg > 0) ? dl.v_tc[seg-1]   : 0.f, tcEnd  = dl.v_tc[seg],   tcRng  = tcEnd  - tcBeg;
	Ogre::Real tcBeg0 = (seg > 0) ? DL0.v0_tc[seg-1] : 0.f, tcEnd0 = DL0.v0_tc[seg], tcRng0 = tcEnd0 - tcBeg0;
	Ogre::Real tcRmul = tcRng0 / tcRng;

	bool visible = mP.at(seg).idMtr >= 0;
	if (visible) {
		for (int i = -1; i <= il + 1; i++) {
			dlm.iLmrg++;

			Ogre::Vector3 vL0 = interpolate(seg, l);
			Ogre::Vector3 vl = getLenDir(seg, l, l + la), vw;
			Ogre::Real len = vl.length();
			vl.normalise();

			if (i <= 0) { dl.tcLen = 0; }
			Ogre::Real tc = dl.tcLen * tcRmul + tcBeg0;

			if (i == -1)     { tc = -skLen * tcRmul + tcBeg0; }
			if (i == il + 1) { tc =  skLen * tcRmul + tcEnd0; }

			if (mP.at(seg).onTerr && mP.at(seg1).onTerr) {
				vw = Ogre::Vector3(vl.z, 0, -vl.x);
			} else {
				Ogre::Real ay = ay1 + ay21 * l;
				Ogre::Real ar = ar1 + ar21 * l;
				vw = getRot(ay, ar);
			}
			vw.normalise();
			Ogre::Vector3 vwm = vw;

			Ogre::Real wiMul = interpolateWidth(seg, l);
			//TODO FINISH
		}
	}
}

void Road::destroyRoad() {
	for (int i = 0; i < vbtTriMesh.size(); i++) { delete vbtTriMesh[i]; }
	vbtTriMesh.clear();

	for (int seg = 0; seg < vSegs.size(); seg++) { destroySeg(seg); }
	vSegs.clear();

	// idStr = 0?
}

void Road::destroySeg(int id) {
	RoadSeg& rs = vSegs[id];
	if (rs.empty) return;

	// Assuming one LOD
	// Skipping walls, blends, cols

	rs.road.node->detachAllObjects();
	mSceneMgr->destroySceneNode(rs.road.node);
	Ogre::MeshManager::getSingleton().remove(rs.road.meshStr);

	rs.empty = true;
	// No rs.lpos
}

//---- (Former) SplineEdit methods
Ogre::Real Road::getTerrH(Ogre::Vector3 p) {
	return mTerrain ? mTerrain->getHeightAtWorldPosition(p.x, 0.f, p.z) : 0.f;
}