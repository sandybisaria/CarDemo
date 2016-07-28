#include "Road.hpp"

#include "../Sim.hpp"

#include "../tinyxml/tinyxml2.h"
#include "../terrain/RenderConst.hpp"

#include <Ogre.h>

Road::Road(Sim* sim)
	: idStr(0),
      mSim(sim) {
	//Init some variables
	setDefault();
}

void Road::setDefault() {
	//Reset some variables
	g_LenDim0 = 1.f; g_iWidthDiv0 = 8;
	g_SkirtLen = 1.f; g_SkirtH = 0.12f;
	g_tcMul = 0.1f;
	g_Height = 0.1f;
	g_MergeLen = 180.f; g_LodPntLen = 10.f;

	isLooped = false; // Assume that roads are NOT looped!
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
		// Valid road material names can be found in surfaces.cfg
		a = n->Attribute("road"); if (a) { roadMtr = Ogre::String(a); }
	}
	// Skipping wall/pipe/col materials

	n = root->FirstChildElement("dim");
	if (n) {
		a = n->Attribute("lenDim"); if (a) { g_LenDim0 = Ogre::StringConverter::parseReal(a); }
		a = n->Attribute("widthSteps"); if (a) { g_iWidthDiv0 = Ogre::StringConverter::parseInt(a); }

		a = n->Attribute("tcMul"); if (a) { g_tcMul = Ogre::StringConverter::parseReal(a); }

		a = n->Attribute("heightOfs"); if (a) { g_Height = Ogre::StringConverter::parseReal(a); }
	}

	n = root->FirstChildElement("mrg");
	if (n) {
		a = n->Attribute("skirtLen"); if (a) { g_SkirtLen = Ogre::StringConverter::parseReal(a); }
		a = n->Attribute("skirtH"); if (a) { g_SkirtH = Ogre::StringConverter::parseReal(a); }

		a = n->Attribute("mergeLen"); if (a) { g_MergeLen = Ogre::StringConverter::parseReal(a); }
		a = n->Attribute("lodPntLen"); if (a) { g_LodPntLen = Ogre::StringConverter::parseReal(a); }

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
	prepassLod(dr, DL0, dl, 0); // LOD of zero

	DataLodMesh dlm;

	int sNum = dr.sMax - dr.sMin, segM = dr.sMin;
	while (sNum > 0) {
		DataSeg ds;

		buildSeg(dr, DL0, dl, dlm, ds, segM);

		sNum--;
		segM++;
	}
}

void Road::update() {
	// Refer to SplineRoad::UpdLodVis()

	for (size_t seg = 0; seg < vSegs.size(); seg++) {
		RoadSeg& rs = vSegs.at(seg);
		if (rs.empty) { continue; }

		// Only one LOD
		rs.road.ent->setVisible(true);
	}
}

void Road::destroy() {
	destroyRoad();
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

void Road::createMesh(Ogre::SubMesh *subMesh, Ogre::AxisAlignedBox &aaBox, const std::vector<Ogre::Vector3> &pos,
					  const std::vector<Ogre::Vector3> &norm, const std::vector<Ogre::Vector2> &tcs,
					  const std::vector<Ogre::uint16> &idx, Ogre::String sMtrName) {
	size_t i, si = pos.size();
	if (si == 0) { return; } // Error

	subMesh->useSharedVertices = false;
	subMesh->vertexData = new Ogre::VertexData;
	subMesh->vertexData->vertexStart = 0;
	subMesh->vertexData->vertexCount = si;

	Ogre::VertexDeclaration* decl = subMesh->vertexData->vertexDeclaration;
	size_t offset = 0;
	offset += decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION).getSize();
	offset += decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL).getSize();
	offset += decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES).getSize();
	// Skipping clr

	Ogre::HardwareVertexBufferSharedPtr vbuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
		decl->getVertexSize(0), si, Ogre::HardwareBuffer::HBU_STATIC);
	float* vp = static_cast<float*> (vbuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));

	// No clr
	for (i = 0; i < si; i++) {
		const Ogre::Vector3& p = pos[i];
		*vp++ = p.x;  *vp++ = p.y;  *vp++ = p.z; aaBox.merge(p);
		const Ogre::Vector3& n = norm[i];
		*vp++ = n.x;  *vp++ = n.y;  *vp++ = n.z;
		*vp++ = tcs[i].x;  *vp++ = tcs[i].y;
	}
	vbuffer->unlock();
	subMesh->vertexData->vertexBufferBinding->setBinding(0, vbuffer);

	// Index
	Ogre::IndexData* id = subMesh->indexData;
	id->indexCount = idx.size(); id->indexStart = 0;
	id->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
		Ogre::HardwareIndexBuffer::IT_16BIT, id->indexCount, Ogre::HardwareBuffer::HBU_STATIC);
	Ogre::uint16* ip = static_cast<Ogre::uint16*> (id->indexBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));

	for (i = 0; i < idx.size(); i++) { *ip++ = idx[i]; }

	id->indexBuffer->unlock();
	subMesh->setMaterialName(sMtrName);
}

void Road::addMesh(Ogre::MeshPtr mesh, Ogre::String sMesh, const Ogre::AxisAlignedBox &aaBox, Ogre::Entity **pEnt,
				   Ogre::SceneNode **pNode, Ogre::String sEnd) {
	mesh->_setBounds(aaBox);
	mesh->_setBoundingSphereRadius((aaBox.getMaximum() - aaBox.getMinimum()).length() / 2.f);
	mesh->load();

	unsigned short src, dest;
	if (!mesh->suggestTangentVectorBuildParams(Ogre::VES_TANGENT, src, dest)) {
		mesh->buildTangentVectors(Ogre::VES_TANGENT, src, dest);
	}

	*pEnt = mSceneMgr->createEntity("rd.ent" + sEnd, sMesh);
	*pNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("rd.node" + sEnd);
	(*pNode)->attachObject(*pEnt);
	(*pEnt)->setVisible(false); (*pEnt)->setCastShadows(false);
	(*pEnt)->setVisibilityFlags(RV_Road);
}

void Road::addTri(int f1, int f2, int f3, int i) {
	idx.push_back(f1); idx.push_back(f2); idx.push_back(f3);

	// Assume blendTri == false (no blending)

	// bltTri assumed to be true
	if (i > 0 && i < at_ilBt) {
		posBt.push_back((*atPos)[f1]);
		posBt.push_back((*atPos)[f2]);
		posBt.push_back((*atPos)[f3]);
	}
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

	for (int seg = 0; seg < dr.segs; seg++) {
		int seg1 = getNext(seg), seg0 = getPrev(seg);

		Ogre::Real sp = 0, sp1 = 0, sp0 = 0; // Pipes don't exist;
		Ogre::Real p = 1.f, pl = 1.f;
		bool pipe = false;

		// Road
		int iw = std::max(1, (int) (g_iWidthDiv0 / iLodDiv)); // p is 1
		dl.v_iW.push_back(iw);
		int iwl = std::max(1, (int) (g_iWidthDiv0 / iLodDiv)); // pl is 1, too

		// Length steps
		Ogre::Real len = getSegLen(seg);
		int il = int(len / dl.fLenDim) / iwl * iwl + iwl;
		Ogre::Real lenAdd = 1.f / il;

		dl.v_iL.push_back(il);
		dl.v_len.push_back(len);

		// No stats

		// Merge conditions
		dl.sumLenMrg += len;
		// Material should never change, so ignore case
		if (mP.at(seg).onTerr != mP.at(seg1).onTerr || mP.at(seg).onTerr != mP.at(seg0).onTerr) {
			dl.sumLenMrg = 0.f;
			dl.v_bMerge.push_back(1);
		} else if (dl.sumLenMrg >= g_MergeLen) {
			dl.sumLenMrg -= g_MergeLen;
			dl.v_bMerge.push_back(1);
		} else {
			dl.v_bMerge.push_back(0);
		}

		if (dl.isLod0) {
			DL0.v0_iL.push_back(il);
			DL0.v0_Loop.push_back(0); // inLoop will probably be false (no loop-de-loops in this sim)
		}

		// Length dir
		Ogre::Vector3 vl = getLenDir(seg, 0, lenAdd), vw; vl.normalise();
		Ogre::Real ay = mP.at(seg).aYaw, ar = mP.at(seg).aRoll;

		// Width dir
		if (mP.at(seg).onTerr && mP.at(seg1).onTerr) { vw = Ogre::Vector3(vl.z, 0, -vl.x); }
		else										 { vw = getRot(ay, ar); }

		// Normal dir
		if (dl.isLod0) {
			Ogre::Vector3 vn = vl.crossProduct(vw); vn.normalise();
			DL0.v0_N.push_back(vn);
		}

		{
			Ogre::Real wiMul = mP.at(seg).width;
			vw *= wiMul;
			dl.v_W.push_back(vw);
		}

		Ogre::Real l = 0.f;
		for (int i = 0; i < il; i++) {
			Ogre::Vector3 vl = getLenDir(seg, l, l + lenAdd);
			l += lenAdd; dl.tcLen += vl.length();
		}
		dl.v_tc.push_back(dl.tcLen);
		if (dl.isLod0) { DL0.v0_tc.push_back(dl.tcLen); }
	}

	for (int seg = 0; seg < dr.segs; seg++) {
		int seg1 = getNext(seg);
		int il = dl.v_iL[seg];
		std::vector<int> viwL;

		// Width steps per length point in current segment
		int iw0 = dl.v_iW[seg], iw1 = dl.v_iW[seg1];
		for (int i = -1; i <= il+1; ++i) {
			int ii = std::max(0, std::min(il, i));
			int iw = iw0 + (int) (Ogre::Real(ii) / Ogre::Real(il) * (iw1 - iw0));
			viwL.push_back(iw);
		}
		int eq = iw1 == iw0 ? 1 : 0;

		dl.v_iwEq.push_back(eq);
		dl.v_iWL.push_back(viwL);
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
			vw *= wiMul;

			bool onTerr1 = ds.onTerr || mP.at(seg).onTerr && i == 0 || mP.at(seg1).onTerr && i == il;

			Ogre::Vector3 vn;
				 if (i == 0)  { vn = DL0.v0_N[seg ]; }
			else if (i == il) { vn = DL0.v0_N[seg1]; }
			else {
				vn = vl.crossProduct(vw);
				vn.normalise();
			}

			int iw = dl.v_iWL[seg][i+1];

			// Skip pipe amount vars

			Ogre::Vector3 vH0, vH1;
			int w0 = 0, w1 = iw;

			Ogre::Real tcL = tc * g_tcMul;

			for (int w = 0; w <= iw; w++) {
				Ogre::Vector3 vP, vN;
				Ogre::Real tcw = Ogre::Real(w) / Ogre::Real(iw);

				Ogre::Real yTer = 0.f;

				// Assuming fPipe == 0.f (no pipe)
				vP = vL0 + vw * (tcw - 0.5);
				vN = vn;
				yTer = getTerrH(vP);

				if (onTerr1) {
					vP.y = yTer + g_Height * ((w == 0 || w == iw) ? 0.15f : 1.f);
					vN = mTerrain ? TerrUtil::getNormalAt(mTerrain, vP.x, vP.z, dl.fLenDim * 0.5f) : Ogre::Vector3::UNIT_Y;
				}

				if (i == -1 || i == il + 1) { vP -= vn * skH; }

				// Must be visible if in this for-loop
				Ogre::Vector2 vtc(tcw * 1.f, tcL);
				dlm.pos.push_back(vP);
				dlm.norm.push_back(vN);
				dlm.tcs.push_back(vtc);

				if (w == w0) { vH0 = vP; }
				if (w == w1) { vH1 = vP; }
			}

			// Skip walls and columns

			l += (i == -1 || i == il) ? la0 : la;
			dl.tcLen += len;
		}
	}

	if (dl.isLod0) {
		int lps = std::max(2, (int) (dl.v_len[seg] / g_LodPntLen));

		for (int p = 0; p <= lps; p++) {
			Ogre::Vector3 vp = interpolate(seg, Ogre::Real(p) / Ogre::Real(lps));
			dlm.posLod.push_back(vp);
		}
	}

	if (bNxt && !dlm.pos.empty()) {
		createSegMeshes(dl, dlm, ds, rs);

		if (dl.isLod0) {
			for (int p = 0; p < dlm.posLod.size(); p++) { rs.lpos.push_back(dlm.posLod[p]); }
			dlm.posLod.clear();
		}

		createSegCollision(dlm, ds);
	}
}

void Road::createSegMeshes(const DataLod &dl, const DataLodMesh &dlm, DataSeg &ds, RoadSeg &rs) {
	Ogre::String sEnd = Ogre::StringConverter::toString(idStr); idStr++;
	Ogre::String sMesh = "rd.mesh." + sEnd; // No meshes for walls, etc.

	posBt.clear(); idx.clear(); idxB.clear();
	atPos = &dlm.pos; at_ilBt = dlm.iLmrg - 2;
	int seg = ds.seg, seg1 = ds.seg1, seg0 = ds.seg0;

	int iiw = 0;

	// Equal width
	if (dl.v_iwEq[seg] == 1) {
		for (int i = 0; i < dlm.iLmrg - 1; i++) {
			int iw = dl.v_iW[seg];
			for (int w = 0; w < iw; w++) {
				int f0 = iiw + w, f1 = f0 + (iw + 1);
				addTri(f0+0,f1+1,f0+1,i);
				addTri(f0+0,f1+0,f1+1,i);
			}
			iiw += iw + 1;
		}
	} else {
		// Pipe transition only? TODO?
		std::cout << "Should have done this" << std::endl;
	}

	// Skipped nTri

	// Create Ogre mesh
	Ogre::MeshPtr meshOld = Ogre::MeshManager::getSingleton().getByName(sMesh);
	if (!meshOld.isNull()) { std::cout << "Mesh already exists: " + sMesh << std::endl; }

	Ogre::AxisAlignedBox aaBox;
	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(sMesh, "General");
	Ogre::SubMesh* sm = mesh->createSubMesh();

	createMesh(sm, aaBox, dlm.pos, dlm.norm, dlm.tcs, idx, rs.roadMtrStr);

	// Skip wall, columns, blend

	// Add mesh to scene
	Ogre::Entity* ent = 0;
	Ogre::SceneNode* node = 0;

	addMesh(mesh, sMesh, aaBox, &ent, &node, "." + sEnd);
	ent->setRenderQueueGroup(RQG_Road);

	//TODO CastShadows?

	// Only one LOD (for now)
	rs.road.node = node;
	rs.road.ent = ent;
	rs.road.mesh = mesh;
	rs.road.meshStr = sMesh;
	rs.empty = false;
}

void Road::createSegCollision(const DataLodMesh &dlm, const DataSeg &ds) {
	btTriangleMesh* trimesh = new btTriangleMesh; vbtTriMesh.push_back(trimesh);

#define vToBlt(v) btVector3(v.x, -v.z, v.y)
#define addTriB(a, b, c) trimesh->addTriangle(vToBlt(a), vToBlt(b), vToBlt(c));

	size_t si = posBt.size(), a = 0;
	for (size_t i = 0; i < si / 3; i++, a += 3) {
		addTriB(posBt[a], posBt[a+1], posBt[a+2]);
	}

	btCollisionShape* shape = new btBvhTriangleMeshShape(trimesh, true);

	size_t su = SU_Road + ds.idMtr;
	shape->setUserPointer((void*) su);
	shape->setMargin(0.01f); // Dev put a ? next to this line...

	btCollisionObject* bco = new btCollisionObject();
	btTransform tr; tr.setIdentity();

	bco->setActivationState(DISABLE_SIMULATION);
	bco->setCollisionShape(shape); bco->setWorldTransform(tr);
	bco->setFriction(0.8f); // Take note!
	bco->setRestitution(0.f);
	bco->setCollisionFlags(bco->getCollisionFlags() |
						   btCollisionObject::CF_STATIC_OBJECT | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);

	mSim->getCollisionWorld()->getDynamicsWorld()->addCollisionObject(bco);
	mSim->getCollisionWorld()->addShape(shape);

	// Skipping wall
}

void Road::destroyRoad() {
	for (int i = 0; i < vbtTriMesh.size(); i++) { delete vbtTriMesh[i]; }
	vbtTriMesh.clear();

	for (int seg = 0; seg < vSegs.size(); seg++) { destroySeg(seg); }
	vSegs.clear();

	idStr = 0;
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