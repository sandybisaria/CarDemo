#include "Road.hpp"

#include "../tinyxml/tinyxml2.h"

Road::Road() {
	//TODO: Init some variables
	setDefault();
}

void Road::setDefault() {
	//TODO: Reset some variables
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

	// Load road points
	n = root->FirstChildElement("P");
	while (n) {
		a = n->Attribute("pos"); newP.pos = Ogre::StringConverter::parseVector3(a);
		a = n->Attribute("w"); newP.width = Ogre::StringConverter::parseReal(a);
		a = n->Attribute("onTerr"); newP.onTerr = !(a && a[0] == '0'); // Road assumed to be on terrain unless otherwise noted

		insert(INS_End);

		n = n->NextSiblingElement("P");
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

	//TODO: Rebuild method?
}

//---- (Former) SplineEdit methods
Ogre::Real Road::getTerrH(Ogre::Vector3 p) {
	return mTerrain ? mTerrain->getHeightAtWorldPosition(p.x, 0.f, p.z) : 0.f;
}