#include "Scene.hpp"

Scene::Scene(Ogre::SceneManager* sceneMgr)
	: mSceneMgr(sceneMgr), sun(0) {

}

Scene::~Scene() {

}

void Scene::setupTerrain() {
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

	sun = mSceneMgr->createLight("SunLight");
	sun->setType(Ogre::Light::LT_DIRECTIONAL);
	sun->setDirection(Ogre::Vector3::NEGATIVE_UNIT_Y);
	sun->setDiffuseColour(Ogre::ColourValue::White);
	sun->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

	mSceneMgr->setSkyDome(true, "CloudySky");
}
