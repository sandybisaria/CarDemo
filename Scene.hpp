#pragma once

#include <OgreSceneManager.h>

class Scene {
public:
	Scene(Ogre::SceneManager* sceneMgr);
	~Scene();

	void setupTerrain();

private:
	Ogre::SceneManager* mSceneMgr;
	Ogre::Light* sun;
};
