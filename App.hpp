#ifndef APP_HPP
#define APP_HPP

#include <OgreRoot.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>

class App {
public:
	App();
	~App();

	void run();

private:
	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
};

#endif
