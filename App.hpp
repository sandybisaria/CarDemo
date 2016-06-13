#ifndef APP_HPP
#define APP_HPP

#include <OgreRoot.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>

class App : public Ogre::FrameListener {
public:
	App();
	~App();

	void run();

private:
	bool setupPlugins();
	void setupResources();
	bool setupRenderSystem();

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
};

#endif
