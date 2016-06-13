#ifndef APP_HPP
#define APP_HPP

#include <OgreRoot.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreWindowEventUtilities.h>

class App : public Ogre::FrameListener,
			public Ogre::WindowEventListener {
public:
	App();
	~App();

	void run();

private:
	bool setupPlugins();
	void setupResources();
	bool setupRenderSystem();

	void loop();

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

	virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);

	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
};

#endif
