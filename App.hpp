#ifndef APP_HPP
#define APP_HPP

#include <OgreRoot.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>

#include <OISInputManager.h>
#include <OISKeyboard.h>

class App
	: public Ogre::FrameListener,
	  public Ogre::WindowEventListener,
	  public OIS::KeyListener {
public:
	App(Ogre::Root* root);
	~App();

	void run();

private:
	bool setup();
	void setupResources();
	bool setupRenderSystem();
	void setupInputSystem();
	void setupViewSystem();
	void setupListeners();
	void setupScene();

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

	virtual void windowClosed(Ogre::RenderWindow* rw);

	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);

	bool mShutDown;

	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;

	OIS::InputManager* mInputMgr;
	OIS::Keyboard* mKeyboard;
};

#endif
