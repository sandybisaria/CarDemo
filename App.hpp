#ifndef APP_HPP
#define APP_HPP

#include <OgreRoot.h>

class App {
public:
	App();
	~App();

	void run();

private:
	Ogre::Root* mRoot;
};

#endif
