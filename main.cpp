#include <iostream>

#include <OgreException.h>

#include "App.hpp"

int main(int argc, char* argv[]) {
	// Plugins are loaded manually, for now
	Ogre::Root* root = OGRE_NEW Ogre::Root("", "../config/resources.cfg");

	App* app = new App(root);
	try {
		app->run();
	} catch (Ogre::Exception& e) {
		std::cerr << e.getFullDescription() << std::endl;
	}

	delete app;
	OGRE_DELETE root;

	return 0;
}
