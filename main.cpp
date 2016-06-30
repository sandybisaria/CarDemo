#include "App.hpp"

#include <OgreException.h>

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>
#endif

#include <iostream>

int main(int argc, char* argv[]) {

#ifdef COMPILE_UNIT_TESTS
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
#else

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
#endif
}
