#include <iostream>

#include <OgreException.h>

#include "App.hpp"

int main(int argc, char* argv[]) {
	App* a = new App();
	try {
		a->run();
	} catch (Ogre::Exception& e) {
		std::cerr << e.getFullDescription() << std::endl;
	}

	delete a;

	return 0;
}
