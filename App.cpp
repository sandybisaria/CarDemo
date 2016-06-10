#include <iostream>

#include "App.hpp"

App::App() {
	// No .cfg files, yet
	mRoot = OGRE_NEW Ogre::Root("", "");
}

App::~App() {
	delete mRoot;
}

void App::run() {
	std::cout << "Hello world!" << std::endl;
}
