#include <iostream>

#include "App.hpp"

int main(int argc, char* argv[]) {
	App* a = new App();
	a->run();

	delete a;

	return 0;
}
