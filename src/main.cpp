#include <iostream>
#include <random>
#include "scene.h"

int main() {
	Scene* scene = new Scene;
	scene->readScene("../cases/cornell-box/cornell-box");
	scene->getLightFacets();
	scene->MonteCarloPathTracer();
	scene->write2JPEG("../cases/cornell-box/cornell-box.jpg");

	return 0;
}
