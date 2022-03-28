#include <iostream>
#include <random>
#include "scene.h"

int main() {
	Scene* scene = new Scene;
	scene->readScene("./cases/veach-mis/veach-mis");
	scene->getLightFacets();
	scene->MonteCarloPathTracer();
	scene->write2JPEG("./cases/veach-mis/veach-mis.jpg");

	return 0;
}
