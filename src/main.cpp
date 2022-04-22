#include <iostream>
#include <random>

//#define PARALLEL
#ifdef PARALLEL
#include <omp.h>
#endif

#include "scene.h"

int main(int argc, char** argv) {
#ifdef PARALLEL
#pragma omp parallel for num_threads(6)
{
#endif

	Scene* scene = new Scene;
	if(argc < 3){
		std::cout<< "Obj file name needed." << std::endl;
		return 1;
	}
	char* commandline = argv[1];

	std::string scenename = commandline;

	std::string dirname = scenename;
	int pos = dirname.rfind('/');
	dirname = dirname.substr(0, pos);

	std::string outfilename = scenename;
	outfilename.append(".jpg");

	std::string logfilename = scenename;
	logfilename.append(".render.log");

	int sampletime = argv[2][0] - '0';

	scene->readScene(scenename.c_str());
	scene->loadTextures(dirname.c_str());
	scene->getLightFacets();
	scene->MonteCarloPathTracer(sampletime, logfilename.c_str());
	scene->write2JPEG(outfilename.c_str());

#ifdef PARALLEL
}
#endif
	return 0;
}
