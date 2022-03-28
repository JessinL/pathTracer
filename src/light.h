#ifndef MONTE_CARLO_PATH_TRACER_LIGHT_H_
#define MONTE_CARLO_PATH_TRACER_LIGHT_H_

#include <iostream>
#include <string>
#include <fstream>
#include "vector.h"
#include "stringhandle.h"

struct OneLight {
	std::string name;
	Vector radiance;
};

class Light {
public:
	Light() {}
	~Light() {}
	int readXmlFile(const char* filename);
	
public:
	std::vector<OneLight> lights;
};

#endif
