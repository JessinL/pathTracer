#ifndef MONTE_CARLO_PATH_TRACER_OBJECT_H_
#define MONTE_CARLO_PATH_TRACER_OBJECT_H_

#include "tiny_obj_loader.h"
#include <iostream>
#include <string>

class Object {
public:
	Object() {}
	~Object() {}
	int readObjFile(const char* filename);

public:
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
};

#endif