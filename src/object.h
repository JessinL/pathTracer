#ifndef MONTE_CARLO_PATH_TRACER_OBJECT_H_
#define MONTE_CARLO_PATH_TRACER_OBJECT_H_

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <iostream>
#include <string>

class Object {
public:
	Object() {}
	~Object() {}
	inline int readObjFile(const char* filename);

public:
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
};

int Object::readObjFile(const char* filename) {
	std::string inputFileName = filename;
	tinyobj::ObjReaderConfig reader_config;

	tinyobj::ObjReader reader;

	if (!reader.ParseFromFile(inputFileName, reader_config)) {
		if (!reader.Error().empty()) {
			std::cerr << "TinyObjReader: " << reader.Error();
		}
		exit(1);
	}

	if (!reader.Warning().empty()) {
		std::cout << "TinyObjReader: " << reader.Warning();
	}

	attrib = reader.GetAttrib();
	shapes = reader.GetShapes();
	materials = reader.GetMaterials();

	return 0;
}

#endif