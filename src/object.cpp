#include "object.h"

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