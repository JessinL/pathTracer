#ifndef MONTE_CARLO_PATH_TRACER_CAMERA_H_
#define MONTE_CARLO_PATH_TRACER_CAMERA_H_

#include <iostream>
#include <fstream>
#include "vector.h"
#include <math.h>

class Camera {
public:
	Camera() {}
	~Camera() {}
	int readXmlFile(const char* filename);

public:
	Vector eye;
	Vector lookat;
	Vector up;
	double fovy{ 0 };
	int width{ 0 };
	int height{ 0 };
};

#endif