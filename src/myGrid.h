#ifndef MONTE_CARLO_PATH_TRACER_MYGRID_H_
#define MONTE_CARLO_PATH_TRACER_MYGRID_H_

#include <iostream>
#include <algorithm>
#include "vector.h"
#include <vector>
#include <math.h>

class Grid {
public:
	Grid() {}
	~Grid() {}
	
	Grid(const double& x, const double& y, const double& z, const Vector& cp) {
		xLength = x;
		yLength = y;
		zLength = z;
		centerPoint.x = cp.x;
		centerPoint.y = cp.y;
		centerPoint.z = cp.z;
	}

	void insertAnElm(const int& elmId) {
		elmsId.push_back(elmId);
	}

public:
	double xLength{ 0 };
	double yLength{ 0 };
	double zLength{ 0 };
	Vector centerPoint{ Vector(0, 0, 0) };
	std::vector<int> elmsId;
};

/**
* @brief �����οռ仮��Ϊ���grid
* @author Liu Zhixing, liuzhixing@zju.edu.cn
*/
class myGrids {
public:
	myGrids() {}
	~myGrids() {}

	myGrids(const Vector& cp, const Vector& range, int nx, int ny, int nz);

	void insertATriangle(const Vector& point0, const Vector& point1, const Vector& point2, const int& triangleId);

	bool inGrids(const Vector& point) const;

public:
	Vector centerPoint{ Vector(0, 0, 0) };				/* ���οռ������ */
	Vector rangeXYZ{ Vector(0, 0, 0) };					/* ���οռ�ķ�Χ���߳�Ϊ 2*range */
	Vector gridLengthXYZ{ Vector(0, 0, 0) };			/* ÿ��grid�ı߳�Ϊ 2*gridLength */

	int numx{ 0 };
	int numy{ 0 };
	int numz{ 0 };

	std::vector<Grid> grids;
};

#endif