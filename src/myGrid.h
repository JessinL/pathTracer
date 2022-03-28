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

	inline myGrids(const Vector& cp, const Vector& range, int nx, int ny, int nz);

	inline void insertATriangle(const Vector& point0, const Vector& point1, const Vector& point2, const int& triangleId);

	inline bool inGrids(const Vector& point) const;

public:
	Vector centerPoint{ Vector(0, 0, 0) };				/* ���οռ������ */
	Vector rangeXYZ{ Vector(0, 0, 0) };					/* ���οռ�ķ�Χ���߳�Ϊ 2*range */
	Vector gridLengthXYZ{ Vector(0, 0, 0) };			/* ÿ��grid�ı߳�Ϊ 2*gridLength */

	int numx{ 0 };
	int numy{ 0 };
	int numz{ 0 };

	std::vector<Grid> grids;
};

myGrids::myGrids(const Vector& cp, const Vector& range, int nx, int ny, int nz) {
	// get the range in space
	double rangex = range.x * 2, rangey = range.y * 2, rangez = range.z * 2;

	// range of every grid
	double xl = rangex / nx / 2, yl = rangey / ny / 2, zl = rangez / nz / 2;

	// for every grid
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int k = 0; k < nz; k++) {
				// get the center point of this grid
				Vector centerPoint(cp.x + (i - nx / 2)* xl * 2 + xl, cp.y + (j - ny / 2)*yl * 2 + yl, cp.z + (k - nz / 2)*zl * 2 + zl);
				grids.push_back(Grid(xl, yl, zl, centerPoint));
			}
		}
	}

	numx = nx;
	numy = ny;
	numz = nz;

	gridLengthXYZ = Vector(xl, yl, zl);
	centerPoint = cp;
	rangeXYZ = range;
}

void myGrids::insertATriangle(const Vector& point0, const Vector& point1, const Vector& point2, const int& triangleId) {
	// AABB box
	double maxx = std::max(point0.x, point1.x), maxy = std::max(point0.y, point1.y), maxz = std::max(point0.z, point1.z);
	double minx = std::min(point0.x, point1.x), miny = std::min(point0.y, point1.y), minz = std::min(point0.z, point1.z);
	maxx = std::max(maxx, point2.x), maxy = std::max(maxy, point2.y), maxz = std::max(maxz, point2.z);
	minx = std::min(maxx, point2.x), miny = std::min(maxy, point2.y), minz = std::min(maxz, point2.z);

	int xStartId = std::round((minx - (centerPoint.x - rangeXYZ.x)) / (gridLengthXYZ.x * 2) + 0.5);
	int xEndId = std::round((maxx - (centerPoint.x - rangeXYZ.x)) / (gridLengthXYZ.x * 2) + 0.5);
	int yStartId = std::round((miny - (centerPoint.y - rangeXYZ.y)) / (gridLengthXYZ.y * 2) + 0.5);
	int yEndId = std::round((maxy - (centerPoint.y - rangeXYZ.y)) / (gridLengthXYZ.y * 2) + 0.5);
	int zStartId = std::round((minz - (centerPoint.z - rangeXYZ.z)) / (gridLengthXYZ.z * 2) + 0.5);
	int zEndId = std::round((maxz - (centerPoint.z - rangeXYZ.z)) / (gridLengthXYZ.z * 2) + 0.5);

	for (int i = xStartId; i < xEndId + 1; i++) {
		for (int j = yStartId; j < yEndId + 1; j++) {
			for (int k = zStartId; k < zEndId + 1; k++) {
				grids[i* numy *numz + j * numz + k].insertAnElm(triangleId);
			}
		}
	}
}

bool myGrids::inGrids(const Vector& point) const {
	if (
		point.x < centerPoint.x + rangeXYZ.x && point.x > centerPoint.x - rangeXYZ.x &&
		point.y < centerPoint.y + rangeXYZ.y && point.y > centerPoint.y - rangeXYZ.y &&
		point.z < centerPoint.z + rangeXYZ.z && point.z > centerPoint.z - rangeXYZ.z
		) {
		return true;
	}
	return false;
}

#endif