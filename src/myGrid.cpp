#include "myGrid.h"

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
	minx = std::min(minx, point2.x), miny = std::min(miny, point2.y), minz = std::min(minz, point2.z);

	int xStartId = (int)((minx - (centerPoint.x - rangeXYZ.x)) / (gridLengthXYZ.x * 2));
	int xEndId = std::round((maxx - (centerPoint.x - rangeXYZ.x)) / (gridLengthXYZ.x * 2) + 0.5);
	int yStartId = (int)((miny - (centerPoint.y - rangeXYZ.y)) / (gridLengthXYZ.y * 2));
	int yEndId = std::round((maxy - (centerPoint.y - rangeXYZ.y)) / (gridLengthXYZ.y * 2) + 0.5);
	int zStartId = (int)((minz - (centerPoint.z - rangeXYZ.z)) / (gridLengthXYZ.z * 2));
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
