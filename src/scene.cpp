#include "scene.h"
#include "writejpg.h"

/**
 * @brief returns a posituve value if d lies inside the unique circle through a, b and c
 * 
 * @param pa
 * @param pb
 * @param pc
 * @param pd
 * @return 
 */
REAL incirclefast(REAL *pa, REAL *pb, REAL *pc, REAL *pd)
{
	REAL adx, ady, bdx, bdy, cdx, cdy;
	REAL abdet, bcdet, cadet;
	REAL alift, blift, clift;

	adx = pa[0] - pd[0];
	ady = pa[1] - pd[1];
	bdx = pb[0] - pd[0];
	bdy = pb[1] - pd[1];
	cdx = pc[0] - pd[0];
	cdy = pc[1] - pd[1];

	abdet = adx * bdy - bdx * ady;
	bcdet = bdx * cdy - cdx * bdy;
	cadet = cdx * ady - adx * cdy;
	alift = adx * adx + ady * ady;
	blift = bdx * bdx + bdy * bdy;
	clift = cdx * cdx + cdy * cdy;

	return alift * bcdet + blift * cadet + clift * abdet;
}

/**
 * @brief returns a positive value if a lies on the left of the direction line b->c
 */
REAL orient2dfast(REAL *pa, REAL *pb, REAL *pc)
{
	REAL acx, bcx, acy, bcy;

	acx = pa[0] - pc[0];
	bcx = pb[0] - pc[0];
	acy = pa[1] - pc[1];
	bcy = pb[1] - pc[1];
	return acx * bcy - acy * bcx;
}

/**
* @brief returns the area of a triangle
*/
REAL getArea(const Vector& pa, const Vector& pb, const Vector& pc) {
	double dab = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y)*(pa.y - pb.y) + (pa.z - pb.z)*(pa.z - pb.z));
	double dbc = std::sqrt((pc.x - pb.x) * (pc.x - pb.x) + (pc.y - pb.y)*(pc.y - pb.y) + (pc.z - pb.z)*(pc.z - pb.z));
	double dca = std::sqrt((pc.x - pa.x) * (pc.x - pa.x) + (pc.y - pa.y)*(pc.y - pa.y) + (pc.z - pa.z)*(pc.z - pa.z));
	double dpp = (dab + dbc + dca) / 2;
	double area = std::sqrt(dpp*(dpp - dab)*(dpp - dbc)*(dpp - dca));
	return area;
}

/**
 * @brief tell if a point lies inside a triangle
 *
 * @param return 1: inside, 0: on the boundary; -1: outside
 */
int pointInsideTriangle(const Vector& pa, const Vector& pb, const Vector& pc, const Vector& ss) {
	double areaabs = getArea(pa, pb, ss);
	double areabcs = getArea(pb, pc, ss);
	double areacas = getArea(pc, pa, ss);
	double areaabc = getArea(pa, pb, pc);
	if (std::abs(areaabc - areaabs - areabcs - areacas) < 1.0e-6)
		return 1;
	return 0;
}

/**
 * @brief tell if two vectors are parallel to each other
 */
bool parallel(Vector v1, Vector v2) {
	double length1 = v1.magnitude();
	double length2 = v2.magnitude();
	if (std::abs(std::abs(v1*v2) - length1 * length2) <= 1.0e-6)
		return true;
	return false;
}

/**
 * @brief project a point to a plane
 * @note  pa should be the origin point of the plane
 * 
 * @param pa
 * @param pb
 * @param pc pa, pb and pc are three points which construct the plane
 * @param pp pp is the point be projected
 * @param uu
 * @param vv uu and vv are the parameter coordinate of the point pp in this plane
 */
void projection(
	const Vector& pa, const Vector& pb, const Vector& pc,
	const Vector& pp,
	double* uu, double* vv
) {
	Vector op = pp - pa;
	Vector ou = pb - pa; ou.normalize();
	Vector temp = pc - pa;
	Vector nn = temp ^ ou; nn.normalize();
	Vector ov = nn ^ ou; ov.normalize();
	double u = 0, v = 0;
	if (parallel(op, ou)) {
		if (std::abs(ou.x) >= 1.0e-3)
			u = op.x / ou.x;
		else if (std::abs(ou.y) >= 1.0e-3)
			u = op.y / ou.y;
		else
			u = op.z / ou.z;
		v = 0;
	}
	else if (parallel(op, ov)) {
		u = 0;
		if (std::abs(ov.x) >= 1.0e-3)
			v = op.x / ov.x;
		else if (std::abs(ov.y) >= 1.0e-3)
			v = op.y / ov.y;
		else
			v = op.z / ov.z;
	}
	else {
		double a1 = ou.x, b1 = ov.x, a2 = ou.y, b2 = ov.y, p1 = op.x, p2 = op.y;
		double t1, t2, t3;
		std::vector<double> temp({ ou.z, ov.z, op.z });
	label:
		if (std::abs(b1) <= 1.0e-3) {
			if (std::abs(a1) <= 1.0e-3) {
				t1 = a1; t2 = b1; t3 = p1;
				a1 = temp[0]; b1 = temp[1]; p1 = temp[2];
				temp[0] = t1; temp[1] = t2; temp[2] = t3;
				goto label;
			}
			else {
				u = p1 / a1;
				if (std::abs(b2) <= 1.0e-3) {
					t1 = a2; t2 = b2; t3 = p2;
					a2 = temp[0]; b2 = temp[1]; p2 = temp[2];
					temp[0] = t1; temp[1] = t2; temp[2] = t3;
					goto label;
				}
				else {
					v = (p1 - a2 * u) / b2;
				}
			}
		}
		else if (std::abs(a1 - a2) <= 1.0e-3) {
			t1 = a1; t2 = b1; t3 = p1;
			a1 = temp[0]; b1 = temp[1]; p1 = temp[2];
			temp[0] = t1; temp[1] = t2; temp[2] = t3;
			goto label;
		}
		else {
			v = (p1*a2 - p2 * a1) / ((a2 - a1)*b1);
			if (std::abs(a1) <= 1.0e-3) {
				u = (p2 - b2 * v) / a2;
			}
			else
				u = (p1 - b2 * v) / a1;
		}
	}
	*uu = u;
	*vv = v;
}

/**
 * @brief generate random value
 * 
 */
std::random_device rd;
std::default_random_engine eng(rd());

/**
 * @brief generate a random double between 0 and 1
 */
std::uniform_real_distribution<> distr(0, 1);

/**
 * @brief generate a random double between -1 and 1
 */
std::uniform_real_distribution<> distrt(-1, 1);


void Scene::getLightFacets() {
	light2Facets.resize(light.lights.size());
	// for every shape
	for (int ii = 0; ii < object.shapes.size(); ii++) {
		// for every triangle
		for (int i = 0; i < object.shapes[ii].mesh.material_ids.size(); i++) {
			int triangleMtlId = object.shapes[0].mesh.material_ids[i];
			std::string triangleMtlName = object.materials[triangleMtlId].name;
			for (int j = 0; j < light.lights.size(); j++) {
				std::string lightMtlName = light.lights[j].name;
				if (lightMtlName == triangleMtlName) {
					lightFacets.push_back(std::pair<int, int>(i, j));
					light2Facets[j].push_back(i);
					break;
				}
			}
		}
	}
}

void Scene::readScene(const char* scenepath) {
	light.readXmlFile(scenepath);
	camera.readXmlFile(scenepath);
	std::string objfilename = scenepath;
	objfilename.append(".obj");
	object.readObjFile(objfilename.c_str());
	return;
}

void Scene::MonteCarloPathTracer() {
	// info of camera
	// the distance between camera and screen
	double distance = camera.height / (2.0 * std::tan(camera.fovy / 2.0 * PI / 180.0));
	// the direction of camera
	Vector direction = camera.lookat - camera.eye;
	direction.normalize();
	// the up direction of camera
	Vector up = camera.up;
	up.normalize();
	// the side direction of camera
	Vector side = up ^ direction;
	side.normalize();
	// center of screen
	Vector screenCenter = camera.eye + distance * direction;
	
	// store the r, g and b of every pixel
	pixelsr.resize(camera.width*camera.height);
	pixelsg.resize(camera.width*camera.height);
	pixelsb.resize(camera.width*camera.height);

	// build grids to accelerate intersection detection
	buildGrids();

	double start = clock();
	// for every pixel
	for (int ii = 0; ii < camera.height; ii++) {
		double lineStart = clock();
		for (int jj = 0; jj < camera.width; jj++) {
			// sample times
			int sampleTimes = 10;

			Vector pixel = screenCenter - (double)(ii - camera.height / 2) * up - (double)(jj - camera.width / 2)*side;

			double wor = 0, wog = 0, wob = 0;

#ifdef DEBUG_LZX
			int facetId = -1;
#endif

			// for every sample
			for (int time = 0; time < sampleTimes; time++) {
				// randomly generate a ray direction through this pixel
				double a = distr(eng);
				double b = distr(eng);
				Vector rayDirection = pixel - a * up - b * side - camera.eye;
				rayDirection.normalize();

				// 1. detect if there is an intersection
				Vector p;
				int triangleId = -1;
				rayObjectIntersecionDetective(rayDirection, camera.eye, &p, &triangleId);

				// if no intersection, set this pixel background color
				// TODO
				if (triangleId == -1) {
					wor += 0.0;
					wog += 0.0;
					wob += 0.0;
				}

				// 2. shading the intersection point
				else {
					double thiswor = 0, thiswog = 0, thiswob = 0;

					shade(p, triangleId, rayDirection, camera.eye, &thiswor, &thiswog, &thiswob);
					wor += thiswor;
					wog += thiswog;
					wob += thiswob;
				}
			}

			// get the average color of all the samples
			wor /= (double)sampleTimes;
			wog /= (double)sampleTimes;
			wob /= (double)sampleTimes;

			if (wor > 1.0)
				wor = 1.0;
			if (wog > 1.0)
				wog = 1.0;
			if (wob > 1.0)
				wob = 1.0;

			pixelsr[camera.width*ii + jj] = wor;
			pixelsg[camera.width*ii + jj] = wog;
			pixelsb[camera.width*ii + jj] = wob;
		}
		double lineEnd = clock();
		double lineTime = (lineEnd - lineStart) / (double)CLOCKS_PER_SEC;
#ifdef DEBUG_LZX
		std::cout << "Pixel line " << ii << " shading end. Shading time: " << lineTime << 's' << std::endl; 
#endif
	}
	double end = clock();
	double time = (end - start) / (double)CLOCKS_PER_SEC;
	std::cout << "Shading ends, time cost: " << time << 's' << std::endl;

	return;
}

void Scene::rayObjectIntersecionDetective(
	const Vector& rayDirection, const Vector& rayStartingPoint, 
	Vector* p, int* triangleId, double farest
) {
	// a pair < affine coordinate, triangleId > to represent a intersection point
	std::set< std::pair<double, int> > intersectionPoints;

	std::vector< std::vector<int> > pIntersectionTriangles;
	getPIntersectionTriangles(rayDirection, rayStartingPoint, pIntersectionTriangles);

	for (int ii = 0; ii < pIntersectionTriangles.size(); ii++) {
		int intersectionFlag = 0;
		for(int jj = 0; jj < pIntersectionTriangles[ii].size(); jj++){
			int& triangleId = pIntersectionTriangles[ii][jj];

			int index0 = object.shapes[0].mesh.indices[3 * triangleId].vertex_index;
			int index1 = object.shapes[0].mesh.indices[3 * triangleId + 1].vertex_index;
			int index2 = object.shapes[0].mesh.indices[3 * triangleId + 2].vertex_index;
			Vector point0(object.attrib.vertices[3 * index0], object.attrib.vertices[3 * index0 + 1], object.attrib.vertices[3 * index0 + 2]);
			Vector point1(object.attrib.vertices[3 * index1], object.attrib.vertices[3 * index1 + 1], object.attrib.vertices[3 * index1 + 2]);
			Vector point2(object.attrib.vertices[3 * index2], object.attrib.vertices[3 * index2 + 1], object.attrib.vertices[3 * index2 + 2]);

			double intersectionPoint = 0; bool flag = false;
			rayTriangleIntersectionDetective(rayDirection, rayStartingPoint, point0, point1, point2, &intersectionPoint, &flag);
			if (std::abs(intersectionPoint) <= 1.0e-3){
				continue;
			}
			if (flag && farest < 0) {
				intersectionPoints.insert(std::pair<double, int>(intersectionPoint, triangleId));
				if(!flag)
					intersectionFlag++;
			}
			else if(intersectionPoint < farest){
				intersectionPoints.insert(std::pair<double, int>(intersectionPoint, triangleId));
				if(!flag)
					intersectionFlag++;
			}
		}
		if(intersectionFlag)
			break;
	}

	// no acceleration
	// for every shape
	//for (int ii = 0; ii < object.shapes.size(); ii++) {
	//	// for every triangle
	//	for (int i = 0; i < object.shapes[ii].mesh.num_face_vertices.size(); i++) {
	//		// this triangle
	//  	int index0 = object.shapes[ii].mesh.indices[3 * i].vertex_index;
	//		int index1 = object.shapes[ii].mesh.indices[3 * i + 1].vertex_index;
	//		int index2 = object.shapes[ii].mesh.indices[3 * i + 2].vertex_index;
	//		Vector point0(object.attrib.vertices[3 * index0], object.attrib.vertices[3 * index0 + 1], object.attrib.vertices[3 * index0 + 2]);
	//		Vector point1(object.attrib.vertices[3 * index1], object.attrib.vertices[3 * index1 + 1], object.attrib.vertices[3 * index1 + 2]);
	//		Vector point2(object.attrib.vertices[3 * index2], object.attrib.vertices[3 * index2 + 1], object.attrib.vertices[3 * index2 + 2]);
			
			// get the intersection point
	//		double intersectionPoint = 0;
	//		rayTriangleIntersectionDetective(rayDirection, rayStartingPoint, point0, point1, point2, &intersectionPoint);
	//		if (intersectionPoint) {
	//			intersectionPoints.insert(std::pair<double, int>(intersectionPoint, i));
	//		}
	//	}
	//}

	// if no intersection, return
	if (intersectionPoints.empty()) {
		return;
	}
	// intersection exists, get the nearest intersection point
	auto iter = intersectionPoints.begin();
	*triangleId = (*iter).second;
	*p = (rayStartingPoint + (*iter).first * rayDirection);
}

void Scene::rayTriangleIntersectionDetective(
	const Vector& rayDirection, const Vector& rayStartingPoint,
	const Vector& pa, const Vector& pb, const Vector& pc,
	double* affineIntersectionPoint,
	bool* flag
) {
	// calulate the equation of this plane: A, B, C and D
	double AA = pa.y*(pb.z - pc.z) + pb.y*(pc.z - pa.z) + pc.y*(pa.z - pb.z);
	double BB = pa.z*(pb.x - pc.x) + pb.z*(pc.x - pa.x) + pc.z*(pa.x - pb.x);
	double CC = pa.x*(pb.y - pc.y) + pb.x*(pc.y - pa.y) + pc.x*(pa.y - pb.y);
	double DD = 0 - AA * pa.x - BB * pa.y - CC * pa.z;

	// the intersetion point the this ray and this plane, tt: affine coordinate on ray, pp: physics coordinate
	double tt = (0 - AA*rayStartingPoint.x - BB*rayStartingPoint.y - CC*rayStartingPoint.z - DD) / (AA*rayDirection.x + BB * rayDirection.y + CC * rayDirection.z);

	if (tt < 0) {
		affineIntersectionPoint = nullptr;
		*flag = false;
		return;
	}

	Vector pp = rayStartingPoint + tt * rayDirection;

	// tell if the intersection point is located at the internal of the triangle
	int inside = pointInsideTriangle(pa, pb, pc, pp);
	if (inside) {
		*affineIntersectionPoint = tt;
		*flag = true;
		return;
	}
	//else if (inside = 0) {
	//	*affineIntersectionPoint = tt;
	//	*flag = true;
	//	return;
	//}

	// get the parameter coordinate of four points, and then use the orient2d
	// I use orient2dfast here
#ifdef USE_ORIENT2D
	double a[2]; double b[2]; double c[2]; double p[2];
	projection(pa, pb, pc, pp, &p[0], &p[1]);
	projection(pa, pb, pc, pa, &a[0], &a[1]);
	projection(pa, pb, pc, pb, &b[0], &b[1]);
	projection(pa, pb, pc, pc, &c[0], &c[1]);

	double temp0 = orient2dfast(p, a, b);
	double temp1 = orient2dfast(p, b, c);
	double temp2 = orient2dfast(p, c, a);

	// if the intersection point is inside the triangle, get the affine coordinate
	if (std::abs(temp0) <= 1.0e-6) {
		if ((temp1 > 0 && temp2 < 0) || (temp1 < 0 && temp2 < 0)) {
			*affineIntersectionPoint = tt;
			*flag = true;
			//affineIntersectionPoint = nullptr;
			return;
		}
	}
	else if (std::abs(temp1) <= 1.0e-6) {
		if ((temp0 > 0 && temp2 < 0) || (temp0 < 0 && temp2 < 0)) {
			*affineIntersectionPoint = tt;
			*flag = true;
			//affineIntersectionPoint = nullptr;
			return;
		}
	}
	else if (std::abs(temp2) <= 1.0e-6) {
		if ((temp0 > 0 && temp1 < 0) || (temp0 < 0 && temp1 < 0)) {
			*affineIntersectionPoint = tt;
			*flag = true;
			//affineIntersectionPoint = nullptr;
			return;
		}
	}
	else if ((temp0 > 0 && temp1 > 0 && temp2 > 0) || (temp0 < 0 && temp1 < 0 && temp2 < 0)) {
		*affineIntersectionPoint = tt;
		*flag = true;
		return;
	}
#endif
	// if the intersection point is outside the triangle, set affineIntersectionPoint nullptr
	affineIntersectionPoint = nullptr;
	*flag = false;

	return;
}

void Scene::shade(
	const Vector& p, const int& triangleId, 
	const Vector& rayDirection, const Vector& rayStartingPoint,
	double* wor, double* wog, double* wob
) {
	// get the material information
	int triangleMtlId = object.shapes[0].mesh.material_ids[triangleId];
	std::string triangleMtlName = object.materials[triangleMtlId].name;

	// if the shading point is on the light, return white
	for (int ii = 0; ii < light.lights.size(); ii++) {
		std::string lightName = light.lights[ii].name;
		if (lightName == triangleMtlName) {
			*wor = 1.0;
			*wog = 1.0;
			*wob = 1.0;
			return;
		}
	}

	// shading 
	double resultr = 0, resultg = 0, resultb = 0;

	// the triangle owing the shading point, to get the normal direction of the point
	int triangleIndex0 = object.shapes[0].mesh.indices[3 * triangleId].vertex_index;
	int triangleIndex1 = object.shapes[0].mesh.indices[3 * triangleId + 1].vertex_index;
	int triangleIndex2 = object.shapes[0].mesh.indices[3 * triangleId + 2].vertex_index;
	Vector trianglePoint0(object.attrib.vertices[3 * triangleIndex0], object.attrib.vertices[3 * triangleIndex0 + 1], object.attrib.vertices[3 * triangleIndex0 + 2]);
	Vector trianglePoint1(object.attrib.vertices[3 * triangleIndex1], object.attrib.vertices[3 * triangleIndex1 + 1], object.attrib.vertices[3 * triangleIndex1 + 2]);
	Vector trianglePoint2(object.attrib.vertices[3 * triangleIndex2], object.attrib.vertices[3 * triangleIndex2 + 1], object.attrib.vertices[3 * triangleIndex2 + 2]);
	Vector trianglePoint0N(object.attrib.normals[3*triangleIndex0], object.attrib.normals[3*triangleIndex0+1], object.attrib.normals[3*triangleIndex0+2]);
	Vector trianglePoint1N(object.attrib.normals[3*triangleIndex1], object.attrib.normals[3*triangleIndex1+1], object.attrib.normals[3*triangleIndex1+2]);
	Vector trianglePoint2N(object.attrib.normals[3*triangleIndex2], object.attrib.normals[3*triangleIndex2+1], object.attrib.normals[3*triangleIndex2+2]);
	Vector triangleN = (trianglePoint0N + trianglePoint1N + trianglePoint2N)/3;
	triangleN.normalize();

	// 1. direct result
	double dirr = 0, dirg = 0, dirb = 0;

	// sampling for light
	// for every light
	for (int lightId = 0; lightId < light.lights.size(); lightId++) {
		double lightr = 0, lightg = 0, lightb = 0;
		if (light2Facets[lightId].size() <= 50) {
			//TODO
			// 若光源由少量三角面片组成，采样应采用发射多条光线的方法
			for(int i = 0; i < light2Facets[lightId].size(); i++){
				int pathSampleNum = 10;
				if(object.materials[triangleMtlId].specular[0] > 0.001)
					pathSampleNum = light2Facets[lightId].size() / 10;
				double facetr = 0, facetg = 0, facetb = 0;
				for(int sample = 0; sample < pathSampleNum; ){
					Vector pathStartingPoint = p;
					Vector pathDirection(distrt(eng), distrt(eng), distrt(eng));
					pathDirection.normalize();
					if(triangleN*pathDirection < 0){
						pathDirection.x = -pathDirection.x;
						pathDirection.y = -pathDirection.y;
						pathDirection.z = -pathDirection.z;
					}

					int facetId = light2Facets[lightId][i];
					int lightIndex0 = object.shapes[0].mesh.indices[3 * facetId].vertex_index;
					int lightIndex1 = object.shapes[0].mesh.indices[3 * facetId + 1].vertex_index;
					int lightIndex2 = object.shapes[0].mesh.indices[3 * facetId + 2].vertex_index;
					Vector lightPoint0(object.attrib.vertices[3 * lightIndex0], object.attrib.vertices[3 * lightIndex0 + 1], object.attrib.vertices[3 * lightIndex0 + 2]);
					Vector lightPoint1(object.attrib.vertices[3 * lightIndex1], object.attrib.vertices[3 * lightIndex1 + 1], object.attrib.vertices[3 * lightIndex1 + 2]);
					Vector lightPoint2(object.attrib.vertices[3 * lightIndex2], object.attrib.vertices[3 * lightIndex2 + 1], object.attrib.vertices[3 * lightIndex2 + 2]);
					Vector lightN = (lightPoint1 - lightPoint0) ^ (lightPoint2 - lightPoint0);
					lightN.normalize();

					double intersectionAffine = 0;
					bool flag = true;
					rayTriangleIntersectionDetective(pathDirection, pathStartingPoint, lightPoint0, lightPoint1, lightPoint2, &intersectionAffine, &flag);
					if(!flag)
						continue;

					sample++;

					Vector intersectionPoint = pathStartingPoint + intersectionAffine*pathDirection;
					Vector occlude; int occludeTriId = -1;
		    		double farest = (intersectionPoint.x - p.x) / pathDirection.x;
					rayObjectIntersecionDetective(pathDirection, pathStartingPoint, &occlude, &occludeTriId, farest);
					if (!(occludeTriId == -1)) {
						// if occlude, no shading
						continue;
					}

					// square of distance
					double distance2
						= (intersectionPoint.x - p.x) * (intersectionPoint.x - p.x)
						+ (intersectionPoint.y - p.y) * (intersectionPoint.y - p.y)
						+ (intersectionPoint.z - p.z) * (intersectionPoint.z - p.z);
					// cos(theta) and cos(theta')
					double costheta = pathDirection * triangleN; costheta = std::abs(costheta);
					double costhetat = (Vector(0, 0, 0) - pathDirection) * lightN; costhetat = std::abs(costhetat);

					// calculate the area of the light
					double area = 1.0e-6;

					// specular radiance
#ifdef ENABLE_SPECULAR
					// the half vector
					Vector halfVector = intersectionPoint - p; halfVector.normalize();
					halfVector = halfVector - rayDirection; halfVector.normalize();

					double cosalpha = std::max(0.0, triangleN * halfVector);

					double shi = object.materials[triangleMtlId].shininess;
					double specr = object.materials[triangleMtlId].specular[0] * light.lights[lightId].radiance.x * std::pow(cosalpha, shi) * costheta* costhetat / distance2 / (1 / area);
					double specg = object.materials[triangleMtlId].specular[1] * light.lights[lightId].radiance.y * std::pow(cosalpha, shi) * costheta* costhetat / distance2 / (1 / area);
					double specb = object.materials[triangleMtlId].specular[2] * light.lights[lightId].radiance.z * std::pow(cosalpha, shi) * costheta* costhetat / distance2 / (1 / area);
					facetr += specr;
					facetg += specg;
					facetb += specb;
#endif
					// diffuse radiance
					facetr += light.lights[lightId].radiance.x * (object.materials[triangleMtlId].diffuse[0] / PI) *costheta * costhetat / distance2 / (1 / area);
					facetg += light.lights[lightId].radiance.y * (object.materials[triangleMtlId].diffuse[1] / PI) *costheta * costhetat / distance2 / (1 / area);
					facetb += light.lights[lightId].radiance.z * (object.materials[triangleMtlId].diffuse[2] / PI) *costheta * costhetat / distance2 / (1 / area);
				}
				facetr /= (double)pathSampleNum;
				facetg /= (double)pathSampleNum;
				facetb /= (double)pathSampleNum;
			
				facetr = facetr * (double)light2Facets[lightId].size();
				facetg = facetg * (double)light2Facets[lightId].size();
				facetb = facetb * (double)light2Facets[lightId].size();	

				lightr += facetr;
				lightg += facetg;
				lightb += facetb;	
			}
		}
		else {
			// 若光源由多数三角面片组成，采样应采用对三角面片采样的方法
			int lightSampleNum = light2Facets[lightId].size() / 80;
			//if(object.materials[triangleMtlId].specular[0] > 0.001)
				//lightSampleNum = light2Facets[lightId].size() / 10;
			for (int sample = 0; sample < lightSampleNum; sample++) {
				// choose a facet randomly
				int facetId = std::rand() % light2Facets[lightId].size();
				facetId = light2Facets[lightId][facetId];

				int lightIndex0 = object.shapes[0].mesh.indices[3 * facetId].vertex_index;
				int lightIndex1 = object.shapes[0].mesh.indices[3 * facetId + 1].vertex_index;
				int lightIndex2 = object.shapes[0].mesh.indices[3 * facetId + 2].vertex_index;
				Vector lightPoint0(object.attrib.vertices[3 * lightIndex0], object.attrib.vertices[3 * lightIndex0 + 1], object.attrib.vertices[3 * lightIndex0 + 2]);
				Vector lightPoint1(object.attrib.vertices[3 * lightIndex1], object.attrib.vertices[3 * lightIndex1 + 1], object.attrib.vertices[3 * lightIndex1 + 2]);
				Vector lightPoint2(object.attrib.vertices[3 * lightIndex2], object.attrib.vertices[3 * lightIndex2 + 1], object.attrib.vertices[3 * lightIndex2 + 2]);
				Vector lightPoint0N(object.attrib.normals[3*lightIndex0], object.attrib.normals[3*lightIndex0+1], object.attrib.normals[3*lightIndex0+2]);
				Vector lightPoint1N(object.attrib.normals[3*lightIndex1], object.attrib.normals[3*lightIndex1+1], object.attrib.normals[3*lightIndex1+2]);
				Vector lightPoint2N(object.attrib.normals[3*lightIndex2], object.attrib.normals[3*lightIndex2+1], object.attrib.normals[3*lightIndex2+2]);
				Vector lightCenterPoint = (lightPoint0 + lightPoint1 + lightPoint2) / 3;
				Vector lightN = (lightPoint0N + lightPoint1N + lightPoint2N)/3;
				lightN.normalize();

				// make sure no occlude between the shading point and the light facet
				Vector path = lightCenterPoint - p; path.normalize();
				Vector occlude; int occludeTriId = -1;
		    	double farest = (lightCenterPoint.x - p.x) / path.x;
				rayObjectIntersecionDetective(path, p, &occlude, &occludeTriId, farest);
				if (!(occludeTriId == -1)) {
					// if occlude, no shading
					continue;
				}

				// calculate the area of the light
				double area = getArea(lightPoint0, lightPoint1, lightPoint2);
				double r = 0, g = 0, b = 0;

				calDirRadiance(
					p, triangleN, triangleMtlId, 
					lightCenterPoint, lightN, 
					rayDirection, 
					light.lights[lightId].radiance.x, light.lights[lightId].radiance.y, light.lights[lightId].radiance.z,
					area, &r, &g, &b
				);

				lightr += r;
				lightg += g;
				lightb += b;
			}
			lightr /= (double)lightSampleNum;
			lightg /= (double)lightSampleNum;
			lightb /= (double)lightSampleNum;
			
			lightr = lightr * (double)light2Facets[lightId].size();
			lightg = lightg * (double)light2Facets[lightId].size();
			lightb = lightb * (double)light2Facets[lightId].size();
		}
		dirr += lightr;
		dirg += lightg;
		dirb += lightb;
	}

	// effective the direct radiance
	resultr += dirr;
	resultg += dirg;
	resultb += dirb;

	// 2. indirect result
#ifdef ENABLE_INDIRECT_LIGHT
	double indirr = 0, indirg = 0, indirb = 0;

	// the probability of Russian Roulette
	double prr = 0.5;

	// randomly generate the ray or not
	double ksi = distr(eng);

	// no generation, no need to do anything
	// if generation exists
	if (ksi <= prr) {
		// we call the newly generation indirect ray
		// it's starting point is shading point
		// and then randomly get the ray direction
		Vector indirRayStartingPoint = p;
		Vector indirRayDirection(distrt(eng), distrt(eng), distrt(eng));
		indirRayDirection.normalize();
		if(indirRayDirection * triangleN < 0){
			indirRayDirection.x = -indirRayDirection.x;
			indirRayDirection.y = -indirRayDirection.y;
			indirRayDirection.z = -indirRayDirection.z;
		}

		// tell if there is new intersection 
		Vector* newp = new Vector; int newtriId = -1;
		rayObjectIntersecionDetective(indirRayDirection, indirRayStartingPoint, newp, &newtriId);

		// if no intersection, no need to do anything
		// if intersection exists
		if (newtriId != -1) {
			// tell if the newly intersection point is on a light
			int isLight = 0;
			for (int ii = 0; ii < lightFacets.size(); ii++) {
				if (lightFacets[ii].first == newtriId) {
					isLight++;
					break;
				}
			}

			// if it is on light, we have calculated it in the direct radiance module, no need to do anything
			// otherwise
			if (!isLight) {
				// get the indirect radiance
				double indirwor = 0, indirwog = 0, indirwob = 0;
				shade(*newp, newtriId, indirRayDirection, indirRayStartingPoint, &indirwor, &indirwog, &indirwob);

				// diffuse
				double costheta = std::abs(triangleN * indirRayDirection);
				indirr = indirwor * (object.materials[triangleMtlId].diffuse[0] / PI) * costheta / (0.5 / PI) / prr;
				indirg = indirwog * (object.materials[triangleMtlId].diffuse[1] / PI) * costheta / (0.5 / PI) / prr;
				indirb = indirwob * (object.materials[triangleMtlId].diffuse[2] / PI) * costheta / (0.5 / PI) / prr;

				// specular
#ifdef ENABLE_SPECULAR
				Vector halfVector = *newp - p; halfVector.normalize();
				halfVector = halfVector - rayDirection; halfVector.normalize();
				double cosalpha = std::max(0.0, triangleN * halfVector);
				double shi = object.materials[triangleMtlId].shininess;
				double specr = object.materials[triangleMtlId].specular[0] * indirwor * std::pow(cosalpha, shi) *costheta/ (0.5 / PI) / prr;
				double specg = object.materials[triangleMtlId].specular[1] * indirwog * std::pow(cosalpha, shi) *costheta/ (0.5 / PI) / prr;
				double specb = object.materials[triangleMtlId].specular[2] * indirwob * std::pow(cosalpha, shi) *costheta/ (0.5 / PI) / prr;
				indirr += specr;
				indirg += specg;
				indirg += specb;
#endif
			}

		}
	}

	resultr += indirr;
	resultg += indirg;
	resultb += indirb;
#endif

	*wor = resultr;
	*wog = resultg;
	*wob = resultb;
}

void Scene::write2JPEG(const char* filename) {
	std::cout << "Writing to jpeg..." << std::endl;
	unsigned char* colorData = new unsigned char[camera.height*camera.width * 3];
	for (int ii = 0; ii < camera.height; ii++) {
		for (int jj = 0; jj < camera.width; jj++) {					
			colorData[3 * ii*camera.width + 3 * jj] = std::round(255.0 * pixelsr[camera.width*ii + jj]);
			colorData[3 * ii*camera.width + 3 * jj + 1] = std::round(255.0 * pixelsg[camera.width*ii + jj]);
			colorData[3 * ii*camera.width + 3 * jj + 2] = std::round(255.0 * pixelsb[camera.width*ii + jj]);
		}
	}
	writejpg(filename, camera.width, camera.height, 3, colorData, camera.width*camera.height);
	std::cout << "Writing success." << std::endl;
	delete colorData;
	colorData = nullptr;
}

void Scene::buildPoint2Triangles() {
	point2Triangles.resize(object.attrib.vertices.size() / 3);
	for (int triId = 0; triId < object.shapes[0].mesh.num_face_vertices.size(); triId++) {
		int vtx0Id = object.shapes[0].mesh.indices[3 * triId].vertex_index;
		int vtx1Id = object.shapes[0].mesh.indices[3 * triId + 1].vertex_index;
		int vtx2Id = object.shapes[0].mesh.indices[3 * triId + 2].vertex_index;
		point2Triangles[vtx0Id].push_back(triId);
		point2Triangles[vtx1Id].push_back(triId);
		point2Triangles[vtx2Id].push_back(triId);
	}
}

void Scene::buildGrids() {
	double maxx = -1.0e-3, maxy = -1.0e-3, maxz = -1.0e-3;
	double minx = 1.0e3, miny = 1.0e3, minz = 1.0e3;
	for (int pointId = 0; pointId < object.attrib.vertices.size() / 3; pointId++) {
		if (maxx < object.attrib.vertices[3 * pointId])
			maxx = object.attrib.vertices[3 * pointId];
		if (minx > object.attrib.vertices[3 * pointId])
			minx = object.attrib.vertices[3 * pointId];
		if (maxy < object.attrib.vertices[3 * pointId + 1])
			maxy = object.attrib.vertices[3 * pointId + 1];
		if (miny > object.attrib.vertices[3 * pointId + 1])
			miny = object.attrib.vertices[3 * pointId + 1];
		if (maxz < object.attrib.vertices[3 * pointId + 2])
			maxz = object.attrib.vertices[3 * pointId + 2];
		if (minz > object.attrib.vertices[3 * pointId + 2])
			minz = object.attrib.vertices[3 * pointId + 2];
	}
	if (maxx < camera.eye.x)
		maxx = camera.eye.x;
	if (minx > camera.eye.x)
		minx = camera.eye.x;
	if (maxy < camera.eye.y)
		maxy = camera.eye.y;
	if (miny > camera.eye.y)
		miny = camera.eye.y;
	if (maxz < camera.eye.z)
		maxz = camera.eye.z;
	if (minz > camera.eye.z)
		minz = camera.eye.z;
	maxx += 5.0; maxy += 5.0; maxz += 5.0;
	minx -= 5.0; miny -= 5.0; minz -= 5.0;
	Vector mygridscp((minx + maxx) / 2, (miny + maxy) / 2, (minz + maxz) / 2);
	Vector mygridsrange((maxx - minx) / 2, (maxy - miny) / 2, (maxz - minz) / 2);
	mygrids = new myGrids(mygridscp, mygridsrange, 100, 100, 100);

	for (int i = 0; i < object.shapes[0].mesh.num_face_vertices.size(); i++) {
		// this triangle
		int index0 = object.shapes[0].mesh.indices[3 * i].vertex_index;
		int index1 = object.shapes[0].mesh.indices[3 * i + 1].vertex_index;
		int index2 = object.shapes[0].mesh.indices[3 * i + 2].vertex_index;
		Vector point0(object.attrib.vertices[3 * index0], object.attrib.vertices[3 * index0 + 1], object.attrib.vertices[3 * index0 + 2]);
		Vector point1(object.attrib.vertices[3 * index1], object.attrib.vertices[3 * index1 + 1], object.attrib.vertices[3 * index1 + 2]);
		Vector point2(object.attrib.vertices[3 * index2], object.attrib.vertices[3 * index2 + 1], object.attrib.vertices[3 * index2 + 2]);
		mygrids->insertATriangle(point0, point1, point2, i);
	}
}

void Scene::getPIntersectionTriangles(
	const Vector& rayDirection, const Vector& rayStartingPoint,
	std::vector< std::vector<int> >& trianglesId
) {
	trianglesId.clear();

	std::vector<int> gridsId;
	Vector start = rayStartingPoint, end;
	do {
		end = start + rayDirection * 1;
		double leftx = std::min(start.x, end.x), rightx = std::max(start.x, end.x);
		double lefty = std::min(start.y, end.y), righty = std::max(start.y, end.y);
		double leftz = std::min(start.z, end.z), rightz = std::max(start.z, end.z);

		int xStartId = std::round((leftx - (mygrids->centerPoint.x - mygrids->rangeXYZ.x)) / (mygrids->gridLengthXYZ.x * 2) + 0.5);
		int xEndId = std::round((rightx - (mygrids->centerPoint.x - mygrids->rangeXYZ.x)) / (mygrids->gridLengthXYZ.x * 2) + 0.5);
		int yStartId = std::round((lefty - (mygrids->centerPoint.y - mygrids->rangeXYZ.y)) / (mygrids->gridLengthXYZ.y * 2) + 0.5);
		int yEndId = std::round((righty - (mygrids->centerPoint.y - mygrids->rangeXYZ.y)) / (mygrids->gridLengthXYZ.y * 2) + 0.5);
		int zStartId = std::round((leftz - (mygrids->centerPoint.z - mygrids->rangeXYZ.z)) / (mygrids->gridLengthXYZ.z * 2) + 0.5);
		int zEndId = std::round((rightz - (mygrids->centerPoint.z - mygrids->rangeXYZ.z)) / (mygrids->gridLengthXYZ.z * 2) + 0.5);

		if (xStartId < 0)
			xStartId = 0;
		if (yStartId < 0)
			yStartId = 0;
		if (zStartId < 0)
			zStartId = 0;
		if (xEndId > mygrids->numx - 1)
			xEndId = mygrids->numx - 1;
		if (yEndId > mygrids->numy - 1)
			yEndId = mygrids->numy - 1;
		if (zEndId > mygrids->numz - 1)
			zEndId = mygrids->numz - 1;

		for (int i = xStartId; i < xEndId + 1; i++) {
			for (int j = yStartId; j < yEndId + 1; j++) {
				for (int k = zStartId; k < zEndId + 1; k++) {
					gridsId.push_back(i* (mygrids->numy) * (mygrids->numz) + j * (mygrids->numz) + k);
				}
			}
		}

		start = end;
	} while (mygrids->inGrids(start) || mygrids->inGrids(end));

	trianglesId.clear();
	for (int i = 0; i < gridsId.size(); i++) {
		std::vector<int>& tris = mygrids->grids[gridsId[i]].elmsId;
		if(tris.empty())
			continue;
		trianglesId.push_back(tris);
	}
}

void Scene::calDirRadiance(
	const Vector& p, const Vector& pNormal, const int& pMaterial,
	const Vector& lightCenter, const Vector& lightNormal,
	const Vector& ray,
	const double& lightr, const double& lightg, const double& lightb, const double& lightarea,
	double* r, double* g, double* b
){
	*r = 0; *g = 0; *b = 0;
	//get square of distance
	double distance2 = (p.x - lightCenter.x)*(p.x - lightCenter.x) + (p.y - lightCenter.y)*(p.y - lightCenter.y) + (p.z - lightCenter.z)*(p.z - lightCenter.z);

	//get cos(theta), cos(theta')
	Vector path = lightCenter - p; path.normalize();
	double costheta = std::max(0.0, path*pNormal);
	double costheta_ = std::max(0.0, (Vector(0, 0, 0) - path) * lightNormal);

	//get area of light

	// diffuse radiance
	*r += lightr * object.materials[pMaterial].diffuse[0] / PI * costheta * costheta_ * lightarea / distance2;
	*g += lightg * object.materials[pMaterial].diffuse[1] / PI * costheta * costheta_ * lightarea / distance2;
	*b += lightb * object.materials[pMaterial].diffuse[2] / PI * costheta * costheta_ * lightarea / distance2;

	//half vector
	Vector halfVector = path - ray;
	halfVector.normalize();
	double cosalpha = halfVector * pNormal;
	cosalpha = std::max(0.0, 2*cosalpha*cosalpha - 1);

	int shi = object.materials[pMaterial].shininess;

	// specular radiance
	*r += lightr * object.materials[pMaterial].specular[0] * std::pow(cosalpha, shi) * costheta * costheta_ * lightarea / distance2*30;
	*g += lightg * object.materials[pMaterial].specular[1] * std::pow(cosalpha, shi) * costheta * costheta_ * lightarea / distance2*30;
	*b += lightg * object.materials[pMaterial].specular[2] * std::pow(cosalpha, shi) * costheta * costheta_ * lightarea / distance2*30;
}