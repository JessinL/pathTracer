#ifndef MONTE_CARLO_PATH_TRACER_SCENE_H_
#define MONTE_CARLO_PATH_TRACER_SCENE_H_

#include <random>
#include "light.h"
#include "camera.h"
#include "object.h"

#include <time.h>
#include <set>
#include "myGrid.h"

#define PI 3.1415926535
#define REAL double


#define DEBUG_LZX
#define ENABLE_SPECULAR
//#define ENABLE_REFRACTION
//#define ENABLE_INDIRECT_LIGHT

/**
 * @brief scene class, represent a scene with triangle mesh objects and lights
 * @note based on tinyobjloader, https://github.com/tinyobjloader/tinyobjloader
 * @note bases on std_image, https://github.com/nothings/stb/blob/master/stb_image.h
 * @author Liu Zhixing, liuzhixing@zju.edu.cn
 */
class Scene {
public:
	Scene() {}
	~Scene() {
		if (mygrids != nullptr)
			delete mygrids;
	}

	/**
	 * @brief read a scene from *.obj file
	 */
	void readScene(const char* scenepath);

	/**
	 * @brief the Monte Carlo Path Tracer
	 */
	void MonteCarloPathTracer();

	/**
	 * @brief tell if a ray intersects with an object made up by triangle meshes
	 * @note 1.if not only one intersection point, get the nearest point to the rayStartingPoint; 2.could be accelerated by k-d tree data structure
	 * 
	 * @param rayDirection 
	 * @param rayStartingPoint rayDirection and rayStartingPoint represent a ray
	 * @param affineIntersectionPoint the affine coordinate of the intersection point, nullptr if no intersection
	 * @param triangleId the id of the triangle owning the intersection point
	 */
	void rayObjectIntersecionDetective(
		const Vector& rayDirection, const Vector& rayStartingPoint,
		Vector* affineIntersectionPoint, int* triangleId,
		double farest = -1
	);

	/**
	 * @brief tell if a ray intersects with a triangle
	 * 
	 * @param rayDirection
	 * @param rayStartingPoint rayDirection and rayStartingPoint represent a ray
	 * @param pa
	 * @param pb
	 * @param pc pa, pb and pc are the three vertices of the triangle
	 * @param affineIntersectionPoint the affine coordinate of the intersection point on the ray, nullptr if no intersection
	 */
	void rayTriangleIntersectionDetective(
		const Vector& rayDirection, const Vector& rayStartingPoint,
		const Vector& pa, const Vector& pb, const Vector& pc,
		double* affineIntersectionPoint,
		bool* flag
	);
	/**
	 * @brief Monte Carlo shader
	 * 
	 * @param p 
	 * @param triangleId
	 * @param rayDirection
	 * @param rayStartingPoint
	 * @param wor
	 * @param wog
	 * @param wob wor, wog and wob are shading result
	 */
	void shade(
		const Vector& p, const int& triangleId, 
		const Vector& rayDirection, const Vector& rayStartingPoint,
		double* wor, double* wog, double* wob
	);

	/**
	 * @brief get all light triangles in the scene
	 */
	void getLightFacets();

	/**
	 * @brief write the rendering result to a *.jpg file
	 */
	void write2JPEG(const char* filename);

	/**
	* @brief build the relationship point2triangle
	*/
	void buildPoint2Triangles();

	/**
	* @brief build grids
	*/
	void buildGrids();

	/**
	* @brief get possibly intersection
	*/
	void getPIntersectionTriangles(
		const Vector& rayDirection, const Vector& rayStartingPoint,
		std::vector< std::vector<int> >& trianglesId
	);

	/**
	 * @brief to solve the rendering equation
	 * 
	 * @param p 
	 * @param pNormal 
	 * @param pMaterial 
	 * @param lightCenter 
	 * @param lightNormal 
	 * @param ray 
	 * @param lightr 
	 * @param lightg 
	 * @param lightb 
	 * @param lightarea 
	 * @param r the result radiance
	 * @param g the result radiance
	 * @param b the result radiance
	 */
	void calDirRadiance(
		const Vector& p, const Vector& pNormal, const int& pMaterial,
		const Vector& lightCenter, const Vector& lightNormal,
		const Vector& ray,
		const double& lightr, const double& lightg, const double& lightb, const double& lightarea,
		double* r, double* g, double* b
	);

protected:
	Camera camera;
	Light light;
	Object object;

	std::vector< std::pair<int, int> > lightFacets;

	std::vector< std::vector<int> > light2Facets;

	std::vector<float> pixelsr;
	std::vector<float> pixelsg;
	std::vector<float> pixelsb;

	std::vector< std::vector<int> > point2Triangles;

	myGrids* mygrids{ nullptr };
};

#endif
