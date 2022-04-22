#ifndef MONTE_CARLO_PATH_TRACER_SCENE_H_
#define MONTE_CARLO_PATH_TRACER_SCENE_H_

#include <random>
#include <math.h>
#include "light.h"
#include "camera.h"
#include "object.h"

#include <time.h>
#include <set>

#define USE_IGLAABB
#define GAMMA_CORRECTION

#ifdef USE_GRIDS
#include "myGrid.h"
#endif

#ifdef USE_IGLAABB
#include <igl/AABB.h>
#include <igl/Hit.h>
#include <eigen/Eigen/Dense>
#endif


#ifndef USE_GRID
#ifndef USE_IGLAABB
#include "bvhtree.h"
#endif
#endif

#define PI 3.1415926535
#define REAL double

#ifdef GAMMA_CORRECTION
#define GAMMA 2.2
#endif
//#define ENABLE_REFRACTION
#define ENABLE_INDIRECT_LIGHT

struct texture_{
	int materialid;
	int type;	// 0: diffuse 1:specular
	int width;
	int height;
	std::vector<double> data;
};

/**
 * @brief scene class, represent a scene consists of objects and lights, represented by triangle mesh
 * @note obj file i based on tinyobjloader, https://github.com/tinyobjloader/tinyobjloader
 * @note jpg file io bases on std_image, https://github.com/nothings/stb/blob/master/stb_image.h
 * @author Liu Zhixing, liuzhixing@zju.edu.cn
 */
class Scene {
public:
	Scene() {}
	~Scene() {
#ifdef USE_GRIDS
		if (mygrids != nullptr)
			delete mygrids;
		mygrids = nullptr;
#endif

#ifndef USE_GRIDS
#ifndef USE_IGLAABB
		if(bvht != nullptr)
			delete bvht;
		bvht = nullptr;
#endif
#endif
	}

	/**
	 * @brief read a scene from *.obj file
	 */
	void readScene(const char* scenepath);

	/**
	 * @brief load textures
	 * 
	 */
	void loadTextures(const char* scenepath);

	/**
	 * @brief the Monte Carlo Path Tracer
	 */
	void MonteCarloPathTracer(const int& sampletime, const char* logfilename);

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

#ifdef USE_GRIDS
	/**
	* @brief build grids
	*/
	void buildGrids();
#endif

#ifdef USE_IGLAABB
	/**
	* @brief build aabb-tree
 	* 
 	*/
	void buildAABBTree();
#endif
	/**
	* @brief get possibly intersection
	*/
	void getPIntersectionTriangles(
		const Vector& rayDirection, const Vector& rayStartingPoint,
		std::vector< std::vector<int> >& trianglesId
	);

#ifndef USE_GRIDS
#ifndef USE_IGLAABB
	/**
	 * @brief build a bvh-tree to accelerate self-intersection detection
	 * 
	 */
	void buildBVHTree();
#endif
#endif

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
		const Vector& p, const int& triangleId, const Vector& pNormal, const int& pMaterial,
		const Vector& lightCenter, const Vector& lightNormal,
		const Vector& ray,
		const double& lightr, const double& lightg, const double& lightb, const double& lightarea,
		double* r, double* g, double* b
	);

	void calIndirRadiance(
		const Vector& p, const int& triangleId, const Vector& pNormal, const int& pMaterial,
		const Vector& newp, const Vector& indirRayDirection,
		const double& indirRayr, const double& indirRayg, const double& indirRayb,
		const Vector& rayDirection,
		const double& prr,
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

#ifdef USE_GRIDS
	myGrids* mygrids{ nullptr };
#endif

#ifdef USE_IGLAABB
	igl::AABB<Eigen::MatrixXd, 3>* aabb{nullptr};
	Eigen::MatrixXd VERTICES;
	Eigen::MatrixXi FACETS;
#endif

#ifndef USE_GRIDS
#ifndef USE_IGLAABB
	bvhTree_* bvht{nullptr};
#endif
#endif

	std::vector< texture_ > textures;
};

#endif
