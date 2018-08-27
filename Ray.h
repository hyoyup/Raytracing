/*---------------------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.

File Name: Ray.h
Purpose: CS500 Raytracing
Language: C/C++
Platform: Windows: GNU (x86_64-posix-seh-rev1, Built by MinGW-W64 project) 7.1.0
		  Linux: GNU (Ubuntu 5.4.0-6ubuntu1~16.04.4) 5.4.0 20160609
Author: Hyoyup Chung
Creation date: February 11, 2018
-----------------------------------------------------------------------*/
// #include "geom.h"
// #include "Shape.h"
// #include <Eigen/StdVector> // For vectors, matrices (2d,3d,4d) and quaternions in f and d precision.
// #include <Eigen_unsupported/Eigen/BVH> // For KdBVH

// using namespace Eigen;

class Shape;

/* Ray class that represents Q+tD */
class Ray {
private:
	Vector3f m_PointQ;
	Vector3f m_Dir;
public:
	Ray(Vector3f q, Vector3f dir);
	Vector3f GetDirection() const;
	Vector3f GetPoint() const;
	Vector3f Eval(float t); // returns a point: Q+tD
};

/* Intersection class that represents intersection b/w 
   ray and shape */
class Intersection{
public:
	float t;
	Shape* m_pShape;
	Vector3f Point;
	Vector3f Normal;
	bool m_intersected;
	Intersection();
	Intersection(float, Shape*, Vector3f, Vector3f);
	void SetInf_t();
};

/* Interval along a ray */
class Interval{
public:
	float t0, t1;
	Vector3f Norm0, Norm1;

	Interval();
	Interval(float _t0, float _t1, Vector3f _N0, Vector3f _N1);
	Interval FullRay();
	bool IsValid();
	void SetRange(float, float);
	void SetData(std::pair<float,Vector3f>, std::pair<float,Vector3f>);
};

/* Slab for Intersect()*/
class Slab{
public:
	float d0, d1;
	Vector3f N;
	Slab(float _d0, float _d1, Vector3f _N);
	Interval IntersectSlab(Ray);
};