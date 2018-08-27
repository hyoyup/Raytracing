#include "geom.h"
#include "Ray.h"
#include <limits>
#include <cmath>
#include <iostream>

////////////////////////////
// IMPLEMENTATION FOR RAY //
////////////////////////////
Ray::Ray(Vector3f q, Vector3f dir):	m_PointQ(q), m_Dir(dir){
}

Vector3f Ray::GetDirection() const{
	return m_Dir;
}
Vector3f Ray::GetPoint() const{
	return m_PointQ;
}
Vector3f Ray::Eval(float t){
	//printf("eval() called for t: %f\n", t);
	return m_PointQ + t*m_Dir;
}

/////////////////////////////////////
// IMPLEMENTATION FOR INTERSECTION //
/////////////////////////////////////
Intersection::Intersection(){
	t = -1.0f;//std::numeric_limits<float>::max();
	m_pShape=nullptr;
	Point = Vector3f(0.0f,0.0f,0.0f);
	Normal = Vector3f(0.0f,0.0f,0.0f);
	m_intersected = false;
}
Intersection::Intersection(float _t, Shape* _pShape, Vector3f _p, Vector3f _n):
	t(_t), m_pShape(_pShape), Point(_p), Normal(_n) {
	m_intersected = false;
}

void Intersection::SetInf_t(){
	t = std::numeric_limits<float>::max();
}

/////////////////////////////////
// IMPLEMENTATION FOR INTERVAL //
/////////////////////////////////

Interval::Interval(){
	t0 = 1;
	t1 = 0;
}

Interval::Interval(float _t0, float _t1, Vector3f _N0, Vector3f _N1):
	t0(_t0), t1(_t1), Norm0(_N0), Norm1(_N1) {
	if(_t0 > _t1) {
		t0 = _t1;
		Norm0 = _N1;
		t1 = _t0;
		Norm1 = _N0;
	}
}

Interval Interval::FullRay(){
	return Interval(0.0f, std::numeric_limits<float>::max(),Vector3f(),Vector3f());
}

bool Interval::IsValid(){
	if(t0 > t1){
		return false;
	}
	return true;
}

void Interval::SetRange(float _t0, float _t1){
	t0 = _t0;
	t1 = _t1;
}

void Interval::SetData(std::pair<float,Vector3f> p0, std::pair<float,Vector3f> p1){
	t0 = p0.first;
	Norm0 = p0.second;
	t1 = p1.first;
	Norm1 = p1.second;
}

#define EPSILON 0.00001f

/////////////////////////////
// IMPLEMENTATION FOR SLAB //
/////////////////////////////

Slab::Slab(float _d0, float _d1, Vector3f _N)
: d0(_d0), d1(_d1), N(_N){
	// if (d0 < d1){
	// 	//std::cout << "slab value swapped\n";
	// 	d0 = _d1;
	// 	d1 = _d0;
	// }
}

Interval Slab::IntersectSlab(Ray ray){
	float NDotD = N.dot(ray.GetDirection());
	float NDotQ = N.dot(ray.GetPoint());
	if (!(fabs(NDotD)<EPSILON)){ // ray intersects both planes
		float t0 = -(d0 + NDotQ) / NDotD;
		float t1 = -(d1 + NDotQ) / NDotD;
		if (t1 < t0){ // reorder interval
			return Interval(t1,t0,-N,N);
		}
		return Interval(t0,t1,-N,N);
	}else { // ray is parallel to planes
		float s0 = NDotQ + d0;
		float s1 = NDotQ + d1;
		if (s0 < EPSILON){	//s0 neg
			if (s1 < EPSILON) {		// both neg: ray outside of planes
				return Interval();
			} 
			else {					// sign differ: ray b/w planes
				return Interval(0.0f,std::numeric_limits<float>::max(),N,N);// [0,inf]
			}
		}
		else { 			  	//s0 pos
			if (s1 < EPSILON){		// sign differ: ray b/w planes
				return Interval(0.0f,std::numeric_limits<float>::max(),N,N);// [0,inf]
			}
			else {					// both pos: ray outside of planes
				return Interval();
			}
		}
	}
	return Interval();
}