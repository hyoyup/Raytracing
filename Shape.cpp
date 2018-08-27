#include "geom.h"
#include "Shape.h"
#include "Ray.h"
#include "Scene.h"

#include <cmath> // sqrt, fmin, fmax
#include <iostream>
#include <utility>

using namespace std;

#define EPSILON 0.00001f

#include <random>
std::mt19937_64 RNGen2;
std::uniform_real_distribution<> myrandom2(0.0, 1.0);

/*****************************
	IMPLEMENTATION FOR SPHERE
******************************/
Sphere::Sphere():m_Type(SPHERE) {
}

Sphere::Sphere(Vector3f center, float r, Material* mat)
: m_Type(SPHERE), m_Center(center), m_radius(r), m_Material(mat) {
	m_Bbox = Box3d(m_Center-Vector3f(r,r,r), m_Center+Vector3f(r,r,r));
}

Sphere::~Sphere(){
	// if (m_Material){
	// 	delete m_Material;
	// }
}

bool Sphere::Intersect(Ray ray, Intersection& inter){
	// find simplified discriminant 
	// NOTE: ray.Direction.dot(ray.Direction) = 1
	Vector3f centerToQ = (ray.GetPoint() - m_Center);
	float qD = centerToQ.dot(ray.GetDirection());
	float qq = centerToQ.dot(centerToQ);
	float disc = qD*qD - qq + m_radius*m_radius;

	// if discriminant is negative or 0, no intersection
	if (disc<EPSILON){
		//std::cout<<"false";
		return false;
	}
	// compute two t-values
	float t0 = (-1.0f)*qD - sqrt(disc);
	float t1 = (-1.0f)*qD + sqrt(disc);
	//std::cout << t0 << "   " << t1 <<std::endl;
	if (t0 < EPSILON){ 	// t0 neg
		if (t1 < EPSILON){	// both neg
			return false;
		}else { 			// only t1 valid
			inter.t = t1;
			inter.Point = ray.Eval(t1);
			inter.Normal = (inter.Point - m_Center).normalized();
			inter.m_intersected = true;
			return true;
		}
	}else{ 				// t0 pos
		if (t1 < EPSILON) {	// only t0 valid
			inter.t = t0;
			inter.Point = ray.Eval(t0);
			inter.Normal = (inter.Point - m_Center).normalized();
			inter.m_intersected = true;
			return true;
		}else{ 				// both valid
			inter.t = fmin(t0,t1);
			inter.Point = ray.Eval(inter.t);
			inter.Normal = (inter.Point - m_Center).normalized();
			inter.m_intersected = true;
			return true;
		}
	}
}

Box3d Sphere::GetBBox(){
	return m_Bbox;
}

ShapeType Sphere::GetType(){
	return m_Type;
}

Material* Sphere::GetMaterial(){
	return m_Material;
}

bool Sphere::IsLight(){
	return m_Material->IsLight();
}

Vector3f Sphere::Radiance(){
	return m_Material->Radiance();
}


void Sphere::SamplePoint(Intersection& inter){
	float E1 = myrandom2(RNGen2);
	float E2 = myrandom2(RNGen2);

	float z = 2.0f * E1 - 1.0f;
	float r = sqrt(1 - z*z);
	float a = 2.0f * PI * E2;

	inter.Normal = Vector3f(r * cosf(a), r * sinf(a), z ).normalized();
	inter.Point = m_Center + inter.Normal * m_radius;
	inter.m_pShape = this;
}

float Sphere::Area(){
	return 4.0f * PI;
}

/***************************
	IMPLEMENTATION FOR BOX
****************************/
Box::Box(): m_Type(BOX){
}

Box::Box(Vector3f bottomleft, Vector3f diag)
: m_Type(BOX), m_Corner(bottomleft), m_DiagVec(diag){
}

Box::Box(Vector3f bottomleft, Vector3f diag, Material* mat)
: m_Type(BOX), m_Corner(bottomleft), m_DiagVec(diag), m_Material(mat) {
	m_Bbox = Box3d(bottomleft, bottomleft+m_DiagVec);
}


Box::~Box(){
	// if (m_Material){
	// 	delete m_Material;
	// }
}

inline pair<float,Vector3f> PairMax(pair<float,Vector3f> p0, pair<float,Vector3f> p1){
	if (p0.first < p1.first){
		return p1;
	}
	return p0;
}
inline pair<float,Vector3f> PairMin(pair<float,Vector3f> p0, pair<float,Vector3f> p1){
	if (p0.first < p1.first){
		return p0;
	}
	return p1;
}

bool Box::Intersect(Ray ray, Intersection& inter){
	// starting interval
	Interval interval = Interval().FullRay();
	// 3 slabs
	Slab s0 = Slab(-m_Corner[0], -m_Corner[0] - m_DiagVec[0], Vector3f(1,0,0));
	Slab s1 = Slab(-m_Corner[1], -m_Corner[1] - m_DiagVec[1], Vector3f(0,1,0));
	Slab s2 = Slab(-m_Corner[2], -m_Corner[2] - m_DiagVec[2], Vector3f(0,0,1));
	// get interval from 3 slabs
	Interval temp = s0.IntersectSlab(ray);
	if (!temp.IsValid()){
		return false;
	}
	interval.SetData(PairMax(make_pair(interval.t0,interval.Norm0), make_pair(temp.t0,temp.Norm0)),
					 PairMin(make_pair(interval.t1,interval.Norm1), make_pair(temp.t1,temp.Norm1)) );

	temp = s1.IntersectSlab(ray);
	if (!temp.IsValid()){
		return false;
	}
	interval.SetData(PairMax(make_pair(interval.t0,interval.Norm0), make_pair(temp.t0,temp.Norm0)),
					 PairMin(make_pair(interval.t1,interval.Norm1), make_pair(temp.t1,temp.Norm1)) );
	temp = s2.IntersectSlab(ray);
	if (!temp.IsValid()){
		return false;
	}
	interval.SetData(PairMax(make_pair(interval.t0,interval.Norm0), make_pair(temp.t0,temp.Norm0)),
					 PairMin(make_pair(interval.t1,interval.Norm1), make_pair(temp.t1,temp.Norm1)) );
	// evaluate returned interval
	if (!interval.IsValid()){
		return false;
	}

	float t0 = interval.t0;
	float t1 = interval.t1;
	//Vector3f center = GetCenter();
	if (t0 < EPSILON){ 	// t0 neg
		if (t1 < EPSILON){	// both neg
			return false;
		}else { 			// only t1 valid
			inter.t = t1;
			inter.Point = ray.Eval(t1);
			inter.Normal = interval.Norm1;
			inter.m_intersected = true;
			return true;
		}
	}else{ 				// t0 pos
		if (t1 < EPSILON) {	// only t0 valid
			inter.t = t0;
			inter.Point = ray.Eval(t0);
			inter.Normal = interval.Norm0;
			inter.m_intersected = true;
			return true;
		}else{ 				// both valid
			if (t0 < t1){
				//t0 is smaller pos
				inter.t = t0;
				inter.Point = ray.Eval(t0);
				inter.Normal = interval.Norm0;
				inter.m_intersected = true;
				return true;
			}else{
				//t1 is smaller pos
				inter.t = t1;
				inter.Point = ray.Eval(t1);
				inter.Normal = interval.Norm1;
				inter.m_intersected = true;
				return true;
			}
		}
	}

	return false;
}

Box3d Box::GetBBox(){
	return m_Bbox;
}

ShapeType Box::GetType(){
	return m_Type;
}

Material* Box::GetMaterial(){
	return m_Material;
}

bool Box::IsLight(){
	return m_Material->IsLight();
}

Vector3f Box::Radiance(){
	return m_Material->Radiance();
}

void Box::SamplePoint(Intersection& inter){
	// float E1 = myrandom(RNGen);
	// float E2 = myrandom(RNGen);
	std::cout << "NOT YET IMPLEMENTED\n";
}

float Box::Area(){
	std::cout << "NOT YET IMPLEMENTED\n";
	return 0.0f;
}

/********************************
	IMPLEMENTATION FOR TRIANGLE
*********************************/
Triangle::Triangle(): m_Type(TRIANGLE), m_hasNormals(false) {
}

inline float Max3(float f1, float f2, float f3){
	return std::fmax(std::fmax(f1,f2), f3);
}
inline float Min3(float f1, float f2, float f3){
	return std::fmin(std::fmin(f1,f2), f3);
}

Triangle::Triangle(Vector3f v0, Vector3f v1, Vector3f v2, Material* mat)
: m_Type(TRIANGLE), m_V0(v0), m_V1(v1), m_V2(v2), m_Material(mat), m_hasNormals(false) {
	m_Bbox = Box3d(	Vector3f(Min3(v0[0],v1[0],v2[0]),
							 Min3(v0[1],v1[1],v2[1]),
							 Min3(v0[2],v1[2],v2[2])),
					Vector3f(Max3(v0[0],v1[0],v2[0]),
							 Max3(v0[1],v1[1],v2[1]),
							 Max3(v0[2],v1[2],v2[2])) );
	//TODO: calculate normals


}

Triangle::Triangle(Vector3f v0, Vector3f v1, Vector3f v2, Vector3f n0, Vector3f n1, Vector3f n2, Material* mat)
: m_Type(TRIANGLE), m_V0(v0), m_V1(v1), m_V2(v2), m_N0(n0), m_N1(n1), m_N2(n2), m_Material(mat), m_hasNormals(true){
	m_Bbox = Box3d(	Vector3f(Min3(v0[0],v1[0],v2[0]),
							 Min3(v0[1],v1[1],v2[1]),
							 Min3(v0[2],v1[2],v2[2])),
					Vector3f(Max3(v0[0],v1[0],v2[0]),
							 Max3(v0[1],v1[1],v2[1]),
							 Max3(v0[2],v1[2],v2[2])) );
}

Triangle::~Triangle(){
	// if (m_Material){
	// 	delete m_Material;
	// }
}

// find intersection using bary-centric coordinate
bool Triangle::Intersect(Ray ray, Intersection& inter){
	Vector3f E1 = m_V1 - m_V0;
	Vector3f E2 = m_V2 - m_V0;
	Vector3f p = ray.GetDirection().cross(E2);
	float d = p.dot(E1);

	if(fabs(d)<EPSILON){ // ray parallel to tri 
		return false;
	}
	
	Vector3f S = ray.GetPoint() - m_V0;
	float u = p.dot(S) / d;
	if ((u < EPSILON) || (u > 1)){ // ray intersects plane but outside E2 edge
		return false;
	}

	Vector3f q = S.cross(E1);
	float v = ray.GetDirection().dot(q) / d;
	if ((v < EPSILON) || ((u+v) > 1)) { // ray intersects plane but outside other edges
		return false; 
	}

	float t = E2.dot(q) / d;
	if (t < EPSILON){ // triangle behind ray
		return false;
	}

	inter.t = t;
	inter.Point = ray.Eval(t);
	//TODO: if vertext normals are known,
	//			(1-u-v)N0 + u*N1 + v*N2
	// if (HasNormals()){
	// 	inter.Normal = (1.0f - u - v)*m_N0 + u*m_N1 + v*m_N2;
	// 	//inter.Normal.normalize();
	// }
	inter.Normal = E2.cross(E1).normalized();
	inter.m_intersected = true;
	return true;
}

Box3d Triangle::GetBBox(){
	return m_Bbox;
}

ShapeType Triangle::GetType(){
	return m_Type;
}

Material* Triangle::GetMaterial(){
	return m_Material;
}

bool Triangle::IsLight(){
	return m_Material->IsLight();
}

Vector3f Triangle::Radiance(){
	return m_Material->Radiance();
}

void Triangle::SamplePoint(Intersection& inter){
	// float E1 = myrandom(RNGen);
	// float E2 = myrandom(RNGen);
	std::cout << "NOT YET IMPLEMENTED\n";
}

bool Triangle::HasNormals(){
	return m_hasNormals;
}

float Triangle::Area(){
	std::cout << "NOT YET IMPLEMENTED\n";
	return 0.0f;
}

/*********************************
	IMPLEMENTATION FOR CYLINDER
**********************************/
Cylinder::Cylinder(): m_Type(CYLINDER) {
}

Cylinder::Cylinder(Vector3f b, Vector3f axis, float r, Material* mat)
: m_Type(CYLINDER), m_Base(b), m_Axis(axis), m_radius(r), m_Material(mat) {
	m_q = Quaternionf::FromTwoVectors(m_Axis, Vector3f::UnitZ());

	// CHECK THIS BOUNDING BOX...
	Vector3f e0 = m_Base;
	Vector3f e1 = m_Base + m_Axis;

	Vector3f end0 = Vector3f(	fmin(e0[0],e1[0]),
								fmin(e0[1],e1[1]),
								fmin(e0[2],e1[2])	);
	Vector3f end1 = Vector3f(	fmax(e0[0],e1[0]),
								fmax(e0[1],e1[1]),
								fmax(e0[2],e1[2])	);
	m_Bbox = Box3d(	end0-Vector3f(r,r,r), 
					end1+Vector3f(r,r,r));
}

Cylinder::~Cylinder(){
	// if (m_Material){
	// 	delete m_Material;
	// }
}

bool Cylinder::Intersect(Ray ray, Intersection& inter){
	// starting interval
	Interval interval = Interval().FullRay(); // [0,inf]
	// get z-axis aligned ray

	Ray rayQ = Ray(	m_q._transformVector(ray.GetPoint()-m_Base), 
					m_q._transformVector(ray.GetDirection()	  )	 );
	
	// slab that represents end plates 
	Slab s0 = Slab(0, -m_Axis.norm() , Vector3f(0,0,1));
	// check slab intersection first
	Interval temp = s0.IntersectSlab(rayQ);
	if (!temp.IsValid()){
		return false;
	}
	interval.SetData(PairMax(make_pair(interval.t0,interval.Norm0), make_pair(temp.t0,temp.Norm0)),
					 PairMin(make_pair(interval.t1,interval.Norm1), make_pair(temp.t1,temp.Norm1)) );

	// second intersection check
	Vector3f dir = rayQ.GetDirection();
	Vector3f pointQ = rayQ.GetPoint();
	float a = dir[0]*dir[0] + dir[1]*dir[1]; 								// Dx^2 + Dy^2
	float b = 2.0f*(dir[0]*pointQ[0] + dir[1]*pointQ[1]); 							// Dx*Qx + Dy*Qy
	float c = pointQ[0]*pointQ[0] + pointQ[1]*pointQ[1] - m_radius*m_radius;// Qx^2 + Qy^2 - r^2
	float disc = b*b - 4.0f*a*c; // discriminant of quadratic eq.

	if (disc < EPSILON){
		return false;
	}
	// find t-, t+ and corresponding normals
	float t0 = ((-1.0f)*b - sqrt(disc)) / (2*a);
	float t1 = ((-1.0f)*b + sqrt(disc)) / (2*a);
	Vector3f norm0 = (pointQ + t0*dir);
	norm0[2] = 0.0f;
	norm0 = m_q.conjugate()._transformVector(norm0.normalized());
	Vector3f norm1 = (pointQ + t1*dir);
	norm1[2] = 0.0f;
	norm1 = m_q.conjugate()._transformVector(norm1.normalized());

	interval.SetData(PairMax(make_pair(interval.t0, interval.Norm0), make_pair(t0,norm0)),
					 PairMin(make_pair(interval.t1, interval.Norm1), make_pair(t1,norm1)) );
	// final interval check
	if (!interval.IsValid()){
		return false;
	}

	// return smallest poitive of t0, t1;
	t0 = interval.t0;
	t1 = interval.t1;
	if (t0 < EPSILON){ 	// t0 neg
		if (t1 < EPSILON){	// both neg
			return false;
		}else { 			// only t1 valid
			inter.t = t1;
			inter.Point = ray.Eval(t1);
			inter.Normal = interval.Norm1;
			inter.m_intersected = true;
			return true;
		}
	}else{ 				// t0 pos
		if (t1 < EPSILON) {	// only t0 valid
			inter.t = t0;
			inter.Point = ray.Eval(t0);
			inter.Normal = interval.Norm0;
			inter.m_intersected = true;
			return true;
		}else{ 				// both valid
			if (t0 < t1){
				//t0 is smaller pos
				inter.t = t0;
				inter.Point = ray.Eval(t0);
				inter.Normal = interval.Norm0;
				inter.m_intersected = true;
				return true;
			}else{
				//t1 is smaller pos
				inter.t = t1;
				inter.Point = ray.Eval(t1);
				inter.Normal = interval.Norm1;
				inter.m_intersected = true;
				return true;
			}
		}
	}

	return false;
}

Box3d Cylinder::GetBBox(){
	return m_Bbox;
}

ShapeType Cylinder::GetType(){
	return m_Type;
}

Material* Cylinder::GetMaterial(){
	return m_Material;
}

bool Cylinder::IsLight(){
	return m_Material->IsLight();
}

Vector3f Cylinder::Radiance(){
	return m_Material->Radiance();
}

void Cylinder::SamplePoint(Intersection& inter){
	// float E1 = myrandom(RNGen);
	// float E2 = myrandom(RNGen);
	std::cout << "NOT YET IMPLEMENTED\n";
}

float Cylinder::Area(){
	std::cout << "NOT YET IMPLEMENTED\n";
	return 0.0f;
}