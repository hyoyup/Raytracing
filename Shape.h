/*---------------------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.

File Name: Shape.h
Purpose: CS500 Raytracing
Language: C/C++
Platform: Windows: GNU (x86_64-posix-seh-rev1, Built by MinGW-W64 project) 7.1.0
		  Linux: GNU (Ubuntu 5.4.0-6ubuntu1~16.04.4) 5.4.0 20160609
Author: Hyoyup Chung
Creation date: February 11, 2018
-----------------------------------------------------------------------*/

// forward declaration
class Ray;
class Intersection;
class Material;

const float PI = 3.14159f;
typedef Eigen::AlignedBox<float, 3> Box3d;

typedef enum{
	SPHERE,
	BOX,
	TRIANGLE,
	CYLINDER,

	NUM_SHAPES
}ShapeType;

class Shape{
public:
	Material* m_Material;
	ShapeType m_Type;
	Box3d m_Bbox;

	virtual ~Shape(){};
	virtual bool Intersect(Ray, Intersection&) = 0;
	virtual Box3d GetBBox() = 0;
	virtual ShapeType GetType() = 0;
	virtual Material* GetMaterial() = 0;
	virtual bool IsLight()=0;
	virtual Vector3f Radiance() = 0;
	virtual void SamplePoint(Intersection&) = 0;
	virtual float Area() = 0;
};

class Sphere: public Shape{
private:
	ShapeType m_Type;
	Vector3f m_Center;
	float m_radius;
	Box3d m_Bbox;
public:
	Material* m_Material;

	Sphere();
	Sphere(Vector3f, float, Material*);
	virtual ~Sphere();
	virtual bool Intersect(Ray ray, Intersection& inter);
	virtual Box3d GetBBox();
	virtual ShapeType GetType();
	virtual Material* GetMaterial();
	virtual bool IsLight();
	virtual Vector3f Radiance();
	virtual void SamplePoint(Intersection&);
	virtual float Area();
};

class Box: public Shape{
private:
	ShapeType m_Type;
	Vector3f m_Corner;
	Vector3f m_DiagVec;	
	Box3d m_Bbox;
public:
	Material* m_Material;

	Box();
	Box(Vector3f, Vector3f);
	Box(Vector3f, Vector3f, Material*);
	virtual ~Box();
	virtual bool Intersect(Ray ray, Intersection& inter);
	virtual Box3d GetBBox();
	virtual ShapeType GetType();
	virtual Material* GetMaterial();
	virtual bool IsLight();
	virtual Vector3f Radiance();
	virtual void SamplePoint(Intersection&);
	virtual float Area();
	//Vector3f GetCenter();
};

class Triangle: public Shape{
private:
	ShapeType m_Type;
	Vector3f m_V0, m_V1, m_V2;
	Vector3f m_N0, m_N1, m_N2;
	Box3d m_Bbox;
	bool m_hasNormals;
public:
	Material* m_Material;

	Triangle();
	Triangle(Vector3f v0, Vector3f v1, Vector3f v2, Material*);
	Triangle(Vector3f v0, Vector3f v1, Vector3f v2, Vector3f n0, Vector3f n1, Vector3f n2, Material*);
	virtual ~Triangle();
	virtual bool Intersect(Ray ray, Intersection& inter);
	virtual Box3d GetBBox();
	virtual ShapeType GetType();
	virtual Material* GetMaterial();
	virtual bool IsLight();
	virtual Vector3f Radiance();
	virtual void SamplePoint(Intersection&);
	virtual float Area();
	bool HasNormals();
};

class Cylinder: public Shape{
private: 
	ShapeType m_Type;
	Vector3f m_Base;
	Vector3f m_Axis;
	float m_radius;
	Box3d m_Bbox;
	Quaternionf m_q;
public:
	Material* m_Material;

	Cylinder();
	Cylinder(Vector3f, Vector3f, float, Material*);
	~Cylinder();
	virtual bool Intersect(Ray ray, Intersection& inter);
	virtual Box3d GetBBox();
	virtual ShapeType GetType();
	virtual Material* GetMaterial();
	virtual bool IsLight();
	virtual Vector3f Radiance();
	virtual void SamplePoint(Intersection&);
	virtual float Area();
};

