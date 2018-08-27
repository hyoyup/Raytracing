/*---------------------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.

File Name: Scene.h 
Purpose: CS500 Raytracing
Language: C/C++
Platform: Windows: GNU (x86_64-posix-seh-rev1, Built by MinGW-W64 project) 7.1.0
          Linux: GNU (Ubuntu 5.4.0-6ubuntu1~16.04.4) 5.4.0 20160609
Author: Hyoyup Chung
Creation date: February 11, 2018
-----------------------------------------------------------------------*/
// #include "geom.h"
// #include "Ray.h"
// #include "Shape.h"

// #include <Eigen/StdVector>
// #include <Eigen_unsupported/Eigen/BVH>
// const float INF = std::numeric_limits<float>::max();
//typedef Eigen::AlignedBox<float, 3> Box3d; // The BV type provided by Eigen

class Ray;
class Intersection;
class Shape;

class Camera{
private:
    Vector3f m_Eye;
    Quaternionf m_Quat;
    float m_ry; // rx = ry * W/H
    float m_fWidth, m_fHeight;
    Vector3f m_X, m_Y, m_Z;
public:
    Camera(){}
    Camera(Vector3f, Quaternionf, float, float w = 0.1);
    void SetScreen(float w, float h);
    void ParseCamInfo(Vector3f eye, Quaternionf q, float _ry);
    void Finit();
    Ray GenerateRay(int x, int y);
};

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    Vector3f Kd, Ks;
    float alpha;
    unsigned int texid;

    virtual bool IsLight() { return false; }
    virtual Vector3f Radiance() { return Kd / 3.14159f; }

    Material()  : Kd(Vector3f(0.0, 0.0, 0.0)), Ks(Vector3f(0,0,0)), alpha(1.0), texid(0) {}
    Material(const Vector3f d, const Vector3f s, const float a) 
        : Kd(d), Ks(s), alpha(a), texid(0) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }

    //void setTexture(const std::string path);
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:
    Light(const Vector3f e) : Material() { Kd = e; }
    virtual bool IsLight() { return true; }
    virtual Vector3f Radiance() { return Kd; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
////////////////////////////////////////////////////////////////////////
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
    
class VertexData
{
 public:
    Vector3f pnt;
    Vector3f nrm;
    Vector2f tex;
    Vector3f tan;
    VertexData(const Vector3f& p, const Vector3f& n, const Vector2f& t, const Vector3f& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData {
    std::vector<VertexData> vertices;
    std::vector<TriData> triangles;
    Material *mat;
};

enum BRDFComponent{
    DIFFUSE,
    SPECULAR,
    REFRACTION
};

class Scene {
private:
    // probability to choose each component (diff, spec, refr)
    float p_d, p_s, p_r; 

    /* Reflective Object's BRDF Methods*/

    // choose one light(uniformly) randomly and point on that light randomly
    Intersection SampleLight();
    // 1 / (areaOfLight * NumberOfLight)
    float PdfLight(const Intersection& inter);

    /* Emissive Object's Light Methods */

    // sample a direction with a distribution that matches the (N dot W_i) term
    Vector3f SampleBrdf(BRDFComponent choice, const Vector3f w_o, const Intersection inter);
    // (N dot W_i) / PI
    float PdfBrdf(BRDFComponent choice, const Vector3f w_o, const Intersection inter, const Vector3f w_i);
    // BRDF method
    Vector3f EvalBrdf(BRDFComponent choice, const Vector3f w_o, const Vector3f w_i, const Intersection inter);

    /* Auxiliary Functions */

    // convert between angular measure and area measure
    float GeometryFactor(const Vector3f p1, const Vector3f p2, const Vector3f n1, const Vector3f n2);
    // choose a direction vector distributed around a given vector N
    Vector3f SampleLobe(Vector3f N, float c, float delta);
    // checks if the two points are the same
    bool IsSamePoint(Vector3f, Vector3f);
    // checks if the two lights are the same (assume passed in shapes are lights)
    bool IsSameLight(Shape*, Shape*);
    // chooses brdf component to extend path onto (diff, refl, or refr)
    //      ASSUME p_diff + p_spec + p_refr = 1.0f
    BRDFComponent ChooseBRDF(float p_diff, float p_spec, float p_refr);

    /* Microfacet Models for Reflection(SPECULAR) */

    // Fresnel Term
    Vector3f Fresnel(float d, Vector3f Ks);
    // Geometric Attenuation
    float GeomAtten(Vector3f v, Vector3f m, Vector3f norm, float alpha);
    // DistributionFactor
    float DFactor(Vector3f m, Vector3f norm, float alpha);
public: 
    unsigned int width, height;
    float RussianRoulette;
    Material* currentMat;
    Camera m_Camera;
    std::vector<Shape*> m_Shapes;
    std::vector<Shape*> m_Lights;

    Scene(float diff, float spec, float refr);
    ~Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4f& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImageKdBVH(Color* image, const std::string, const int pass, const int endAt);
    

    // Trace given ray and return a corresponding color for such pixel
    Vector3f TracePathKdBVH(const Ray, KdBVH<float,3,Shape*> &ptr);
};
