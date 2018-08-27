#include <vector>
#include <iostream>
#include "geom.h"
#include "Scene.h"
#include "Ray.h"
#include "Shape.h"

#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

using namespace Eigen;

const float Radians = PI/180.0f;                     // Convert degrees to radians
const float INF = std::numeric_limits<float>::max(); // KdBVH Scalar for non-intersection
const float EPSILON = 0.000001f;
static int count = 0;

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width*height*3];
    float* dp = data;
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            Color pixel = image[y*width + x];

            //printf("color: %f, %f, %f\n", pixel[0],pixel[1],pixel[2]);

            *dp++ = pixel[0];
            *dp++ = pixel[1];
            *dp++ = pixel[2]; } }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = {0};

    FILE* fp  =  fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width,  height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    
    delete[] data;
}


/*******************************
    IMPLEMENTATION FOR CAMERA
********************************/
Camera::Camera(Vector3f eye, Quaternionf q, float _ry, float w)
: m_Eye(eye), m_Quat(q), m_ry(_ry){
    // camera space coord system
    //m_X = m_ry *m_Width / m_Height * m_Quat._transformVector(Vector3f::UnitX()).normalized(); // rx = ry * W/H
    //m_Y = m_ry * m_Quat._transformVector(Vector3f::UnitY()).normalized();
    //m_Z = -1 * m_Quat._transformVector(Vector3f::UnitZ()).normalized();
}

void Camera::SetScreen(float w, float h){
    m_fWidth = w;
    m_fHeight = h;
}

void Camera::ParseCamInfo(Vector3f eye, Quaternionf q, float _ry){
    m_Eye = eye;
    m_Quat = q;
    m_ry = _ry;
}

void Camera::Finit(){
    m_X = m_ry *m_fWidth / m_fHeight * m_Quat._transformVector(Vector3f::UnitX()).normalized(); // rx = ry * W/H
    m_Y = m_ry * m_Quat._transformVector(Vector3f::UnitY()).normalized();
    m_Z = -1.0f * m_Quat._transformVector(Vector3f::UnitZ()).normalized();
}

Ray Camera::GenerateRay(int x, int y){
    float eps = myrandom(RNGen); //for naive anti-aliasing
    float dx = 2.0f*(static_cast<float>(x)+eps)/m_fWidth - 1.0f;
    float dy = 2.0f*(static_cast<float>(y)+eps)/m_fHeight - 1.0f;
    //std::cout << dx<<" "<<dy<<std::endl;
    return Ray(m_Eye, Vector3f(dx*m_X + dy*m_Y + m_Z).normalized());
}


// for KdBVH<float, 3, Shape*> Tree
static Box3d bounding_box(Shape* obj){
    return obj->GetBBox();
}

class Minimizer{
public:
    typedef float Scalar;       // need Scalar defined for Eigen
    Ray ray;
    Intersection* intersection; // keeps track of min-intersection info.

    Minimizer(const Ray& r, Intersection* inter)
        : ray(r), intersection(inter) {}
    // Called by BVMinimize to intersect the ray with a Shape.
    // should return the intersection t, but should also track
    // the minimum t and it's corresponding intersection info.
    // Return INF to indicate no intersection.
    float minimumOnObject(Shape* obj){
        //printf("current: %f\n", current);
        Intersection temp_inter;
        if (obj->Intersect(ray, temp_inter)){
            if (intersection->t > 0 && intersection->t < temp_inter.t){
                return temp_inter.t;
            }
            else{
                intersection->t = temp_inter.t;
                intersection->m_pShape = obj;
                intersection->Point = temp_inter.Point;
                intersection->Normal = temp_inter.Normal;
                intersection->m_intersected = temp_inter.m_intersected;
                //printf("current: %f\n", temp_inter.t);
                return temp_inter.t;
            }
        }
        else {return INF;}
    }

    // Called by BVMinimize to intersect the ray with a Box3d and
    // returns the t value. This should be similar to the already
    // written box (3 slab) intersection. (The difference begin a
    // distance of zero should be returned if the ray starts within the bbox.)
    // Return INF to indicate no intersection.
    float minimumOnVolume(const Box3d& box){
        Vector3f L = box.min(); // Box corner
        Vector3f U = box.max(); // Box corner

        Vector3f rPoint = ray.GetPoint();

        // ray starts within the bbox
        if ((rPoint[0]>L[0] && rPoint[0]<U[0])&&
            (rPoint[1]>L[1] && rPoint[1]<U[1])&&
            (rPoint[2]>L[2] && rPoint[2]<U[2]) ) {
            return 0.0f;
        }

        Intersection temp_inter;
        Box boxObj = Box(L, U-L, nullptr);
        if (boxObj.Intersect(ray, temp_inter)) {

            return temp_inter.t;
        }
        return INF;
    }
};

/*****************************
    IMPLEMENTATION FOR SCENE
******************************/
Scene::Scene(float diff, float spec, float refr) 
: p_d(diff), p_s(spec), p_r(refr)
{ 
    //realtime = new Realtime();
    m_Camera = Camera();
    currentMat = new Material();
    RussianRoulette = 0.8f;
}

Scene::~Scene(){
    //delete currentMat;
    for(auto shape: m_Shapes){
        delete shape;
    }
    for (auto light: m_Lights){
        delete light;
    }
}

void Scene::Finit(){
    m_Camera.Finit();
    //m_KDTree = KdBVH<float, 3, Shape*>(m_Shapes.begin(), m_Shapes.end());
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    for (auto tri: mesh->triangles){
        Vector3f v0 = mesh->vertices[tri[0]].pnt;
        Vector3f v1 = mesh->vertices[tri[1]].pnt;
        Vector3f v2 = mesh->vertices[tri[2]].pnt;
        // printf("%f, %f, %f\n", v0[0],v0[1],v0[2]);
        // printf("%f, %f, %f\n", v1[0],v1[1],v1[2]);
        // printf("%f, %f, %f\n\n", v2[0],v2[1],v2[2]);
        Vector3f n0 = mesh->vertices[tri[0]].nrm;
        Vector3f n1 = mesh->vertices[tri[1]].nrm;
        Vector3f n2 = mesh->vertices[tri[2]].nrm;
        if ((n0[0]+n0[1] < 0.00001f)|| 
            (n1[0]+n1[1] < 0.00001f)||
            (n2[0]+n2[1] < 0.00001f) ) {
            m_Shapes.push_back(new Triangle(v0,v1,v2,mesh->mat));
            continue;
        }
        m_Shapes.push_back(new Triangle(v0,v1,v2,n0,n1,n2,mesh->mat));
    }
}

Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}


void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        //realtime->setScreen(int(f[1]),int(f[2]));
        width = int(f[1]);
        height = int(f[2]);
        m_Camera.SetScreen(f[1],f[2]);
    }
    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        //realtime->setCamera(Vector3f(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]); 
        m_Camera.ParseCamInfo(Vector3f(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]);
    }
    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        
        //realtime->setAmbient(Vector3f(f[1], f[2], f[3])); }
    }
    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]); 
    }
    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(Vector3f(f[1], f[2], f[3])); 
    }
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        m_Shapes.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat)); 
        if (currentMat->IsLight()){
            m_Lights.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
        }
    }
    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        m_Shapes.push_back(new Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat)); 
        if (currentMat->IsLight()){
            m_Lights.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
        }
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        m_Shapes.push_back(new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat));
        if (currentMat->IsLight()){
            m_Lights.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
        }
    }
    else if (c == "capsule") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        m_Shapes.push_back(new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat));
        if (currentMat->IsLight()){
            m_Lights.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
        }
    }
    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        Matrix4f modelTr = translate(Vector3f(f[2],f[3],f[4]))
                          *scale(Vector3f(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  
        //**************THIS CURRENTLY DOES NOTHING*****************//
    }
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}



void Scene::TraceImageKdBVH(Color* image, std::string hdrName, const int pass, const int endAt){
    KdBVH<float, 3, Shape*> Tree(m_Shapes.begin(), m_Shapes.end());

    Color *tempImage =  new Color[width*height];
    for (int y=0;  y<height;  y++)
        for (int x=0;  x<width;  x++)
            tempImage[y*width + x] = Color(0,0,0);

    int counter = 0;
    int totalCount = 0;
    int imageNum = 1;
    while(true){
        if (counter < pass){
            #pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
            for (int y=0;  y<height;  y++) {
            //fprintf(stderr, "Rendering %4d\r", y);
                for (int x=0;  x<width;  x++) {
                    Color color = TracePathKdBVH(m_Camera.GenerateRay(x, y), Tree);  
                    //printf("%f, %f, %f\n", color[0],color[1],color[2]); 
                    tempImage[y*width + x] += color;
                }
            }
            counter++;
            totalCount++;    
        }else {
            #pragma omp parallel for schedule(dynamic, 1)
            for (int y=0;  y<height;  y++) {
                for (int x=0;  x<width;  x++) {  
                    image[y*width + x] = (image[y*width + x]*(totalCount - static_cast<float>(pass))
                                         + tempImage[y*width + x]) / totalCount ;
                }
            }
            std::string hdrNameOut = hdrName + std::to_string(imageNum) + ".hdr";
            WriteHdrImage(hdrNameOut, width, height, image);
            std::cout << totalCount <<" Hdr Image Written!\n";
            if (totalCount >= endAt){
                std::cout << "COMPLETE!\n";
                break;
            }
            counter = 0;
            imageNum++;
        }
    }
}

Vector3f Scene::TracePathKdBVH(const Ray ray, KdBVH<float,3,Shape*> &tree){
    Intersection intersection; // t = -1.0f initially
    Vector3f color = Vector3f(0,0,0); // accumulated light
    Vector3f weight = Vector3f(1,1,1);// accumulated weight

    // KdBVH Version of Raytracer
    Minimizer minimizer(ray, &intersection);
    float minDist = BVMinimize(tree, minimizer);

    if (!intersection.m_intersected){ // if no inter
        return Vector3f(0,0,0);
    }
    if (intersection.m_pShape->IsLight()){ // if shape is light
        return intersection.m_pShape->Radiance();
    }

    Vector3f w_o = -ray.GetDirection();

    while(myrandom(RNGen) <= RussianRoulette){
        //if (color[0] > 1.0f || color[1] > 1.0f || color[2] > 1.0f) break;
        // Explicit light connection
        Intersection L = SampleLight(); // randomly choose a light and a point on that light
        Vector3f f, monteCarlo, w_i;
        float t;
        float p = PdfLight(L) / GeometryFactor(intersection.Point, L.Point, intersection.Normal, L.Normal);
        
        Intersection inter_explicit;
        w_i = (L.Point - intersection.Point).normalized();
        Ray ray_explicit = Ray(intersection.Point + 0.00001f * w_i, w_i);
        Minimizer minimizer_explicit(ray_explicit, &inter_explicit);
        t = BVMinimize(tree,minimizer_explicit);

        if (p > 0 && inter_explicit.m_intersected && inter_explicit.m_pShape->IsLight() && IsSamePoint(inter_explicit.Point, L.Point)){
            f = fabs(inter_explicit.Normal.dot(w_i)) * (EvalBrdf(BRDFComponent::DIFFUSE, w_o, w_i, inter_explicit) 
                                                        + EvalBrdf(BRDFComponent::SPECULAR, w_o, w_i, inter_explicit));
            monteCarlo = f/p;
            Vector3f expColor = Vector3f(   weight[0] * monteCarlo[0] * inter_explicit.m_pShape->Radiance()[0],
                                            weight[1] * monteCarlo[1] * inter_explicit.m_pShape->Radiance()[1],
                                            weight[2] * monteCarlo[2] * inter_explicit.m_pShape->Radiance()[2]   );
            //printf("explicit color added: %f, %f, %f\n", expColor[0],expColor[1],expColor[2]);
            color += expColor;
            //break;
        }

        // Extend path
        BRDFComponent choice = ChooseBRDF(p_d, p_s, p_r);
        w_i = SampleBrdf(choice, w_o, intersection); // choose a sample direction from P

        Intersection new_inter;
        //Ray new_ray = Ray(intersection.Point+0.000001f*w_i, w_i);
        Ray new_ray = Ray(intersection.Point, w_i);
        Minimizer new_minimizer(new_ray, &new_inter);
        t = BVMinimize(tree,new_minimizer);
        
        if (!new_inter.m_intersected){ // if intersection doesn't exist
            break;
        }
        f = fabs(intersection.Normal.dot(w_i)) * EvalBrdf(choice, w_o, w_i, intersection);
        p = PdfBrdf(choice, w_o, intersection, w_i) * RussianRoulette;
        if (p < EPSILON){ // avoid division by zero or nearly zero
            break;
        }

        monteCarlo = f/p;
        weight = Vector3f(  weight[0] * monteCarlo[0],
                            weight[1] * monteCarlo[1],
                            weight[2] * monteCarlo[2]   );

        // Implicit light connection
        if (new_inter.m_pShape->IsLight()){
            color += Vector3f(  weight[0] * new_inter.m_pShape->Radiance()[0],
                                weight[1] * new_inter.m_pShape->Radiance()[1],
                                weight[2] * new_inter.m_pShape->Radiance()[2]   );
            break;
        }

        // step forward
        intersection = new_inter;
        w_o = -w_i;
    }
    if (color[0] > 50.0f) 
        color[0] /= 50.0f;

    if (color[1] > 50.0f) 
        color[1] /= 50.0f;
    
    if (color[2] > 50.0f) 
        color[2] /= 50.0f;

    return color;
}

Intersection Scene::SampleLight(){
    int lightNum = m_Lights.size();
    float offset = 1.00001f / lightNum;
    unsigned int index = myrandom(RNGen) / offset;
    
    Intersection inter;
    m_Lights[index]->SamplePoint(inter);
    //inter.m_pShape = m_Lights[index];

    return inter;
}

float Scene::PdfLight(const Intersection& inter){

    return 1.0f / (inter.m_pShape->Area() * m_Lights.size());
    //return 1.0f / (m_Lights.size());
}

Vector3f Scene::SampleBrdf(BRDFComponent choice, const Vector3f w_o, const Intersection inter){
    float E1 = myrandom(RNGen);
    float E2 = myrandom(RNGen);
    if (choice == BRDFComponent::DIFFUSE){
        return SampleLobe(inter.Normal, sqrt(E1), 2.0f * PI * E2);
    }
    else if (choice == BRDFComponent::SPECULAR){
        Vector3f m = SampleLobe(inter.Normal, pow(E1, 1.0f / (1.0f + inter.m_pShape->GetMaterial()->alpha)), 2.0f*PI*E2);
        return 2.0f*(w_o.dot(m)) * m - w_o;
    }
}

float Scene::PdfBrdf(BRDFComponent choice, const Vector3f w_o, const Intersection inter, const Vector3f w_i){
    Vector3f norm = inter.Normal;
    if (choice == BRDFComponent::DIFFUSE){
        return p_d * fabs(w_i.dot(norm)) / PI;
    }
    else if (choice == BRDFComponent::SPECULAR){
        Vector3f m = (w_o + w_i).normalized();
        return p_s * DFactor(m, norm, inter.m_pShape->GetMaterial()->alpha) * fabs(m.dot(norm)) * (1.0f / (4.0f * fabs(w_i.dot(m))));
    }
}

Vector3f Scene::EvalBrdf(BRDFComponent choice, const Vector3f w_o, const Vector3f w_i, const Intersection inter){
    if (choice == BRDFComponent::DIFFUSE){
        return inter.m_pShape->GetMaterial()->Kd / PI;
    }
    else if (choice == BRDFComponent::SPECULAR){
        Vector3f m = (w_o + w_i).normalized();
        Vector3f norm = inter.Normal;
        Vector3f k_s = inter.m_pShape->GetMaterial()->Ks;
        float a = inter.m_pShape->GetMaterial()->alpha;
        //printf("ks: %f, %f, %f, %f\n", k_s[0],k_s[1],k_s[2],a);
        return DFactor(m,norm,a) * GeomAtten(w_i,m,norm,a) * GeomAtten(w_o,m,norm,a) * Fresnel(w_i.dot(m),k_s)
                / (4.0f * fabs(w_i.dot(norm)) * fabs(w_o.dot(norm)));
    }    
}

float Scene::GeometryFactor(const Vector3f p1, const Vector3f p2, const Vector3f n1, const Vector3f n2){
    Vector3f D = p1 - p2;
    float DdotD = D.dot(D);
    return fabs(n1.dot(D) * n2.dot(D) / (DdotD * DdotD));
}

Vector3f Scene::SampleLobe(Vector3f N, float c, float delta){
    float s = sqrt(1 - c*c);
    Vector3f K = Vector3f(s*cosf(delta), s*sinf(delta), c).normalized();
    Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(),N); // quat to rotate Z to N
    return q._transformVector(K);
}


bool Scene::IsSamePoint(Vector3f p0, Vector3f p1){
    return ((fabs(p0[0] - p1[0]) < EPSILON)&&
            (fabs(p0[0] - p1[0]) < EPSILON)&&
            (fabs(p0[0] - p1[0]) < EPSILON) );
}

bool Scene::IsSameLight(Shape* obj0, Shape* obj1){
    return false;
}

BRDFComponent Scene::ChooseBRDF(float p_diff, float p_spec, float p_refr){
    float diff = p_diff;
    float spec = p_diff + p_spec;

    float E0 = myrandom(RNGen);
    if (E0 < diff){
        return BRDFComponent::DIFFUSE;
    }else if (E0 < spec){
        return BRDFComponent::SPECULAR;
    }else {
        return BRDFComponent::REFRACTION;
    }
}

Vector3f Scene::Fresnel(float d, Vector3f Ks){
    Vector3f oneMinusKs = Vector3f(1.0f - Ks[0], 1.0f - Ks[1], 1.0f - Ks[2]);
    return Ks + oneMinusKs * pow((1.0f - fabs(d)), 5.0f);
}

float Scene::GeomAtten(Vector3f v, Vector3f m, Vector3f norm, float alpha){
    float vDotN = v.dot(norm);
    if (vDotN > 1.0f){
        return 1.0f;
    }
    float tangent = sqrt((1.0f - vDotN * vDotN)) / vDotN;

    if (fabs(tangent) < EPSILON){
        return 1.0f;
    }
    float d = v.dot(m) / vDotN;
    if (d < EPSILON){
        return 0.0f;
    }

    float a = sqrt(alpha / 2.0f + 1.0f) / fabs(tangent);
    float aa = a*a;
    if (a < 1.6f){
       return (3.535f*a + 2.181*aa) / (1.0f + 2.276*a + 2.577*aa);
    }else{
        return 1.0f;
    }
}

float Scene::DFactor(Vector3f m, Vector3f norm, float alpha){
    float mDotN = m.dot(norm);
    if (mDotN < EPSILON)
        return 0.0f;
    else
        return (alpha + 2.0f) / (2.0f * PI) * pow(mDotN, alpha);
}
