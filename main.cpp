///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron 
//     updated by Hyoyup Chung in 2018
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>
#include <iostream>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
    #include <stdlib.h>
    #include <time.h> 
    #include <chrono>
#endif

#include "geom.h"
//#include "raytrace.h"
#include "Scene.h"
#include "Ray.h"
#include "Shape.h"

// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene(const std::string inName, Scene* scene)
{
    std::ifstream input(inName.c_str());
    if (input.fail()) {
        std::cerr << "File not found: "  << inName << std::endl;
        fflush(stderr);
        exit(-1); }

    // For each line in file
    for (std::string line; getline(input, line); ) {
        std::vector<std::string> strings;
        std::vector<float> floats;
        
        // Parse as parallel lists of strings and floats
        std::stringstream lineStream(line);
        for (std::string s; lineStream >> s; ) { // Parses space-separated strings until EOL
            float f;
            //std::stringstream(s) >> f; // Parses an initial float into f, or zero if illegal
            if (!(std::stringstream(s) >> f)) f = nan(""); // An alternate that produced NANs
            floats.push_back(f);
            strings.push_back(s); }

        if (strings.size() == 0) continue; // Skip blanks lines
        if (strings[0][0] == '#') continue; // Skip comment lines
        
        // Pass the line's data to Command(...)
        scene->Command(strings, floats);
    }

    input.close();
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    //bool linearEnabled = true; // for benchmark linear vs KdBVH
    float p_diff = 0.7f;
    float p_spec = 0.3f;
    float p_refr = 0.0f; // NO Refraction for proj3

    Scene* scene = new Scene(p_diff,p_spec,p_refr);

    // Read the command line argument
    std::string inName =  (argc > 1) ? argv[1] : "testscene.scn";
    std::string hdrName = "testscene";

    //hdrName.replace(hdrName.size()-3, hdrName.size(), "hdr");

    // Read the scene, calling scene.Command for each line.
    ReadScene(inName, scene);

    scene->Finit();

    // Allocate and clear an image array
    Color *image =  new Color[scene->width*scene->height];
    for (int y=0;  y<scene->height;  y++)
        for (int x=0;  x<scene->width;  x++)
            image[y*scene->width + x] = Color(0,0,0);


    int writeEvery = 64;
    int endAt = 64;
    std::cout << "every: "<< writeEvery <<std::endl;
    std::cout << "end at: "<< endAt <<std::endl;

    std::cout << "KdBVH: Raytracer starting now...\n";
    auto start = std::chrono::high_resolution_clock::now();
    
    scene->TraceImageKdBVH(image, hdrName, writeEvery, endAt);

    auto end = std::chrono::high_resolution_clock::now();
    auto dur = end - start;
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(dur);
    std::cout << "\ttime in seconds: " << f_secs.count() << '\n';
    std::cout << "KdBVH: Raytracer finished...\n\n";



    // RayTrace the image using KdBVH
    // std::cout << "KdBVH: Raytracer starting now...\n";
    // auto start = std::chrono::high_resolution_clock::now();
    // scene->TraceImageKdBVH(image, 1);
    // auto end = std::chrono::high_resolution_clock::now();
    // auto dur = end - start;
    // auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(dur);
    // std::cout << "\ttime in seconds: " << f_secs.count() << '\n';
    // std::cout << "KdBVH: Raytracer finished...\n\n";

    // Write the image
    //WriteHdrImage(hdrName, scene->width, scene->height, image);

    delete[] image;
    delete scene;
}
