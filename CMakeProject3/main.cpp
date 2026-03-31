#include "rtweekend.h"

#include "camera.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "sphere.h"
#include "triangle.h"
#include "mesh.h"
#include "implementation.h"
#include "matrix.h"

#include "scene.h"
#include "integrator.h"
#include "light.h"

int main()
{
    //std::cout << "Looking in: " << std::filesystem::current_path() << std::endl;

    Scene scene;

    std::unordered_map<std::string, std::shared_ptr<material>> matMap1;

    matMap1["white"] = std::make_shared<lambertian>(color(1, 1, 1));
    matMap1["red"] = std::make_shared<lambertian>(color(1, 0, 0));
    matMap1["green"] = std::make_shared<lambertian>(color(0, 1, 0));
    matMap1["blue"] = std::make_shared<lambertian>(color(0, 0, 1));
    matMap1["light"] = std::make_shared<diffuse_light>(color(20, 20, 20));

    // Perfect mirror (ideal specular)
    auto perfectMirror = std::make_shared<mirror>(color(1.0, 1.0, 1.0));

    // Slightly tinted mirror
    auto redMirror = std::make_shared<mirror>(color(1.0, 0.2, 0.2));

    // Standard glass
    auto glassMaterial = std::make_shared<dielectric>(1.5); // index of refraction ~1.5

    // Water-like material
    auto water = std::make_shared<dielectric>(1.33);

    // Diamond-like
    auto diamond = std::make_shared<dielectric>(2.42);


    auto glossyRed = std::make_shared<microfacet>(
        color(0.1, 0.1, 0.8), // base color: red
        0.0,                   // metallic: 0.0 = dielectric-like, 1.0 = fully metal
        0.2                    // roughness: 0 = perfect mirror, 1 = fully diffuse
    );

    auto subtleGloss = std::make_shared<microfacet>(
        color(0.7, 0.7, 0.7),  // grey color
        0.0,                    // non-metallic
        0.05                    // very low roughness = highly glossy
    );

    auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);

    auto defaultMat1 = make_shared<lambertian>(color(0.8, 0.1, 0.1));

    auto material1 = make_shared<dielectric>(1.5);
    auto material4 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);

    auto material_ = std::make_shared<diffuse_light>(color(15, 15, 15));;

    auto checker = make_shared<checker_texture>(0.32, color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto matChecker = make_shared<lambertian>(checker);

    
    auto earthTex = make_shared<image_texture>("models/earthmap.jpg");
    auto matEarth = make_shared<lambertian>(earthTex);

    
    auto hTex = make_shared<image_texture>("models/Hygieia_C_Hygieia_O.png");
    auto MatHyg = make_shared<lambertian>(hTex);


    

    mesh myMesh;
    if (!loadTinyObjToMesh("models/rabbit.obj", myMesh, matMap1, MatHyg))
    {
        std::cerr << "Failed to load OBJ\n";
        return -1;
    }

    myMesh.build(1);

    scene.add_object(std::make_shared<mesh>(myMesh));

    scene.add_object(make_shared<sphere>(point3(-12, 15, -2), 6.0, matEarth));

    myMesh.build(1);

    camera cam;

    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 1200;
    cam.samples_per_pixel = 40;
    cam.max_depth = 50;

    cam.vfov = 40;
    cam.lookfrom = point3(2, 17, 40);
    cam.lookat = point3(2, 7, -40);
    cam.vup = vec3(0, 1, 0);

    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;

    cam.render(scene);
}