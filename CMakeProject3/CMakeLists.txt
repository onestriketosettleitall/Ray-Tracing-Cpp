# CMakeList.txt : CMake project for CMakeProject3, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (CMakeProject3 "main.cpp" "camera.h" "color.h" "hittable.h" "hittable_list.h" "interval.h" "material.h" "ray.h" "rtweekend.h" "sphere.h" "vec3.h" "triangle.h"  "mesh.h" "tiny_obj_loader.h"   "implementation.h"  "matrix.h"            "scene.h" "light.h" "integrator.h" "bsdf.h" "texture.h" "stb_image.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET CMakeProject3 PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
