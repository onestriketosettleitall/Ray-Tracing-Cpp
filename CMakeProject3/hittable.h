#ifndef HITTABLE_H
#define HITTABLE_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

class material;
class hittable;


class hit_record {
public:
    point3 p = point3(0,0,0);
    vec3 normal = vec3(0, 0, 0);;
    shared_ptr<material> mat;
    int emitter_id = -1;
    double t = 0.0;
    bool front_face =  true;
    const hittable* hitObject = nullptr;
    int prim_id = -1;
    double u = 0.0;
    double v = 0.0;
    double bary_u = 0.0;
    double bary_v = 0.0;
    bool has_bary = false;

    void set_face_normal(const ray& r, const vec3& outward_normal) {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.

        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};


class hittable {
public:
    virtual ~hittable() = default;

    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
};


#endif