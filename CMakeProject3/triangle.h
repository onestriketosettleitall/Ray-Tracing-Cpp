#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"

class triangle : public hittable
{
public:
    triangle(const point3& v0, const point3& v1, const point3& v2, shared_ptr<material> mat)
        : v0(v0), v1(v1), v2(v2), mat(mat) {}

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
        vec3 n;
        double t, beta, gamma;
        if (!intersect_triangle(r, v0, v1, v2, n, t, beta, gamma))
            return false;

        if (!ray_t.contains(t))
            return false;

        rec.t = t;
        rec.p = r.at(t);

        vec3 outward_normal = unit_vector(n);
        rec.set_face_normal(r, outward_normal);

        rec.mat = mat;

        rec.bary_u = beta;
        rec.bary_v = gamma;
        rec.has_bary = true;

        return true;
    }

private:
    point3 v0, v1, v2;
    shared_ptr<material> mat;

    static bool intersect_triangle(const ray& r, const point3& v0, const point3& v1, const point3& v2,
        vec3& normal, double& t, double& beta, double& gamma)
    {
        const double EPSILON = 1e-8;

        vec3 edge1 = v1 - v0;
        vec3 edge2 = v2 - v0;
        vec3 h = cross(r.direction(), edge2);
        double det = dot(edge1, h);

        if (std::abs(det) < EPSILON)
            return false;

        double inv_det = 1.0 / det;
        vec3 s = r.origin() - v0;
        beta = inv_det * dot(s, h);
        if (beta < 0.0 || beta > 1.0)
            return false;

        vec3 q = cross(s, edge1);
        gamma = inv_det * dot(r.direction(), q);
        if (gamma < 0.0 || beta + gamma > 1.0)
            return false;

        t = inv_det * dot(edge2, q);
        normal = unit_vector(cross(edge1, edge2));
        return true;
    }
};

#endif // TRIANGLE_H
