#pragma once
#include "vec3.h"
#include "hittable.h"
#include "mesh.h"
#include "sphere.h"
#include "rtweekend.h"

#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>

// ----------------------------------------------------
// LightSample
// ----------------------------------------------------
struct LightSample {
    point3 p;     // sampled point on light
    vec3   n;     // surface normal at p
    color  Le;    // emitted radiance
    double pdfA;  // pdf w.r.t area
};

// ----------------------------------------------------
// Abstract Light Base
// ----------------------------------------------------
class Light {
public:
    virtual ~Light() = default;

    // sample a point on the light, given reference point in scene
    virtual LightSample sample(const point3& ref) const = 0;

    // total surface area of emitter
    virtual double area() const = 0;

    // pdf wrt area for a given point on light
    virtual double pdf_area_at_point(const point3& p) const = 0;

    // evaluate emitted radiance (Le)
    virtual color evaluate(const point3& p, const vec3& n, const vec3& wi) const = 0;

    // geometry pointer (optional)
    virtual std::shared_ptr<hittable> geometry() const { return nullptr; }
};

// ----------------------------------------------------
// SphereLight (cosine-weighted hemisphere sampling)
// ----------------------------------------------------
class SphereLight : public Light {
public:
    std::shared_ptr<sphere> geom;
    color radiance;

    SphereLight(std::shared_ptr<sphere> s, const color& L)
        : geom(std::move(s)), radiance(L) {
    }

    double area() const override {
        const double r = geom->get_radius();
        return 4.0 * pi * r * r;
    }

    LightSample sample(const point3& ref) const override {
        LightSample ls;

        const point3 center = geom->get_center();
        const double radius = geom->get_radius();

        // direction from ref to sphere center
        vec3 wc = center - ref;
        double dist2 = wc.length_squared();
        double dist = std::sqrt(dist2);
        wc /= dist; // normalize

        // compute max cone angle subtended by sphere
        double sinThetaMax2 = radius * radius / dist2;
        double cosThetaMax = std::sqrt(std::max(0.0, 1.0 - sinThetaMax2));

        // sample direction within cone using cosine-weighted solid angle
        double u1 = random_double();
        double u2 = random_double();

        double cosTheta = (1.0 - u1) + u1 * cosThetaMax;
        double sinTheta = std::sqrt(std::max(0.0, 1.0 - cosTheta * cosTheta));
        double phi = 2.0 * pi * u2;

        // build orthonormal basis around wc
        vec3 w = wc;
        vec3 u = unit_vector(cross((std::fabs(w.x()) > 0.1 ? vec3(0, 1, 0) : vec3(1, 0, 0)), w));
        vec3 v = cross(w, u);

        // direction towards sampled point
        vec3 dir = sinTheta * std::cos(phi) * u +
            sinTheta * std::sin(phi) * v +
            cosTheta * w;

        // hit point on sphere surface
        ls.p = center + radius * (-dir);  // point visible to ref
        ls.n = unit_vector(ls.p - center);
        ls.Le = radiance;

        // pdf wrt solid angle (uniform over cone)
        double solidAngle = 2.0 * pi * (1.0 - cosThetaMax);
        double pdfW = 1.0 / solidAngle;

        // convert to area pdf: pdfA = pdfW * (dist^2 / |n·wi|)
        double cosAtLight = std::fabs(dot(-dir, ls.n));
        ls.pdfA = (cosAtLight <= 0) ? 0.0 : pdfW * (cosAtLight / dist2);

        return ls;
    }

    double pdf_area_at_point(const point3& /*p*/) const override {
        return 1.0 / area();
    }

    color evaluate(const point3& /*p*/, const vec3& /*n*/, const vec3& /*wi*/) const override {
        return radiance;
    }

    std::shared_ptr<hittable> geometry() const override { return geom; }
};

// ----------------------------------------------------
// MeshLight (area-weighted triangle sampling)
// ----------------------------------------------------
class MeshLight : public Light {
public:
    std::shared_ptr<mesh> geom;
    std::vector<int> triIndices;
    std::vector<color> triRadiance;
    std::vector<double> cdf;
    double totalArea = 0.0;

    std::unordered_map<int, int> primToLocal;

    MeshLight(std::shared_ptr<mesh> m,
        const std::vector<int>& indices,
        const std::vector<color>& radiances)
        : geom(std::move(m)), triIndices(indices), triRadiance(radiances)
    {
        cdf.resize(triIndices.size());
        totalArea = 0.0;

        for (size_t i = 0; i < triIndices.size(); ++i) {
            int tri = triIndices[i];
            double a = (tri >= 0 && (size_t)tri < geom->triArea.size())
                ? geom->triArea[tri] : 0.0;
            totalArea += a;
            cdf[i] = totalArea;
            primToLocal[tri] = (int)i;
        }

        if (totalArea > 0.0) {
            for (auto& v : cdf) v /= totalArea; // normalize
        }
    }

    double area() const override { return totalArea; }

    LightSample sample(const point3& /*ref*/) const override {
        LightSample ls;
        if (totalArea <= 0.0) { ls.pdfA = 0.0; ls.Le = color(0, 0, 0); return ls; }

        // pick triangle by area-weighted CDF
        double r = random_double();
        auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
        size_t idx = std::distance(cdf.begin(), it);
        if (idx >= triIndices.size()) idx = triIndices.size() - 1;
        int tri = triIndices[idx];

        const Face& F = geom->faces[tri];
        const point3& p0 = geom->positions[F.v[0]];
        const point3& p1 = geom->positions[F.v[1]];
        const point3& p2 = geom->positions[F.v[2]];

        // uniform barycentric sample
        double u = random_double(), v = random_double();
        double su = std::sqrt(u);
        double b0 = 1.0 - su;
        double b1 = su * (1.0 - v);
        double b2 = su * v;

        ls.p = b0 * p0 + b1 * p1 + b2 * p2;
        ls.n = geom->triFaceNormal[tri];
        ls.Le = triRadiance[idx];
        ls.pdfA = 1.0 / totalArea;

        return ls;
    }

    double pdf_area_at_point(const point3& /*p*/) const override {
        return (totalArea > 0.0) ? (1.0 / totalArea) : 0.0;
    }

    double pdf_area_for_prim(int prim_id) const {
        if (totalArea <= 0.0) return 0.0;
        return (primToLocal.find(prim_id) != primToLocal.end())
            ? (1.0 / totalArea) : 0.0;
    }

    color radiance_for_prim(int prim_id) const {
        auto it = primToLocal.find(prim_id);
        if (it == primToLocal.end()) return color(0, 0, 0);
        return triRadiance[it->second];
    }

    color evaluate(const point3& /*p*/, const vec3& /*n*/, const vec3& /*wi*/) const override {
        return color(0, 0, 0); // integrator should use radiance_for_prim
    }

    std::shared_ptr<hittable> geometry() const override { return geom; }
};
