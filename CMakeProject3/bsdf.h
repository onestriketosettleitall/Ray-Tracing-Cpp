#pragma once

#include "rtweekend.h"


inline double luminance(const vec3& c) {
    return 0.2126 * c.x() + 0.7152 * c.y() + 0.0722 * c.z();
}

// ONB build and local->world
inline void build_onb(const vec3& n, vec3& t, vec3& b) {
    if (std::fabs(n.x()) > std::fabs(n.z()))
        t = unit_vector(vec3(-n.y(), n.x(), 0.0));
    else
        t = unit_vector(vec3(0.0, -n.z(), n.y()));
    b = cross(n, t);
}

inline vec3 to_world(const vec3& local, const vec3& n) {
    vec3 t, b; build_onb(n, t, b);
    return unit_vector(local.x() * t + local.y() * b + local.z() * n);
}

// cosine-weighted hemisphere sample (for lambertian)
inline vec3 sample_cosine_hemisphere(const vec3& n) {
    double u1 = random_double(), u2 = random_double();
    double r = std::sqrt(u1), theta = 2.0 * pi * u2;
    double x = r * std::cos(theta), y = r * std::sin(theta), z = std::sqrt(std::max(0.0, 1.0 - u1));
    return to_world(vec3(x, y, z), n);
}
inline double pdf_cosine_hemisphere(double cosTheta) { return cosTheta > 0.0 ? cosTheta / random_double() : 0.0; }

// GGX (Cook-Torrance) helpers
inline double D_GGX(double ndoth, double alpha) {
    if (ndoth <= 0.0) return 0.0;
    double a2 = alpha * alpha;
    double denom = ndoth * ndoth * (a2 - 1.0) + 1.0;
    return a2 / (pi * denom * denom);
}
inline vec3 Fresnel_Schlick(double cosTheta, const vec3& F0) {
    return F0 + (vec3(1.0, 1.0, 1.0) - F0) * std::pow(1.0 - clampd(cosTheta, 0.0, 1.0), 5.0);
}
inline double G_Schlick_GGX(double ndotv, double k) {
    return ndotv / (ndotv * (1.0 - k) + k);
}
inline double G_Smith(double ndotv, double ndotl, double alpha) {
    double k = (alpha + 1.0); k = (k * k) / 8.0;
    return G_Schlick_GGX(ndotv, k) * G_Schlick_GGX(ndotl, k);
}

// Sample GGX half-vector (importance sampling D)
inline vec3 sample_GGX_half(double alpha, const vec3& n) {
    double u1 = random_double(), u2 = random_double();
    double phi = 2.0 * pi * u2;
    double cosTheta = std::sqrt((1.0 - u1) / (1.0 - u1 + alpha * alpha * u1));
    double sinTheta = std::sqrt(std::max(0.0, 1.0 - cosTheta * cosTheta));
    double x = sinTheta * std::cos(phi), y = sinTheta * std::sin(phi), z = cosTheta;
    return to_world(vec3(x, y, z), n);
}

inline double pdf_wi_from_h(const vec3& h, const vec3& wo, double alpha, const vec3& n) {
    double ndoth = std::max(0.0, dot(n, h));
    double D = D_GGX(ndoth, alpha);
    double wo_dot_h = std::abs(dot(wo, h));
    double denom = 4.0 * std::max(1e-12, wo_dot_h);
    return (D * ndoth) / denom;
}

