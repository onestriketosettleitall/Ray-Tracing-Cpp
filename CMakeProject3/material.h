#ifndef MATERIAL_H
#define MATERIAL_H

#include <memory>
#include <cmath>
#include <algorithm>

#include "vec3.h"
#include "hittable.h"
#include "bsdf.h"
#include "texture.h" // <--- NEW: Include the texture system
#include "rtweekend.h" 

// Use color alias
using color = vec3;

inline vec3 random_in_unit_sphere() {
    while (true) {
        vec3 p = vec3(random_double(-1, 1),
            random_double(-1, 1),
            random_double(-1, 1));
        if (p.length_squared() >= 1) continue;
        return p;
    }
}

// BSDF result for sampling
struct BSDFSample {
    vec3 wi;
    color f;     // raw BRDF (not multiplied by cos)
    double pdf;  // pdf w.r.t solid angle
    bool delta;
    bool is_reflection;
    BSDFSample() : wi(0, 0, 0), f(0, 0, 0), pdf(0.0), delta(false), is_reflection(false) {}
};

// Unified material base (emission + BRDF API)
class material {
public:
    virtual ~material() = default;

    // Emitted radiance (area lights override). Default = black.
    virtual color emitted(const hit_record& rec) const { (void)rec; return color(0, 0, 0); }

    // Evaluate BRDF f(wo,wi) at hit.
    virtual color evaluate(const vec3& wo, const vec3& wi, const hit_record& rec) const = 0;

    // Sample wi given outgoing wo.
    virtual BSDFSample sample(const vec3& wo, const hit_record& rec) const = 0;

    // pdf for given pair (wo,wi)
    virtual double pdf(const vec3& wo, const vec3& wi, const hit_record& rec) const = 0;
};

// ---------------- Lambertian ----------------
class lambertian : public material {
public:
    std::shared_ptr<texture> albedo;

    // Constructor taking a texture (e.g., image or checker)
    lambertian(std::shared_ptr<texture> a) : albedo(a) {}

    // Constructor taking a color (automatically wraps in solid_color)
    lambertian(const color& a) : albedo(std::make_shared<solid_color>(a)) {}

    virtual color evaluate(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo;
        // Sample texture at UV coordinates
        return albedo->value(rec.u, rec.v, rec.p) / pi;
    }

    virtual BSDFSample sample(const vec3& wo, const hit_record& rec) const override {
        (void)wo;
        BSDFSample bs;
        bs.wi = sample_cosine_hemisphere(rec.normal);

        // Sample texture at UV coordinates
        bs.f = albedo->value(rec.u, rec.v, rec.p) / pi;

        bs.pdf = pdf_cosine_hemisphere(std::max(0.0, dot(rec.normal, bs.wi)));
        bs.delta = false;
        return bs;
    }

    virtual double pdf(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo;
        return pdf_cosine_hemisphere(std::max(0.0, dot(rec.normal, wi)));
    }
};

// ---------------- Mirror (ideal specular delta) ----------------
class mirror : public material {
public:
    std::shared_ptr<texture> reflectance; // Updated to texture (allows tinted mirrors)

    mirror(std::shared_ptr<texture> r) : reflectance(r) {}
    mirror(const color& r = color(1, 1, 1)) : reflectance(std::make_shared<solid_color>(r)) {}

    virtual color evaluate(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec;
        return color(0, 0, 0);
    }

    virtual BSDFSample sample(const vec3& wo, const hit_record& rec) const override {
        BSDFSample bs;
        vec3 refl = reflect(-wo, rec.normal);
        bs.wi = unit_vector(refl);
        bs.pdf = 1.0;

        // Get color from texture
        color R = reflectance->value(rec.u, rec.v, rec.p);

        double cos_i = std::abs(dot(rec.normal, bs.wi));
        if (cos_i < 1e-12) bs.f = color(0, 0, 0);
        else bs.f = R / cos_i;

        bs.delta = true;
        return bs;
    }

    virtual double pdf(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec;
        return 0.0;
    }
};

// ---------------- Metal (rough specular approximated as delta-like) ----------------
class metal : public material {
public:
    std::shared_ptr<texture> albedo; // Updated to texture
    double fuzz;

    metal(std::shared_ptr<texture> a, double f = 0.0) : albedo(a), fuzz(f < 1 ? f : 1) {}
    metal(const color& a, double f = 0.0) : albedo(std::make_shared<solid_color>(a)), fuzz(f < 1 ? f : 1) {}

    virtual color evaluate(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec;
        return color(0, 0, 0);
    }

    virtual BSDFSample sample(const vec3& wo, const hit_record& rec) const override {
        BSDFSample bs;
        vec3 ideal = reflect(-wo, rec.normal);
        vec3 jitter = fuzz * random_in_unit_sphere();
        vec3 dir = unit_vector(ideal + jitter);

        bs.wi = dir;
        bs.delta = true;
        bs.pdf = 1.0;

        // Get color from texture
        color alb = albedo->value(rec.u, rec.v, rec.p);

        double cos_i = std::abs(dot(rec.normal, bs.wi));
        if (cos_i < 1e-12 || dot(bs.wi, rec.normal) <= 0) bs.f = color(0, 0, 0);
        else bs.f = alb / cos_i;

        return bs;
    }

    virtual double pdf(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec;
        return 0.0;
    }
};

// ---------------- Dielectric (Glass) ----------------
// Dielectrics usually rely on IOR, but surface tint is possible. 
// Kept simple (no texture) for now as requested, but easily adaptable.
class dielectric : public material {
public:
    double ir; // index of refraction

    dielectric(double ref_index) : ir(ref_index) {}

    virtual color evaluate(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec;
        return color(0, 0, 0);
    }

    virtual BSDFSample sample(const vec3& wo, const hit_record& rec) const override {
        BSDFSample bs;
        bs.delta = true;
        bs.pdf = 1.0;
        bs.is_reflection = false;

        vec3 unit_wo = unit_vector(wo);
        vec3 unit_incident = -unit_wo;

        bool entering = rec.front_face;
        double eta_i = entering ? 1.0 : ir;
        double eta_t = entering ? ir : 1.0;
        double etai_over_etat = eta_i / eta_t;

        double cos_theta = std::fmin(std::fmax(dot(-unit_incident, rec.normal), -1.0), 1.0);
        double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));

        bool tir = (etai_over_etat * sin_theta) > 1.0;

        double reflect_prob;
        if (tir) {
            reflect_prob = 1.0;
        }
        else {
            reflect_prob = schlick_reflectance(cos_theta, eta_i, eta_t);
        }

        if (random_double() < reflect_prob) {
            vec3 refl = reflect(unit_incident, rec.normal);
            bs.wi = unit_vector(refl);
            bs.is_reflection = true;
            bs.f = color(1.0, 1.0, 1.0);
        }
        else {
            vec3 refr;
            bool ok = refract_vec(unit_incident, rec.normal, etai_over_etat, refr);
            if (!ok) {
                vec3 refl = reflect(unit_incident, rec.normal);
                bs.wi = unit_vector(refl);
                bs.is_reflection = true;
                bs.f = color(1.0, 1.0, 1.0);
            }
            else {
                bs.wi = unit_vector(refr);
                bs.is_reflection = false;
                double eta_ratio = etai_over_etat;
                double s = eta_ratio * eta_ratio;
                bs.f = color(s, s, s);
            }
        }
        return bs;
    }

    virtual double pdf(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec;
        return 0.0;
    }

private:
    static double schlick_reflectance(double cos_theta, double eta_i, double eta_t) {
        double r0 = (eta_i - eta_t) / (eta_i + eta_t);
        r0 = r0 * r0;
        cos_theta = std::fmin(std::fmax(cos_theta, 0.0), 1.0);
        return r0 + (1.0 - r0) * std::pow(1.0 - cos_theta, 5.0);
    }
};

// ---------------- Microfacet Cook-Torrance GGX ----------------
class microfacet : public material {
public:
    std::shared_ptr<texture> baseColor; // Updated to texture
    double metallic;
    double roughness;

    // Constructor with texture
    microfacet(std::shared_ptr<texture> base, double m = 0.0, double r = 0.5) :
        baseColor(base), metallic(clampd(m, 0.0, 1.0)), roughness(clampd(r, 0.01, 1.0)) {
    }

    // Constructor with color
    microfacet(const color& base = color(0.8, 0.8, 0.8), double m = 0.0, double r = 0.5) :
        baseColor(std::make_shared<solid_color>(base)), metallic(clampd(m, 0.0, 1.0)), roughness(clampd(r, 0.01, 1.0)) {
    }

    virtual color evaluate(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        vec3 n = rec.normal;
        double ndotwi = std::max(0.0, dot(n, wi));
        double ndotwo = std::max(0.0, dot(n, wo));
        if (ndotwi <= 0 || ndotwo <= 0) return color(0, 0, 0);

        // Fetch albedo from texture
        color albedo = baseColor->value(rec.u, rec.v, rec.p);

        vec3 F0 = vec3(0.04, 0.04, 0.04);
        F0 = F0 * (1.0 - metallic) + albedo * metallic;
        double alpha = roughness * roughness;

        vec3 h = unit_vector(wi + wo);
        double ndoth = std::max(0.0, dot(n, h));
        double D = D_GGX(ndoth, alpha);
        double G = G_Smith(std::max(0.0, dot(n, wi)), std::max(0.0, dot(n, wo)), alpha);
        vec3 F = Fresnel_Schlick(std::max(0.0, dot(wo, h)), F0);

        vec3 spec = (D * G) * F / (4.0 * ndotwi * ndotwo + 1e-12);
        color diff = (1.0 - metallic) * (albedo / pi);
        return diff + spec;
    }

    virtual BSDFSample sample(const vec3& wo, const hit_record& rec) const override {
        BSDFSample bs;

        // Fetch albedo from texture for probability calculations
        color albedo = baseColor->value(rec.u, rec.v, rec.p);

        vec3 F0 = vec3(0.04, 0.04, 0.04);
        F0 = F0 * (1.0 - metallic) + albedo * metallic;
        double specProb = clampd(luminance(F0), 0.05, 0.95);
        double r = random_double();
        double alpha = roughness * roughness;

        if (r < specProb) {
            // Specular sample
            vec3 h = sample_GGX_half(alpha, rec.normal);
            vec3 wi = reflect(-wo, h);
            if (dot(wi, rec.normal) <= 1e-6) {
                bs.pdf = 0.0; bs.f = color(0, 0, 0); bs.wi = wi; bs.delta = false; return bs;
            }
            bs.wi = unit_vector(wi);
            bs.f = evaluate(wo, bs.wi, rec); // evaluate calls texture internally
            double pdf_spec = pdf_wi_from_h(h, wo, alpha, rec.normal);
            double pdf_diff = pdf_cosine_hemisphere(std::max(0.0, dot(rec.normal, bs.wi)));
            bs.pdf = specProb * pdf_spec + (1.0 - specProb) * pdf_diff;
            bs.delta = false;
            return bs;
        }
        else {
            // Diffuse sample
            bs.wi = sample_cosine_hemisphere(rec.normal);
            bs.f = evaluate(wo, bs.wi, rec); // evaluate calls texture internally
            double pdf_diff = pdf_cosine_hemisphere(std::max(0.0, dot(rec.normal, bs.wi)));
            vec3 h = unit_vector(wo + bs.wi);
            double pdf_spec = pdf_wi_from_h(h, wo, alpha, rec.normal);
            bs.pdf = specProb * pdf_spec + (1.0 - specProb) * pdf_diff;
            bs.delta = false;
            return bs;
        }
    }

    virtual double pdf(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        // We need albedo to calculate specProb to weight the PDF correctly
        color albedo = baseColor->value(rec.u, rec.v, rec.p);

        vec3 F0 = vec3(0.04, 0.04, 0.04);
        F0 = F0 * (1.0 - metallic) + albedo * metallic;
        double specProb = clampd(luminance(F0), 0.05, 0.95);

        double pdf_diff = pdf_cosine_hemisphere(std::max(0.0, dot(rec.normal, wi)));
        vec3 h = unit_vector(wo + wi);
        double alpha = roughness * roughness;
        double pdf_spec = pdf_wi_from_h(h, wo, alpha, rec.normal);

        return specProb * pdf_spec + (1.0 - specProb) * pdf_diff;
    }
};

// ---------------- Emissive material ----------------
class diffuse_light : public material {
public:
    std::shared_ptr<texture> emit; // Updated to texture

    diffuse_light(std::shared_ptr<texture> a) : emit(a) {}
    diffuse_light(const color& c) : emit(std::make_shared<solid_color>(c)) {}

    virtual color emitted(const hit_record& rec) const override {
        if (rec.front_face) {
            return emit->value(rec.u, rec.v, rec.p);
        }
        return color(0, 0, 0);
    }

    virtual color evaluate(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec; return color(0, 0, 0);
    }
    virtual BSDFSample sample(const vec3& wo, const hit_record& rec) const override {
        (void)wo; (void)rec; return BSDFSample();
    }
    virtual double pdf(const vec3& wo, const vec3& wi, const hit_record& rec) const override {
        (void)wo; (void)wi; (void)rec; return 0.0;
    }
};

#endif // MATERIAL_H