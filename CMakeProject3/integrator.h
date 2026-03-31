#pragma once

#include "scene.h"
#include "material.h"
#include "rtweekend.h"
#include "bsdf.h"
#include "light.h"
#include <cmath>
#include <memory>

// Helpers ---------------------------------------------------------------

inline double power_heuristic(double pdf_a, double pdf_b) {
    double a = pdf_a * pdf_a;
    double b = pdf_b * pdf_b;
    return (a == 0.0 && b == 0.0) ? 0.0 : a / (a + b);
}

inline double pdfA_to_pdfOmega(double pdfA, double dist2, double cosLight) {
    if (pdfA <= 0.0 || cosLight <= 0.0) return 0.0;
    return pdfA * dist2 / cosLight;
}

static constexpr double SHADOW_EPS = 1e-6;
static constexpr double NEXT_RAY_EPS = 1e-4;

// forward declarations
class MeshLight;
class SphereLight;

// ----------------------------------------------------------------------

// Replace your current path_trace with this improved version
color path_trace(const Scene& scene, const ray& r0, int maxDepth) {
    color L(0.0, 0.0, 0.0);
    color throughput(1.0, 1.0, 1.0);
    ray r = r0;

    const size_t nLights = scene.lights.size();

    // Info to support MIS when a BSDF-sampled ray later hits an emitter.
    double prev_bsdf_pdf = 0.0;
    bool prev_bsdf_was_delta = false;
    point3 prev_shading_point(0, 0, 0);
    size_t prev_light_count_at_spawn = 0;

    for (int depth = 0; depth < maxDepth; ++depth) {
        hit_record rec;
        if (!scene.hit(r, interval(NEXT_RAY_EPS, infinity), rec)) {
            // Miss: environment (black by default). You can set a visible debug environment here.
            //color env = color(0.2, 0.4, 0.8); L += throughput * env;
            break;
        }

        vec3 wo = unit_vector(-r.direction());

        // --- 1) Emission at the hit point ---
        // Start with material emission fallback (covers emissive materials not registered as Light)
        color emitted_here = rec.mat ? rec.mat->emitted(rec) : color(0, 0, 0);

        const Light* hitLight = scene.lookup_light_for_hit(rec);
        if (hitLight) {
            // If it's a MeshLight and we hit a specific prim, prefer the per-prim radiance
            if (rec.prim_id >= 0) {
                if (const MeshLight* ml = dynamic_cast<const MeshLight*>(hitLight)) {
                    emitted_here = ml->radiance_for_prim(rec.prim_id);
                }
                else {
                    // For other lights, use their evaluate() (object-level emission)
                    emitted_here = hitLight->evaluate(rec.p, rec.normal, wo);
                }
            }
            else {
                // Whole-object light (like a sphere) or prim id not present
                emitted_here = hitLight->evaluate(rec.p, rec.normal, wo);
            }
        }
        // else: emitted_here remains from rec.mat->emitted(rec) fallback

        // If there's emission, add it (MIS if ray came from BSDF)
        if (emitted_here.length_squared() > 0.0) {
            if (depth > 0 && !prev_bsdf_was_delta) {
                // We must compute the pdf of this hit under the light-sampling strategy (solid angle)
                double pdf_light_omega = 0.0;

                // direction from the previous shading point (that spawned this ray) -> this hit
                vec3 wi_from_prev = unit_vector(rec.p - prev_shading_point);
                double dist2_prev_to_hit = (rec.p - prev_shading_point).length_squared();
                double cosLight_prev = std::max(0.0, dot(rec.normal, -wi_from_prev));

                if (hitLight) {
                    if (const MeshLight* ml = dynamic_cast<const MeshLight*>(hitLight)) {
                        // Fast per-prim area pdf when available
                        double pdfA = ml->pdf_area_for_prim(rec.prim_id);
                        pdf_light_omega = pdfA_to_pdfOmega(pdfA, dist2_prev_to_hit, cosLight_prev);
                    }
                    else if (const SphereLight* sl = dynamic_cast<const SphereLight*>(hitLight)) {
                        // Reconstruct the sphere sampling PDF as seen from prev_shading_point
                        point3 center = sl->geom->get_center();
                        double radius = sl->geom->get_radius();
                        vec3 wc = center - prev_shading_point;
                        double dist2_center = wc.length_squared();
                        double dist_center = std::sqrt(dist2_center);

                        if (dist_center > radius + 1e-12) {
                            double sinThetaMax2 = (radius * radius) / dist2_center;
                            sinThetaMax2 = std::clamp(sinThetaMax2, 0.0, 1.0);
                            double cosThetaMax = std::sqrt(1.0 - sinThetaMax2);
                            double solidAngle = 2.0 * pi * (1.0 - cosThetaMax);
                            if (solidAngle > 0.0) {
                                double pdfW = 1.0 / solidAngle;
                                // convert pdfW -> pdfA for this hit point
                                vec3 dir = unit_vector(rec.p - prev_shading_point);
                                double cos_at_light = std::max(0.0, dot(-dir, rec.normal));
                                double pdfA = (cos_at_light > 0.0) ? pdfW * (cos_at_light / dist2_prev_to_hit) : 0.0;
                                pdf_light_omega = pdfA_to_pdfOmega(pdfA, dist2_prev_to_hit, cosLight_prev);
                            }
                        }
                    }
                    else {
                        // Generic light fallback
                        double pdfA = hitLight->pdf_area_at_point(rec.p);
                        pdf_light_omega = pdfA_to_pdfOmega(pdfA, dist2_prev_to_hit, cosLight_prev);
                    }
                }
                else {
                    // No registered light: fall back to material-only (we don't have a light-sampling pdf in this case)
                    pdf_light_omega = 0.0;
                }

                // Light strategy included a light selection probability when the ray was spawned.
                double lightPickProb = (prev_light_count_at_spawn > 0) ? (1.0 / double(prev_light_count_at_spawn))
                    : ((nLights > 0) ? 1.0 / double(nLights) : 1.0);
                double pdf_light_strategy = pdf_light_omega * lightPickProb;
                double pdf_bsdf = prev_bsdf_pdf;

                double w = 1.0;
                if (pdf_bsdf > 0.0 || pdf_light_strategy > 0.0)
                    w = power_heuristic(pdf_bsdf, pdf_light_strategy);

                L += throughput * emitted_here * w;
            }
            else {
                // Primary or delta-spawned ray: add emission directly
                L += throughput * emitted_here;
            }
        }

        // --- 2) Direct lighting (NEE) ---
        if (nLights > 0 && rec.mat) {
            size_t lightIndex = (nLights == 1) ? 0 : random_int(0, int(nLights) - 1);
            const auto& lightPtr = scene.lights[lightIndex];
            LightSample ls = lightPtr->sample(rec.p);

            if (ls.pdfA > 0.0) {
                vec3 wi = unit_vector(ls.p - rec.p);
                double dist2 = (ls.p - rec.p).length_squared();
                double dist = std::sqrt(dist2);
                double cosSurface = std::max(0.0, dot(rec.normal, wi));
                double cosLight = std::max(0.0, dot(ls.n, -wi));

                if (cosSurface > 0.0 && cosLight > 0.0) {
                    // shadow ray (offseting along surface normal is fine for shadow test)
                    ray shadowRay(rec.p + rec.normal * SHADOW_EPS, wi);
                    hit_record tmpHit;
                    bool occluded = scene.world.hit(shadowRay, interval(SHADOW_EPS, dist - SHADOW_EPS), tmpHit);

                    if (!occluded) {
                        color f = rec.mat ? rec.mat->evaluate(wo, wi, rec) : color(0, 0, 0);

                        double pdf_light_omega = pdfA_to_pdfOmega(ls.pdfA, dist2, cosLight);
                        double pdf_bsdf = rec.mat ? rec.mat->pdf(wo, wi, rec) : 0.0;

                        double w = 1.0;
                        if (pdf_light_omega > 0.0 || pdf_bsdf > 0.0)
                            w = power_heuristic(pdf_light_omega, pdf_bsdf);

                        double lightPickProb = 1.0 / double(nLights);

                        if (pdf_light_omega > 0.0) {
                            color contrib = throughput * f * ls.Le * (cosSurface / pdf_light_omega) * w * (1.0 / lightPickProb);
                            L += contrib;
                        }
                    }
                }
            }
        }

        // --- 3) Sample BSDF to continue path ---
        BSDFSample bs = rec.mat ? rec.mat->sample(wo, rec) : BSDFSample();

        if (bs.pdf <= 0.0 || bs.f.length_squared() == 0.0) {
            break;
        }

        // store info about this shading point so we can MIS if the spawned ray later hits an emitter
        prev_shading_point = rec.p;
        prev_bsdf_pdf = bs.pdf;           // pdf w.r.t solid angle at prev_shading_point
        prev_bsdf_was_delta = bs.delta;
        prev_light_count_at_spawn = nLights;

        // update throughput
        if (bs.delta) {
            // delta: multiply throughput by bs.f (do not divide by pdf)
            throughput = throughput * bs.f;
        }
        else {
            double cosTheta = std::max(0.0, dot(rec.normal, bs.wi));
            throughput = throughput * bs.f * (cosTheta / bs.pdf);
        }

        // choose EPS small; tune if necessary
        constexpr double BASE_EPS = 1e-5; // try 1e-6..1e-5 if you still see small leakage
        double eps = BASE_EPS;

        vec3 offset_dir;
        if (bs.delta && !bs.is_reflection) {
            // refracted transmission: start the ray slightly *along* the outgoing refracted direction,
            // so it is guaranteed to be inside the medium for the next intersection.
            offset_dir = bs.wi;
        }
        else {
            // reflection or non-delta: offset away from surface along the outward normal
            // choose sign so we step to the outside of the surface
            offset_dir = (dot(bs.wi, rec.normal) > 0.0) ? rec.normal : -rec.normal;
        }

        r = ray(rec.p + offset_dir * eps, bs.wi);



        // Russian roulette
        if (depth >= 5) {
            double p_continue = std::min(0.99, std::max(0.05,
                std::max({ throughput.x(), throughput.y(), throughput.z() })));
            if (random_double() > p_continue) break;
            throughput /= p_continue;
        }
    } // for depth

    return L;
}

