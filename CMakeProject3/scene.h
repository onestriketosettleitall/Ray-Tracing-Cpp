// scene.h
#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <functional>

#include "hittable_list.h" // hittable_list and world.add / world.hit
#include "light.h"         // Light, MeshLight, SphereLight, LightSample
#include "mesh.h"
#include "sphere.h"

// Small helper for colors/points
using color = vec3;
using point3 = vec3;

struct Scene {
    // Geometry & intersection entry point:
    hittable_list world;

    // Registered lights (created at load/scene-setup time)
    std::vector<std::shared_ptr<Light>> lights;

    // Map from (hittable pointer, prim_id) -> light index in `lights`.
    // prim_id == -1 is used for whole-object lights (e.g. analytic sphere).
    struct Key {
        const hittable* obj;
        int prim;
        Key(const hittable* o = nullptr, int p = -1) : obj(o), prim(p) {}
    };

    struct KeyHash {
        std::size_t operator()(Key const& k) const noexcept {
            // combine pointer hash and prim id hash
            auto h1 = std::hash<const void*>()(reinterpret_cast<const void*>(k.obj));
            auto h2 = std::hash<int>()(k.prim);
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
        }
    };
    struct KeyEq {
        bool operator()(Key const& a, Key const& b) const noexcept {
            return a.obj == b.obj && a.prim == b.prim;
        }
    };

    std::unordered_map<Key, int, KeyHash, KeyEq> lightIndex;

    Scene() = default;

    // Add geometry to scene and auto-register lights that come from that object.
    void add_object(const std::shared_ptr<hittable>& obj) {
        world.add(obj);
        register_lights_from(obj);
    }

    // If you want to add a Light manually:
    void add_light(const std::shared_ptr<Light>& L) {
        int idx = int(lights.size());
        lights.push_back(L);

        // If the light wraps geometry, attempt to register mapping from that geometry -> light index
        if (auto geom = L->geometry()) {
            // If geometry is mesh: we don't know which prims belong to this Light unless the Light
            // provides that info; many Light implementations constructed at scene-setup time should
            // fill the map there (see register_lights_from).
            // As fallback, map prim = -1 (whole-object)
            lightIndex[{geom.get(), -1}] = idx;
        }
    }

    // intersection wrapper for integrators (for camera rays and shadow rays)
    bool hit(const ray& r, const interval& t, hit_record& rec) const {
        return world.hit(r, t, rec);
    }

    // Look up the Light* corresponding to a particular hit record (if any).
    // Returns nullptr if that hit is not an emitter registered as a Light.
    const Light* lookup_light_for_hit(const hit_record& rec) const {
        if (!rec.hitObject) return nullptr;
        // Try exact primitive mapping first (triangles)
        Key k(rec.hitObject, rec.prim_id);
        auto it = lightIndex.find(k);
        if (it != lightIndex.end()) return lights[it->second].get();
        // fallback: whole-object mapping
        Key k2(rec.hitObject, -1);
        it = lightIndex.find(k2);
        if (it != lightIndex.end()) return lights[it->second].get();
        return nullptr;
    }

private:
    // Register lights coming from a loaded object.
    void register_lights_from(const std::shared_ptr<hittable>& obj) {
        // ---------- Mesh case: group emissive faces into one MeshLight ---------
        if (auto m = std::dynamic_pointer_cast<mesh>(obj)) {
            std::vector<int> emissiveTriIndices;
            std::vector<color> triRadiance; // per-entry radiance (can be constant or per-tri)

            // collect emissive triangles by inspecting per-face material emission
            for (size_t i = 0; i < m->faces.size(); ++i) {
                if (!m->triValid[i]) continue;
                const Face& F = m->faces[i];
                if (!F.mat) continue;

                // Build a minimal hit_record to query material->emitted
                hit_record dummy;
                dummy.hitObject = m.get();
                dummy.prim_id = int(i);
                dummy.p = m->triCentroid[i];
                dummy.normal = m->triFaceNormal[i];
                dummy.mat = F.mat;
                dummy.has_bary = false;

                color Le = F.mat->emitted(dummy);
                if (Le.length_squared() > 0.0) {
                    emissiveTriIndices.push_back(int(i));
                    triRadiance.push_back(Le);
                }
            }

            if (!emissiveTriIndices.empty()) {
                // create a MeshLight that samples over those triangles proportionally to area
                auto ml = std::make_shared<MeshLight>(m, emissiveTriIndices, triRadiance);
                int idx = int(lights.size());
                lights.push_back(ml);

                // map each triangle to this light index so BSDF-sampled hits can be looked-up
                for (int t : emissiveTriIndices) {
                    lightIndex[{m.get(), t}] = idx;
                }
            }
            return;
        }

        // ---------- Sphere case: analytic sphere light ----------
        if (auto s = std::dynamic_pointer_cast<sphere>(obj)) {
            // Query material emission (create dummy)
            hit_record dummy;
            dummy.hitObject = s.get();
            dummy.prim_id = -1;
            dummy.p = s->get_center();
            dummy.normal = vec3(0, 1, 0);
            dummy.mat = s->get_material();
            color Le = (dummy.mat) ? dummy.mat->emitted(dummy) : color(0, 0, 0);
            if (Le.length_squared() > 0.0) {
                auto sl = std::make_shared<SphereLight>(s, Le);
                int idx = int(lights.size());
                lights.push_back(sl);
                lightIndex[{s.get(), -1}] = idx;
            }
            return;
        }

        // Other shape types: add handling here (e.g. analytic disk, quad, etc.)
    }
};
