#ifndef MESH_H
#define MESH_H

#include "matrix.h"
#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>
#include <numeric>
#include <cassert>

struct Face
{
    int v[3];
    int vt[3];
    int vn[3];
    std::shared_ptr<material> mat;

    Face()
    {
        for (int i = 0; i < 3; ++i) v[i] = vt[i] = vn[i] = -1;
        mat = nullptr;
    }
};

class mesh : public hittable
{
public:
    std::vector<point3> positions;
    std::vector<vec3> normals;
    std::vector<std::pair<double, double>> uvs;
    std::vector<Face> faces;
    std::shared_ptr<material> mat;

    point3 aabb_min{ +infinity, +infinity, +infinity };
    point3 aabb_max{ -infinity, -infinity, -infinity };

    // triangle precompute
    std::vector<vec3> triEdge1;
    std::vector<vec3> triEdge2;
    std::vector<vec3> triFaceNormal;
    std::vector<double> triArea;
    std::vector<char> triValid;
    std::vector<point3> triCentroid;

    std::shared_ptr<material> get_material() const { return mat; }

    // BVH
    struct BVHNode {
        point3 aabbMin, aabbMax;
        uint32_t leftNode;      // left child index (right = left + 1)
        uint32_t firstTriIdx;   // start index in triIdx
        uint32_t triCount;      // >0 => leaf
        BVHNode() : aabbMin(+infinity, +infinity, +infinity), aabbMax(-infinity, -infinity, -infinity),
            leftNode(0), firstTriIdx(0), triCount(0) {
        }
        bool isLeaf() const { return triCount > 0; }
    };

    std::vector<BVHNode> bvhNodes;
    std::vector<uint32_t> triIdx; // permutation / indirection for triangles

    // knobs
    static constexpr double EPS_PARALLEL = 1e-12;
    static constexpr double EPS_AREA = 1e-12;
    static constexpr double RAY_EPS = 1e-3;
    static constexpr uint32_t MAX_LEAF_TRIANGLES = 8; // tune: 4..16 usually good

    // ctors
    mesh(const std::vector<point3>& verts,
        const std::vector<Face>& faces_,
        std::shared_ptr<material> m)
        : positions(verts), faces(faces_), mat(m)
    {
    }
    mesh() {}

    void reserve(size_t nv, size_t nuv, size_t nn, size_t nf)
    {
        positions.reserve(nv);
        uvs.reserve(nuv);
        normals.reserve(nn);
        faces.reserve(nf);
    }

    void applyTransform(const Mat4& M)
    {
        for (auto& v : positions) v = transformVec3(v, M, 1.0);
        for (auto& n : normals) n = transformNormal(n, M);
    }

    int addVertex(const point3& p) { positions.push_back(p); expandBounds(p); return int(positions.size() - 1); }
    int addNormal(const vec3& n) { normals.push_back(n); return int(normals.size() - 1); }
    int addUV(const std::pair<double, double>& t) { uvs.push_back(t); return int(uvs.size() - 1); }

    void addFace(int v0, int v1, int v2,
        int vt0 = -1, int vt1 = -1, int vt2 = -1,
        int vn0 = -1, int vn1 = -1, int vn2 = -1,
        std::shared_ptr<material> mat_ = nullptr)
    {
        Face f;
        f.v[0] = v0; f.v[1] = v1; f.v[2] = v2;
        f.vt[0] = vt0; f.vt[1] = vt1; f.vt[2] = vt2;
        f.vn[0] = vn0; f.vn[1] = vn1; f.vn[2] = vn2;
        f.mat = mat_;
        faces.push_back(f);
    }

    // build pipeline
    void build(bool computeVertexNormalsIfMissing)
    {
        computeBounds();
        precomputeTriangles();
        if (computeVertexNormalsIfMissing) computeVertexNormals();
        buildBVH();
    }

    // hit: uses BVH if present; fallback to brute force otherwise
    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
        if (faces.empty()) return false;

        if (bvhNodes.empty())
        {
            // brute-force fallback
            if (!rayIntersectsAABB(r, ray_t)) return false;
            bool hitAnything = false;
            double closest = ray_t.max;
            for (size_t i = 0; i < faces.size(); ++i)
            {
                if (!triValid[i]) continue;
                double t, u, v;
                if (rayTriangleIntersectM(positions[faces[i].v[0]], triEdge1[i], triEdge2[i], r, ray_t, t, u, v))
                {
                    if (t < closest)
                    {
                        closest = t;
                        const_cast<mesh*>(this)->fillHitRecordFromTriangle(i, t, u, v, r, rec);
                        hitAnything = true;
                    }
                }
            }
            return hitAnything;
        }
        else
        {
            if (!rayIntersectsAABB(r, ray_t)) return false;

            bool hitAnything = false;
            double closest = ray_t.max;

            // stack traversal (iterative)
            std::vector<uint32_t> stack;
            stack.reserve(64);
            stack.push_back(0); // root

            while (!stack.empty())
            {
                uint32_t nodeIdx = stack.back();
                stack.pop_back();

                // safe guard
                if (nodeIdx >= bvhNodes.size()) continue;
                const BVHNode& node = bvhNodes[nodeIdx];

                if (!intersectAABBNode(r, ray_t, node.aabbMin, node.aabbMax, closest)) continue;

                if (node.isLeaf())
                {
                    // safety assert
                    assert(node.firstTriIdx + node.triCount <= triIdx.size());
                    for (uint32_t i = 0; i < node.triCount; ++i)
                    {
                        uint32_t triIndex = triIdx[node.firstTriIdx + i];
                        if (!triValid[triIndex]) continue;
                        double t, u, v;
                        if (rayTriangleIntersectM(positions[faces[triIndex].v[0]], triEdge1[triIndex], triEdge2[triIndex], r, ray_t, t, u, v))
                        {
                            if (t < closest)
                            {
                                closest = t;
                                const_cast<mesh*>(this)->fillHitRecordFromTriangle(triIndex, t, u, v, r, rec);
                                hitAnything = true;
                            }
                        }
                    }
                }
                else
                {
                    // children are left and left+1
                    // push in either order (no near-first ordering here)
                    stack.push_back(node.leftNode);
                    stack.push_back(node.leftNode + 1);
                }
            }

            return hitAnything;
        }
    }

private:
    // --- ray-triangle (unchanged) ---
    static bool rayTriangleIntersectM(
        const point3& v0, const vec3& e1, const vec3& e2,
        const ray& r,
        const interval& ray_t,
        double& outT, double& outU, double& outV,
        bool backface_cull = true)
    {
        const double EPS_PARALLEL = 1e-12;
        const double EPS_BARY = 1e-10;
        const double EPS_T = 1e-8;

        vec3 pvec = cross(r.direction(), e2);
        double det = dot(e1, pvec);

        if (backface_cull) {
            if (det < EPS_PARALLEL) return false;
        }
        else {
            if (fabs(det) < EPS_PARALLEL) return false;
        }

        double invDet = 1.0 / det;

        vec3 tvec = r.origin() - v0;
        double u = dot(tvec, pvec) * invDet;
        if (u < -EPS_BARY || u > 1.0 + EPS_BARY) return false;

        vec3 qvec = cross(tvec, e1);
        double v = dot(r.direction(), qvec) * invDet;
        if (v < -EPS_BARY || (u + v) > 1.0 + EPS_BARY) return false;

        double t = dot(e2, qvec) * invDet;

        double tmin_allowed = ray_t.min + EPS_T;
        if (t < tmin_allowed || t > ray_t.max - EPS_T) return false;

        outT = t; outU = u; outV = v;
        return true;
    }

    // --- fixed & safer fillHitRecordFromTriangle (bugfix here) ---
    void fillHitRecordFromTriangle(size_t triIndex, double t, double u, double v, const ray& r, hit_record& rec) const
    {
        const Face& f = faces[triIndex];

        vec3 faceN = triFaceNormal[triIndex];
        rec.set_face_normal(r, faceN);

        // per-corner normals?
        bool havePerCornerNormals =
            (f.vn[0] != -1 && f.vn[1] != -1 && f.vn[2] != -1) &&
            (size_t)f.vn[0] < normals.size() &&
            (size_t)f.vn[1] < normals.size() &&
            (size_t)f.vn[2] < normals.size();

        if (havePerCornerNormals)
        {
            vec3 n0 = normals[f.vn[0]];
            vec3 n1 = normals[f.vn[1]];
            vec3 n2 = normals[f.vn[2]];
            //rec.normal = unit_vector(((1.0 - u - v) * n0 + u * n1 + v * n2));
        }
        else
        {
            // per-vertex normals aligned with positions?
            bool havePerVertexNormals =
                (normals.size() >= positions.size()) &&
                (f.v[0] != -1 && f.v[1] != -1 && f.v[2] != -1) &&
                (size_t)f.v[0] < normals.size() &&
                (size_t)f.v[1] < normals.size() &&
                (size_t)f.v[2] < normals.size();

            if (havePerVertexNormals)
            {
                vec3 n0 = normals[f.v[0]];
                vec3 n1 = normals[f.v[1]];
                vec3 n2 = normals[f.v[2]];
                //rec.normal = unit_vector(((1.0 - u - v) * n0 + u * n1 + v * n2));
            }
            else
            {
                rec.normal = faceN;
            }
        }

        // UVs
        bool havePerCornerUVs =
            (f.vt[0] >= 0 && f.vt[1] >= 0 && f.vt[2] >= 0) &&
            (size_t)f.vt[0] < uvs.size() &&
            (size_t)f.vt[1] < uvs.size() &&
            (size_t)f.vt[2] < uvs.size();

        if (havePerCornerUVs)
        {
            auto uv0 = uvs[f.vt[0]];
            auto uv1 = uvs[f.vt[1]];
            auto uv2 = uvs[f.vt[2]];
            rec.u = (1.0 - u - v) * uv0.first + u * uv1.first + v * uv2.first;
            rec.v = (1.0 - u - v) * uv0.second + u * uv1.second + v * uv2.second;
        }
        else
        {
            bool havePerVertexUVs =
                (uvs.size() >= positions.size()) &&
                (f.v[0] >= 0 && f.v[1] >= 0 && f.v[2] >= 0) &&
                (size_t)f.v[0] < uvs.size() &&
                (size_t)f.v[1] < uvs.size() &&
                (size_t)f.v[2] < uvs.size();

            if (havePerVertexUVs)
            {
                auto uv0 = uvs[f.v[0]];
                auto uv1 = uvs[f.v[1]];
                auto uv2 = uvs[f.v[2]];
                rec.u = (1.0 - u - v) * uv0.first + u * uv1.first + v * uv2.first;
                rec.v = (1.0 - u - v) * uv0.second + u * uv1.second + v * uv2.second;
            }
            else
            {
                rec.u = u; rec.v = v;
            }
        }

        rec.t = t;
        rec.p = r.at(t);
        rec.mat = f.mat;
        rec.hitObject = this;
        rec.prim_id = (int)triIndex;
        rec.has_bary = true; rec.bary_u = u; rec.bary_v = v;
    }

    // expand / compute bounds
    void expandBounds(const point3& p)
    {
        aabb_min = point3(std::min(aabb_min.x(), p.x()),
            std::min(aabb_min.y(), p.y()),
            std::min(aabb_min.z(), p.z()));
        aabb_max = point3(std::max(aabb_max.x(), p.x()),
            std::max(aabb_max.y(), p.y()),
            std::max(aabb_max.z(), p.z()));
    }

    void computeBounds()
    {
        if (positions.empty())
        {
            aabb_min = point3(0, 0, 0);
            aabb_max = point3(0, 0, 0);
            return;
        }
        aabb_min = aabb_max = positions[0];
        for (const auto& p : positions) expandBounds(p);
    }

    // precompute triangles
    void precomputeTriangles()
    {
        size_t n = faces.size();
        triEdge1.assign(n, vec3(0, 0, 0));
        triEdge2.assign(n, vec3(0, 0, 0));
        triFaceNormal.assign(n, vec3(0, 0, 0));
        triArea.assign(n, 0.0);
        triValid.assign(n, 0);
        triCentroid.assign(n, point3(0, 0, 0));

        for (size_t i = 0; i < n; ++i)
        {
            const Face& f = faces[i];
            if (!indexValid(f.v[0], positions) || !indexValid(f.v[1], positions) || !indexValid(f.v[2], positions))
            {
                triValid[i] = 0;
                continue;
            }
            const point3& p0 = positions[f.v[0]];
            const point3& p1 = positions[f.v[1]];
            const point3& p2 = positions[f.v[2]];
            vec3 e1 = p1 - p0;
            vec3 e2 = p2 - p0;
            vec3 c = cross(e1, e2);
            double area = 0.5 * c.length();
            triCentroid[i] = point3(
                (p0.x() + p1.x() + p2.x()) / 3.0,
                (p0.y() + p1.y() + p2.y()) / 3.0,
                (p0.z() + p1.z() + p2.z()) / 3.0);
            if (area <= EPS_AREA)
            {
                triValid[i] = 0;
                triArea[i] = 0.0;
            }
            else
            {
                triValid[i] = 1;
                triEdge1[i] = e1;
                triEdge2[i] = e2;
                triFaceNormal[i] = unit_vector(c);
                triArea[i] = area;
            }
        }
    }

    void computeVertexNormals()
    {
        normals.assign(positions.size(), vec3(0, 0, 0));
        for (size_t i = 0; i < faces.size(); ++i)
        {
            if (!triValid[i]) continue;
            const Face& f = faces[i];
            const point3& p0 = positions[f.v[0]];
            const point3& p1 = positions[f.v[1]];
            const point3& p2 = positions[f.v[2]];
            vec3 fn = cross(p1 - p0, p2 - p0);
            normals[f.v[0]] += fn;
            normals[f.v[1]] += fn;
            normals[f.v[2]] += fn;
        }
        for (auto& n : normals) n = unit_vector(n);
    }

    template <typename T>
    bool indexValid(int idx, const std::vector<T>& V) const
    {
        return idx >= 0 && (size_t)idx < V.size();
    }

    bool rayIntersectsAABB(const ray& r, interval ray_t) const
    {
        if (positions.empty()) return false;

        double tmin = ray_t.min, tmax = ray_t.max;
        // slab test X
        {
            double invD = 1.0 / r.direction().x();
            double t0 = (aabb_min.x() - r.origin().x()) * invD;
            double t1 = (aabb_max.x() - r.origin().x()) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            tmin = std::max(tmin, t0);
            tmax = std::min(tmax, t1);
            if (tmax <= tmin) return false;
        }
        // Y
        {
            double invD = 1.0 / r.direction().y();
            double t0 = (aabb_min.y() - r.origin().y()) * invD;
            double t1 = (aabb_max.y() - r.origin().y()) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            tmin = std::max(tmin, t0);
            tmax = std::min(tmax, t1);
            if (tmax <= tmin) return false;
        }
        // Z
        {
            double invD = 1.0 / r.direction().z();
            double t0 = (aabb_min.z() - r.origin().z()) * invD;
            double t1 = (aabb_max.z() - r.origin().z()) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            tmin = std::max(tmin, t0);
            tmax = std::min(tmax, t1);
            if (tmax <= tmin) return false;
        }
        return true;
    }

    // ---------------- BVH build (median-of-centroids) ----------------

    void updateNodeBounds(uint32_t nodeIdx)
    {
        assert(nodeIdx < bvhNodes.size());
        BVHNode& node = bvhNodes[nodeIdx];
        node.aabbMin = point3(+infinity, +infinity, +infinity);
        node.aabbMax = point3(-infinity, -infinity, -infinity);

        size_t first = node.firstTriIdx;
        size_t count = node.triCount;
        assert(first + count <= triIdx.size());
        for (size_t i = 0; i < count; ++i)
        {
            uint32_t triId = triIdx[first + i];
            const Face& f = faces[triId];
            const point3& p0 = positions[f.v[0]];
            const point3& p1 = positions[f.v[1]];
            const point3& p2 = positions[f.v[2]];

            // expand min/max
            node.aabbMin = point3(
                std::min(node.aabbMin.x(), std::min(p0.x(), std::min(p1.x(), p2.x()))),
                std::min(node.aabbMin.y(), std::min(p0.y(), std::min(p1.y(), p2.y()))),
                std::min(node.aabbMin.z(), std::min(p0.z(), std::min(p1.z(), p2.z())))
            );
            node.aabbMax = point3(
                std::max(node.aabbMax.x(), std::max(p0.x(), std::max(p1.x(), p2.x()))),
                std::max(node.aabbMax.y(), std::max(p0.y(), std::max(p1.y(), p2.y()))),
                std::max(node.aabbMax.z(), std::max(p0.z(), std::max(p1.z(), p2.z())))
            );
        }
    }

    void buildBVH()
    {
        size_t N = faces.size();
        triIdx.resize(N);
        for (size_t i = 0; i < N; ++i)
            triIdx[i] = uint32_t(i);

        bvhNodes.clear();
        if (N == 0) return;

        bvhNodes.reserve(N * 2);

        // root node covers all triangles
        bvhNodes.emplace_back();
        bvhNodes[0].firstTriIdx = 0;
        bvhNodes[0].triCount = (uint32_t)N;
        updateNodeBounds(0);

        subdivideNode(0);
    }

    void subdivideNode(uint32_t nodeIdx)
    {
        BVHNode& node = bvhNodes[nodeIdx];
        size_t first = node.firstTriIdx;
        size_t count = node.triCount;

        // stop if leaf is small enough
        static constexpr size_t MAX_LEAF_TRIANGLES = 8;
        if (count <= MAX_LEAF_TRIANGLES)
            return;

        // compute extent
        point3 extent = node.aabbMax - node.aabbMin;
        int axis = 0;
        if (extent.y() > extent.x()) axis = 1;
        if (extent.z() > extent[axis]) axis = 2;

        // split around median centroid
        size_t mid = first + count / 2;
        auto midIt = triIdx.begin() + mid;
        std::nth_element(
            triIdx.begin() + first,
            midIt,
            triIdx.begin() + first + count,
            [&](uint32_t a, uint32_t b) {
                double ca = (axis == 0) ? triCentroid[a].x() :
                    (axis == 1) ? triCentroid[a].y() : triCentroid[a].z();
                double cb = (axis == 0) ? triCentroid[b].x() :
                    (axis == 1) ? triCentroid[b].y() : triCentroid[b].z();
                return ca < cb;
            });

        size_t leftCount = mid - first;
        size_t rightCount = count - leftCount;

        // if split failed (all centroids identical), stop here
        if (leftCount == 0 || rightCount == 0)
            return;

        // create left child
        uint32_t leftChildIdx = (uint32_t)bvhNodes.size();
        bvhNodes.emplace_back();
        bvhNodes[leftChildIdx].firstTriIdx = (uint32_t)first;
        bvhNodes[leftChildIdx].triCount = (uint32_t)leftCount;
        updateNodeBounds(leftChildIdx);

        // create right child
        uint32_t rightChildIdx = (uint32_t)bvhNodes.size();
        bvhNodes.emplace_back();
        bvhNodes[rightChildIdx].firstTriIdx = (uint32_t)(first + leftCount);
        bvhNodes[rightChildIdx].triCount = (uint32_t)rightCount;
        updateNodeBounds(rightChildIdx);

        // mark parent as interior
        node.leftNode = leftChildIdx;
        node.triCount = 0;

        // recurse
        subdivideNode(leftChildIdx);
        subdivideNode(rightChildIdx);
    }
    // AABB test for node pruning: compare against current closest found (tMax)
    static bool intersectAABBNode(const ray& r, const interval& ray_t, const point3& bmin, const point3& bmax, double tMax)
    {
        double tmin = ray_t.min, tmax = tMax;

        // X
        double invD = 1.0 / r.direction().x();
        double t0 = (bmin.x() - r.origin().x()) * invD;
        double t1 = (bmax.x() - r.origin().x()) * invD;
        if (invD < 0.0) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax <= tmin) return false;

        // Y
        invD = 1.0 / r.direction().y();
        t0 = (bmin.y() - r.origin().y()) * invD;
        t1 = (bmax.y() - r.origin().y()) * invD;
        if (invD < 0.0) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax <= tmin) return false;

        // Z
        invD = 1.0 / r.direction().z();
        t0 = (bmin.z() - r.origin().z()) * invD;
        t1 = (bmax.z() - r.origin().z()) * invD;
        if (invD < 0.0) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax <= tmin) return false;

        return true;
    }

}; // end class mesh

#endif // MESH_H
