#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_DOUBLE
#include "tiny_obj_loader.h"

#include <unordered_map>
#include <string>
#include <iostream>
#include <filesystem>

bool loadTinyObjToMesh(const std::string& objFilename, mesh& outMesh,
    const std::unordered_map<std::string, std::shared_ptr<material>>& userMaterialMap = {},
    std::shared_ptr<material> defaultMaterial = nullptr)
{
    // 1. Automatically determine the base directory from the objFilename
    std::filesystem::path objPath(objFilename);
    std::string baseDir = objPath.parent_path().string();
    if (baseDir.empty()) baseDir = ".";
    baseDir += "/"; // Ensure trailing slash

    tinyobj::ObjReaderConfig reader_config;
    // Point this to the folder containing the .obj and .mtl
    reader_config.mtl_search_path = baseDir;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(objFilename, reader_config))
    {
        if (!reader.Error().empty()) std::cerr << "TinyObjReader ERROR: " << reader.Error() << "\n";
        return false;
    }

    // This warning usually means the .mtl filename in the .obj doesn't exist in the folder
    if (!reader.Warning().empty()) std::cerr << "TinyObjReader WARNING: " << reader.Warning() << "\n";

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();
    const auto& materials = reader.GetMaterials();

    // ... [Vertex/Normal/UV Map initialization remains the same] ...
    const size_t nPositions = attrib.vertices.size() / 3;
    const size_t nNormals = attrib.normals.size() / 3;
    const size_t nTexcoords = attrib.texcoords.size() / 2;
    std::vector<int> positionMap(nPositions, -1);
    std::vector<int> normalMap(nNormals, -1);
    std::vector<int> uvMap(nTexcoords, -1);

    for (size_t s = 0; s < shapes.size(); ++s)
    {
        const tinyobj::shape_t& shape = shapes[s];
        size_t index_offset = 0;

        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f)
        {
            int fv = shape.mesh.num_face_vertices[f];
            if (fv < 3) { index_offset += fv; continue; }

            // ... [Vertex loop for faceV, faceVT, faceVN remains the same] ...
            std::vector<int> faceV(fv), faceVT(fv), faceVN(fv);
            for (int v = 0; v < fv; ++v) {
                tinyobj::index_t idx = shape.mesh.indices[index_offset + v];
                if (idx.vertex_index >= 0) {
                    if (positionMap[idx.vertex_index] == -1) {
                        point3 p(attrib.vertices[3 * idx.vertex_index + 0], attrib.vertices[3 * idx.vertex_index + 1], attrib.vertices[3 * idx.vertex_index + 2]);
                        positionMap[idx.vertex_index] = outMesh.addVertex(p);
                    }
                    faceV[v] = positionMap[idx.vertex_index];
                }
                if (idx.normal_index >= 0) {
                    if (normalMap[idx.normal_index] == -1) {
                        vec3 n(attrib.normals[3 * idx.normal_index + 0], attrib.normals[3 * idx.normal_index + 1], attrib.normals[3 * idx.normal_index + 2]);
                        normalMap[idx.normal_index] = outMesh.addNormal(unit_vector(n));
                    }
                    faceVN[v] = normalMap[idx.normal_index];
                }
                if (idx.texcoord_index >= 0) {
                    if (uvMap[idx.texcoord_index] == -1) {
                        std::pair<double, double> uv(attrib.texcoords[2 * idx.texcoord_index + 0], attrib.texcoords[2 * idx.texcoord_index + 1]);
                        uvMap[idx.texcoord_index] = outMesh.addUV(uv);
                    }
                    faceVT[v] = uvMap[idx.texcoord_index];
                }
            }

            // --- IMPROVED MATERIAL LOGIC ---
            std::shared_ptr<material> faceMat = defaultMaterial;
            int mat_id = (shape.mesh.material_ids.size() > f) ? shape.mesh.material_ids[f] : -1;

            if (mat_id >= 0 && (size_t)mat_id < materials.size()) {
                const tinyobj::material_t& tmat = materials[mat_id];

                auto it = userMaterialMap.find(tmat.name);
                if (it != userMaterialMap.end()) {
                    faceMat = it->second;
                }
                else {
                    std::shared_ptr<texture> matTexture;

                    if (!tmat.diffuse_texname.empty()) {
                        // Concatenate baseDir with the filename found in .mtl
                        std::string texPath = baseDir + tmat.diffuse_texname;

                        // Debug log to see exactly where it's looking
                        std::cout << "Attempting to load texture: " << texPath << std::endl;

                        matTexture = std::make_shared<image_texture>(texPath.c_str());
                    }
                    else {
                        color kd(tmat.diffuse[0], tmat.diffuse[1], tmat.diffuse[2]);
                        matTexture = std::make_shared<solid_color>(kd);
                    }
                    faceMat = std::make_shared<lambertian>(matTexture);
                }
            }
            // --- END MATERIAL LOGIC ---

            for (int k = 1; k + 1 < fv; ++k) {
                if (faceV[0] < 0 || faceV[k] < 0 || faceV[k + 1] < 0) continue;
                outMesh.addFace(faceV[0], faceV[k], faceV[k + 1], faceVT[0], faceVT[k], faceVT[k + 1], faceVN[0], faceVN[k], faceVN[k + 1], faceMat);
            }
            index_offset += fv;
        }
    }
    outMesh.build(true);
    return true;
}