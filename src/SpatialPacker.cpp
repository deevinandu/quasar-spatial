#define CGLTF_IMPLEMENTATION
#include "../include/third_party/cgltf.h"
#include "SpatialPacker.h"
#include "../lib/quasar_core/wavelet.h"
#include <vector>
#include <iostream>

std::vector<MeshData> SpatialPacker::extractMeshData(const char* path) {
    cgltf_options options = {};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, path, &data);

    if (result != cgltf_result_success) {
        std::cerr << "Failed to parse GLB: " << path << std::endl;
        return {};
    }

    result = cgltf_load_buffers(&options, data, path);
    if (result != cgltf_result_success) {
        std::cerr << "Failed to load buffers for: " << path << std::endl;
        cgltf_free(data);
        return {};
    }

    std::vector<MeshData> sceneData;

    // Traverse nodes to associate names with meshes
    for (cgltf_size i = 0; i < data->nodes_count; ++i) {
        cgltf_node* node = &data->nodes[i];
        if (!node->mesh) continue;

        MeshData meshData;
        meshData.name = node->name ? node->name : "unnamed_component";

        for (cgltf_size j = 0; j < node->mesh->primitives_count; ++j) {
            cgltf_primitive& prim = node->mesh->primitives[j];

            // 1. Extract Vertices (Position)
            for (cgltf_size k = 0; k < prim.attributes_count; ++k) {
                if (prim.attributes[k].type == cgltf_attribute_type_position) {
                    cgltf_accessor* accessor = prim.attributes[k].data;
                    cgltf_size num_components = cgltf_num_components(accessor->type);

                    for (cgltf_size v = 0; v < accessor->count; ++v) {
                        float v_data[16];
                        if (cgltf_accessor_read_float(accessor, v, v_data, 16)) {
                            for (cgltf_size c = 0; c < num_components; ++c) {
                                meshData.vertices.push_back(v_data[c]);
                            }
                        }
                    }
                }
            }

            // 2. Extract Indices
            if (prim.indices) {
                cgltf_accessor* indexAccessor = prim.indices;
                for (cgltf_size v = 0; v < indexAccessor->count; ++v) {
                    meshData.indices.push_back((uint32_t)cgltf_accessor_read_index(indexAccessor, v));
                }
            }
        }
        sceneData.push_back(std::move(meshData));
    }

    cgltf_free(data);
    return sceneData;
}

void SpatialPacker::compressMesh(std::vector<float>& vertices, float threshold) {
    if (vertices.empty()) return;

    size_t num_vertices = vertices.size() / 3;
    std::vector<float> X(num_vertices), Y(num_vertices), Z(num_vertices);

    // 1. De-interleave (Spatial -> Planar)
    for (size_t i = 0; i < num_vertices; ++i) {
        X[i] = vertices[i * 3 + 0];
        Y[i] = vertices[i * 3 + 1];
        Z[i] = vertices[i * 3 + 2];
    }

    // 2. Apply Wavelet Transform to each plane independently
    // This prevents the X coordinate of one point from "bleeding" into the Y of another
    haar1D(X, (int)X.size());
    haar1D(Y, (int)Y.size());
    haar1D(Z, (int)Z.size());

    // 3. Apply Geometry Saliency
    auto filter = [&](std::vector<float>& plane) {
        for (float& val : plane) {
            if (std::abs(val) < threshold) val = 0.0f;
        }
    };
    filter(X); filter(Y); filter(Z);

    // 4. Re-interleave back to main vector
    for (size_t i = 0; i < num_vertices; ++i) {
        vertices[i * 3 + 0] = X[i];
        vertices[i * 3 + 1] = Y[i];
        vertices[i * 3 + 2] = Z[i];
    }
    
    std::cout << "[Spatial] Planar Wavelet compression complete." << std::endl;
}

void SpatialPacker::decompressMesh(std::vector<float>& vertices) {
    if (vertices.empty()) return;

    size_t num_vertices = vertices.size() / 3;
    std::vector<float> X(num_vertices), Y(num_vertices), Z(num_vertices);

    // 1. De-interleave
    for (size_t i = 0; i < num_vertices; ++i) {
        X[i] = vertices[i * 3 + 0];
        Y[i] = vertices[i * 3 + 1];
        Z[i] = vertices[i * 3 + 2];
    }

    // 2. Inverse Haar on each plane
    invHaar1D(X, (int)X.size());
    invHaar1D(Y, (int)Y.size());
    invHaar1D(Z, (int)Z.size());

    // 3. Re-interleave
    for (size_t i = 0; i < num_vertices; ++i) {
        vertices[i * 3 + 0] = X[i];
        vertices[i * 3 + 1] = Y[i];
        vertices[i * 3 + 2] = Z[i];
    }
    std::cout << "[Receiver] Inverse Planar Haar completed." << std::endl;
}

#include <fstream>
void SpatialPacker::saveAsOBJ(const std::string& path, const std::vector<float>& vertices, const std::vector<uint32_t>& indices) {
    std::cout << "[Debug] Saving OBJ with " << vertices.size()/3 << " vertices and " << indices.size()/3 << " faces." << std::endl;
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for OBJ export: " << path << std::endl;
        return;
    }

    file << "# Quasar-Spatial Recovered Model\n";
    
    // Write vertices
    for (size_t i = 0; i < vertices.size(); i += 3) {
        file << "v " << vertices[i] << " " << vertices[i+1] << " " << vertices[i+2] << "\n";
    }

    // Write faces (OBJ indices are 1-based)
    for (size_t i = 0; i < indices.size(); i += 3) {
        file << "f " << (indices[i] + 1) << " " << (indices[i+1] + 1) << " " << (indices[i+2] + 1) << "\n";
    }

    file.close();
    std::cout << "[Spatial] Exported to OBJ: " << path << std::endl;
}
