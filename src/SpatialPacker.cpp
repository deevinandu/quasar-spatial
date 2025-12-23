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

    // Apply Haar 1D transform from wavelet.h
    // Note: haar1D usually expects a power-of-2 size or handles it internally.
    // We'll pass the size.
    haar1D(vertices, (int)vertices.size());

    // Geometry Saliency Filter
    for (float& val : vertices) {
        if (std::abs(val) < threshold) {
            val = 0.0f;
        }
    }
    
    std::cout << "Spatial compression applied. Threshold: " << threshold << std::endl;
}
