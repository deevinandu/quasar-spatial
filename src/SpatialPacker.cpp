#define CGLTF_IMPLEMENTATION
#include "../include/third_party/cgltf.h"
#include "SpatialPacker.h"
#include "../lib/quasar_core/wavelet.h"
#include <vector>
#include <iostream>

std::vector<float> SpatialPacker::extractVertices(const char* path) {
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

    std::vector<float> allVertices;

    for (cgltf_size i = 0; i < data->meshes_count; ++i) {
        for (cgltf_size j = 0; j < data->meshes[i].primitives_count; ++j) {
            cgltf_primitive& prim = data->meshes[i].primitives[j];
            
            for (cgltf_size k = 0; k < prim.attributes_count; ++k) {
                if (prim.attributes[k].type == cgltf_attribute_type_position) {
                    cgltf_accessor* accessor = prim.attributes[k].data;
                    cgltf_size num_components = cgltf_num_components(accessor->type);
                    
                    // Extract all elements (X, Y, Z usually)
                    for (cgltf_size v = 0; v < accessor->count; ++v) {
                        float v_data[16]; // Max components for cgltf_accessor_read_float
                        if (cgltf_accessor_read_float(accessor, v, v_data, 16)) {
                            for (cgltf_size c = 0; c < num_components; ++c) {
                                allVertices.push_back(v_data[c]);
                            }
                        }
                    }
                }
            }
        }
    }

    cgltf_free(data);
    return allVertices;
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
