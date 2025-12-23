#ifndef SPATIAL_PACKER_H
#define SPATIAL_PACKER_H

#include <vector>
#include <string>
#include <iostream>
#include <cstdint>

/**
 * Accessor Stride Aura Check:
 * In cgltf, 'accessor->stride' represents the number of bytes between the start of one element 
 * and the start of the next in the buffer. By using this stride instead of assuming a tightly 
 * packed layout (sizeof(float) * 3), we can handle GLBs where vertex positions are interleaved 
 * with other data (like normals or UVs) in the same buffer view.
 * Implementation: When reading, we offset the data pointer by 'accessor->stride' bytes 
 * for each iteration to jump to the next vertex reliably.
 */

struct MeshData {
    std::vector<float> vertices;
    std::vector<uint32_t> indices;
    std::string name;
};

/**
 * Accessor Stride & Index Type Aura Check:
 * - Accessor Stride: Handles interleaved data layouts by jumping 'stride' bytes per element.
 * - uint32_t Indices: We promote all indices to 32-bit to ensure internal pipeline uniformity 
 *   and safety. Large Martian terrains or robotic models can easily exceed the 64k limit 
 *   of uint16_t; using uint32_t prevents parity errors and overflow during decompression.
 */

class SpatialPacker {
public:
    SpatialPacker() = default;

    // Loads a GLB file and extracts full MeshData for all components
    std::vector<MeshData> extractMeshData(const char* path);

    // Applies Haar wavelet transform and threshold-based saliency filtering (Vertices only)
    void compressMesh(std::vector<float>& vertices, float threshold);
};

#endif // SPATIAL_PACKER_H
