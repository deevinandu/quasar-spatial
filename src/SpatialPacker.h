#ifndef SPATIAL_PACKER_H
#define SPATIAL_PACKER_H

#include <vector>
#include <string>
#include <iostream>

/**
 * Accessor Stride Aura Check:
 * In cgltf, 'accessor->stride' represents the number of bytes between the start of one element 
 * and the start of the next in the buffer. By using this stride instead of assuming a tightly 
 * packed layout (sizeof(float) * 3), we can handle GLBs where vertex positions are interleaved 
 * with other data (like normals or UVs) in the same buffer view.
 * Implementation: When reading, we offset the data pointer by 'accessor->stride' bytes 
 * for each iteration to jump to the next vertex reliably.
 */

class SpatialPacker {
public:
    SpatialPacker() = default;

    // Loads a GLB file and extracts XYZ vertex positions
    std::vector<float> extractVertices(const char* path);

    // Applies Haar wavelet transform and threshold-based saliency filtering
    void compressMesh(std::vector<float>& vertices, float threshold);
};

#endif // SPATIAL_PACKER_H
