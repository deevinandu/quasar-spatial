#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include "src/SpatialPacker.h"
#include "lib/quasar_core/udp_link.h"
#include "lib/quasar_core/quasar_format.h"

/**
 * Quasar Spatial CLI
 * Spatial compression tool for robotics telemetry.
 */

void print_usage() {
    std::cout << "Usage: quasar-spatial --model <path> --tx <ip> <port> [--threshold <value>]" << std::endl;
}

int main(int argc, char* argv[]) {
    std::string model_path;
    std::string target_ip;
    int target_port = 0;
    float threshold = 0.01f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc) {
            model_path = argv[++i];
        } else if (arg == "--tx" && i + 2 < argc) {
            target_ip = argv[++i];
            target_port = std::stoi(argv[++i]);
        } else if (arg == "--threshold" && i + 1 < argc) {
            threshold = std::stof(argv[++i]);
        }
    }

    if (model_path.empty() || target_ip.empty() || target_port == 0) {
        print_usage();
        return 1;
    }

    std::cout << "Initializing Quasar Spatial Pipeline..." << std::endl;
    SpatialPacker packer;

    // 1. Extract Vertices
    std::cout << "Extracting vertices from: " << model_path << std::endl;
    std::vector<float> vertices = packer.extractVertices(model_path.c_str());
    if (vertices.empty()) {
        std::cerr << "No vertices extracted or error loading model." << std::endl;
        return 1;
    }
    size_t original_bytes = vertices.size() * sizeof(float);
    std::cout << "Extracted " << vertices.size() << " vertex components (" << original_bytes << " bytes)." << std::endl;

    // 2. Compress & Quantize
    std::cout << "Applying Wavelet Transform and 32-bit Quantization..." << std::endl;
    packer.compressMesh(vertices, threshold);

    // Create a temporary GrayImage wrapper to use our existing quantization engine
    // We treat the XYZ components as a 1D signal (width = vertices.size(), height = 1)
    GrayImage spatial_signal(vertices.size(), 1);
    spatial_signal.data = vertices; 

    // Quantize the 3D signal using your high-precision engine
    float spatial_scale = 1000.0f; // High precision for 3D coordinates
    std::vector<uint8_t> quantized_data = quantize(spatial_signal, spatial_scale);

    // Entropy Encode (The Librarian)
    HuffmanCodec codec;
    std::vector<uint8_t> compressed_bitstream = codec.compress(quantized_data);
    
    float ratio = (float)compressed_bitstream.size() / original_bytes * 100.0f;
    std::cout << "Compressed 3D Mesh: " << compressed_bitstream.size() << " bytes (" << ratio << "% of original)" << std::endl;

    // 3. Pack into QuasarHeader
    QuasarHeader header = {};
    std::memcpy(header.magic, "QSR1", 4);
    header.file_type = 0x03; // Spatial/Mesh
    header.original_size = original_bytes;
    header.compression_flags = 0x02 | 0x01; // Wavelet + Huffman
    header.scale = spatial_scale;
    header.width = (uint16_t)vertices.size(); // Store component count
    header.height = 1;

    // Build final packet
    std::vector<uint8_t> packet_data(sizeof(QuasarHeader) + compressed_bitstream.size());
    std::memcpy(packet_data.data(), &header, sizeof(QuasarHeader));
    std::memcpy(packet_data.data() + sizeof(QuasarHeader), compressed_bitstream.data(), compressed_bitstream.size());

    // 4. Transmit
    std::cout << "Transmitting " << packet_data.size() << " bytes to " << target_ip << ":" << target_port << "..." << std::endl;
    QuasarTx transmitter;
    transmitter.send_frame(packet_data, target_ip, target_port);

    std::cout << "Mission complete. Spatial data dispatched." << std::endl;

    return 0;
}
