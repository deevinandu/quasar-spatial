#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include "src/SpatialPacker.h"
#include "lib/quasar_core/udp_link.h"
#include "lib/quasar_core/quasar_format.h"
#include "lib/quasar_core/huffman.h"

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
    HuffmanCodec librarian;

    // 1. Extract Full Mesh Topology
    std::cout << "Extracting MeshData from: " << model_path << std::endl;
    std::vector<MeshData> components = packer.extractMeshData(model_path.c_str());
    
    if (components.empty()) {
        std::cerr << "No mesh data extracted or error loading model." << std::endl;
        return 1;
    }

    QuasarTx transmitter;
    uint32_t current_target_id = 0;

    for (auto& component : components) {
        std::cout << "\nProcessing Component [" << current_target_id << "]: " << component.name << std::endl;
        
        // --- VERTEX PATH (Signal Logic) ---
        size_t original_vertex_bytes = component.vertices.size() * sizeof(float);
        packer.compressMesh(component.vertices, threshold);
        
        // --- INDEX PATH (Discrete Logic) ---
        // Convert uint32_t indices to byte stream for Huffman
        std::vector<uint8_t> index_bytes(component.indices.size() * sizeof(uint32_t));
        std::memcpy(index_bytes.data(), component.indices.data(), index_bytes.size());
        
        std::cout << "Compressing indices (" << component.indices.size() << ") using Librarian (Huffman)..." << std::endl;
        std::vector<uint8_t> huffman_indices = librarian.compress(index_bytes);

        // --- THE QUASAR BRIDGE ---
        QuasarHeader header = {};
        std::memcpy(header.magic, "QSR1", 4);
        header.file_type = 0x03; // Spatial/Mesh
        header.original_size = original_vertex_bytes + index_bytes.size();
        header.compression_flags = 0x03; // Wavelet (Bit 1) + Huffman (Bit 0)
        header.scale = 1.0f;
        header.target_id = current_target_id++;

        // Final payload: Header + Vertices (Signal) + Indices (Discrete)
        size_t vertex_payload_size = component.vertices.size() * sizeof(float);
        size_t index_payload_size = huffman_indices.size();

        std::vector<uint8_t> packet_data(sizeof(QuasarHeader) + vertex_payload_size + index_payload_size);
        std::memcpy(packet_data.data(), &header, sizeof(QuasarHeader));
        std::memcpy(packet_data.data() + sizeof(QuasarHeader), component.vertices.data(), vertex_payload_size);
        std::memcpy(packet_data.data() + sizeof(QuasarHeader) + vertex_payload_size, huffman_indices.data(), index_payload_size);

        // 4. Transmit Component
        std::cout << "Transmitting component payload: " << packet_data.size() << " bytes." << std::endl;
        transmitter.send_frame(packet_data, target_ip, target_port);
    }

    std::cout << "\nMission complete. All spatial components dispatched." << std::endl;

    return 0;
}
