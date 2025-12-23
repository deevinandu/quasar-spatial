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
    std::cout << "Usage:\n";
    std::cout << "  TX: quasar-spatial --model <path> --tx <ip> <port> [--threshold <value>]\n";
    std::cout << "  RX: quasar-spatial --rx <port>\n";
}

int main(int argc, char* argv[]) {
    // Shared parameters
    std::string model_path;
    std::string target_ip;
    int tx_port = 0;
    int rx_port = 0;
    float threshold = 0.01f;
    bool rx_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc) {
            model_path = argv[++i];
        } else if (arg == "--tx" && i + 2 < argc) {
            target_ip = argv[++i];
            tx_port = std::stoi(argv[++i]);
        } else if (arg == "--rx" && i + 1 < argc) {
            rx_port = std::stoi(argv[++i]);
            rx_mode = true;
        } else if (arg == "--threshold" && i + 1 < argc) {
            threshold = std::stof(argv[++i]);
        }
    }

    SpatialPacker packer;
    HuffmanCodec librarian;

    if (rx_mode) {
        std::cout << "Starting Quasar-Spatial GCS Receiver on port " << rx_port << "..." << std::endl;
        QuasarRx receiver;
        std::vector<uint8_t> frame_raw;

        while (true) {
            if (receiver.listen(rx_port, frame_raw)) {
                if (frame_raw.size() < sizeof(QuasarHeader)) continue;

                QuasarHeader* header = reinterpret_cast<QuasarHeader*>(frame_raw.data());
                if (std::string(header->magic, 4) != "QSR1") continue;

                if (header->file_type == 0x03) {
                    std::cout << "\n[Receiver] Incoming Spatial Frame (Target: " << header->target_id << ")" << std::endl;
                    
                    size_t vertex_count = header->width; 
size_t vertex_bytes = vertex_count * sizeof(float);
                    
                    // 1. Separate Payload
                    uint8_t* payload_ptr = frame_raw.data() + sizeof(QuasarHeader);
                    
                    // Extract Vertices
                    if (frame_raw.size() < sizeof(QuasarHeader) + vertex_bytes) continue;
                    std::vector<float> recovered_vertices(vertex_count);
                    std::memcpy(recovered_vertices.data(), payload_ptr, vertex_bytes);
                    
                    // Extract Huffman-compressed Indices
                    size_t huffman_bytes = frame_raw.size() - sizeof(QuasarHeader) - vertex_bytes;
                    std::vector<uint8_t> huffman_indices(huffman_bytes);
                    std::memcpy(huffman_indices.data(), payload_ptr + vertex_bytes, huffman_bytes);

                    // 2. Decompress Indices (Librarian)
                    std::cout << "[Receiver] Decompressing topology..." << std::endl;
                    std::vector<uint8_t> index_raw = librarian.decompress(huffman_indices);
                    std::vector<uint32_t> recovered_indices(index_raw.size() / sizeof(uint32_t));
                    std::memcpy(recovered_indices.data(), index_raw.data(), index_raw.size());

                    // 3. Decompress Vertices (Inverse Wavelet)
                    packer.decompressMesh(recovered_vertices);

                    // 4. Export to OBJ
                    std::string export_name = "recovered_mesh_" + std::to_string(header->target_id) + ".obj";
                    SpatialPacker::saveAsOBJ(export_name, recovered_vertices, recovered_indices);
                }
            }
        }
    } else {
        if (model_path.empty() || target_ip.empty() || tx_port == 0) {
            print_usage();
            return 1;
        }

        std::cout << "Initializing Quasar Spatial TX Pipeline..." << std::endl;

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
            std::vector<uint8_t> index_bytes(component.indices.size() * sizeof(uint32_t));
            std::memcpy(index_bytes.data(), component.indices.data(), index_bytes.size());
            
            std::cout << "Compressing indices (" << component.indices.size() << ") using Librarian (Huffman)..." << std::endl;
            std::vector<uint8_t> huffman_indices = librarian.compress(index_bytes);

            // --- THE QUASAR BRIDGE ---
            QuasarHeader header = {};
            std::memcpy(header.magic, "QSR1", 4);
            header.file_type = 0x03; // Spatial/Mesh
            header.original_size = original_vertex_bytes + index_bytes.size();
            header.compression_flags = 0x03; // Wavelet + Huffman
            header.scale = 1.0f;
            header.target_id = current_target_id++;
            header.width = (uint32_t)component.vertices.size(); // Store vertex count for receiver

            // Final payload: Header + Vertices (Wavelet) + Indices (Huffman)
            size_t vertex_payload_size = component.vertices.size() * sizeof(float);
            size_t index_payload_size = huffman_indices.size();

            std::vector<uint8_t> packet_data(sizeof(QuasarHeader) + vertex_payload_size + index_payload_size);
            std::memcpy(packet_data.data(), &header, sizeof(QuasarHeader));
            std::memcpy(packet_data.data() + sizeof(QuasarHeader), component.vertices.data(), vertex_payload_size);
            std::memcpy(packet_data.data() + sizeof(QuasarHeader) + vertex_payload_size, huffman_indices.data(), index_payload_size);

            // 4. Transmit Component
            std::cout << "Transmitting component payload: " << packet_data.size() << " bytes." << std::endl;
            transmitter.send_frame(packet_data, target_ip, tx_port);
        }

        std::cout << "\nMission complete. All spatial components dispatched." << std::endl;
    }

    return 0;
}
