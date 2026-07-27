#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace radar::fast_livo2::rgb {

struct RgbChunkRecord {
    float x;
    float y;
    float z;
    std::uint32_t rgb;
    double quality;
};

struct RgbFinalRecord {
    float x;
    float y;
    float z;
    std::uint32_t rgb;
};

bool write_rgb_chunk_transactional(
    const std::filesystem::path& path,
    std::span<const RgbChunkRecord> records,
    std::string& error);

bool write_rgb_final_transactional(
    const std::filesystem::path& path,
    std::span<const RgbFinalRecord> records,
    std::string& error);

bool read_rgb_chunk(
    const std::filesystem::path& path,
    std::vector<RgbChunkRecord>& records,
    std::string& error);

bool read_rgb_final(
    const std::filesystem::path& path,
    std::vector<RgbFinalRecord>& records,
    std::string& error);

bool validate_rgb_chunk_file(
    const std::filesystem::path& path,
    std::size_t expected_count,
    std::string& error);

bool validate_rgb_final_file(
    const std::filesystem::path& path,
    std::size_t expected_count,
    std::string& error);

}  // namespace radar::fast_livo2::rgb
