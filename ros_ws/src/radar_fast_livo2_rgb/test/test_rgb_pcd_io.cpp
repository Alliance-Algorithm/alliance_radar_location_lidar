#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "radar_fast_livo2_rgb/rgb_pcd_io.hpp"

namespace fs = std::filesystem;
using namespace radar::fast_livo2::rgb;

namespace {

class RgbPcdIoTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        const auto unique_suffix = std::chrono::steady_clock::now()
                                        .time_since_epoch()
                                        .count();
        temp_dir = fs::temp_directory_path() /
                   ("radar_fast_livo2_rgb_pcd_io_" +
                    std::to_string(unique_suffix));
        ASSERT_TRUE(fs::create_directories(temp_dir));
    }

    void TearDown() override
    {
        std::error_code error;
        fs::remove_all(temp_dir, error);
    }

    fs::path temp_dir;
};

void write_valid_chunk_then_truncate(const fs::path& path)
{
    const std::vector<RgbChunkRecord> input{
        {.x = 1.0F, .y = 2.0F, .z = 3.0F, .rgb = 0x00112233U, .quality = 1.0},
    };
    std::string error;
    ASSERT_TRUE(write_rgb_chunk_transactional(path, input, error)) << error;

    const auto file_size = fs::file_size(path);
    ASSERT_GT(file_size, 28U);
    std::error_code resize_error;
    fs::resize_file(path, file_size - 1U, resize_error);
    ASSERT_FALSE(resize_error) << resize_error.message();
}

std::vector<std::string> read_header_lines(const fs::path& path)
{
    std::ifstream input(path, std::ios::binary);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
        if (line == "DATA binary") {
            break;
        }
    }
    return lines;
}

TEST_F(RgbPcdIoTest, WritesAndReadsIntermediateSchemaWithoutChangingRgbBits)
{
    const fs::path path = temp_dir / "missing" / "chunk.pcd";
    const std::vector<RgbChunkRecord> input{
        {.x = 1.0F, .y = 2.0F, .z = 3.0F, .rgb = 0x00FF0000U, .quality = 0.75},
        {.x = -1.0F, .y = 0.5F, .z = 4.0F, .rgb = 0x0000FF00U, .quality = 1.25},
        {.x = 0.0F, .y = -2.0F, .z = 0.1F, .rgb = 0x000000FFU, .quality = 2.0},
    };
    std::string error;

    ASSERT_TRUE(write_rgb_chunk_transactional(path, input, error)) << error;
    EXPECT_TRUE(fs::is_directory(path.parent_path()));
    EXPECT_TRUE(validate_rgb_chunk_file(path, input.size(), error)) << error;
    EXPECT_EQ(read_header_lines(path),
              (std::vector<std::string>{
                  "# .PCD v0.7 - Point Cloud Data file format",
                  "VERSION 0.7",
                  "FIELDS x y z rgb quality",
                  "SIZE 4 4 4 4 8",
                  "TYPE F F F F F",
                  "COUNT 1 1 1 1 1",
                  "WIDTH 3",
                  "HEIGHT 1",
                  "VIEWPOINT 0 0 0 1 0 0 0",
                  "POINTS 3",
                  "DATA binary",
              }));

    std::vector<RgbChunkRecord> output;
    ASSERT_TRUE(read_rgb_chunk(path, output, error)) << error;
    ASSERT_EQ(output.size(), input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
        EXPECT_FLOAT_EQ(output[i].x, input[i].x);
        EXPECT_FLOAT_EQ(output[i].y, input[i].y);
        EXPECT_FLOAT_EQ(output[i].z, input[i].z);
        EXPECT_EQ(output[i].rgb, input[i].rgb);
        EXPECT_DOUBLE_EQ(output[i].quality, input[i].quality);
    }
}

TEST_F(RgbPcdIoTest, WritesAndReadsFinalSchemaWithoutChangingRgbBits)
{
    const fs::path path = temp_dir / "final.pcd";
    const std::vector<RgbFinalRecord> input{
        {.x = 1.0F, .y = 2.0F, .z = 3.0F, .rgb = 0x00FF0000U},
        {.x = -1.0F, .y = 0.5F, .z = 4.0F, .rgb = 0x0000FF00U},
        {.x = 0.0F, .y = -2.0F, .z = 0.1F, .rgb = 0x000000FFU},
    };
    std::string error;

    ASSERT_TRUE(write_rgb_final_transactional(path, input, error)) << error;
    EXPECT_TRUE(validate_rgb_final_file(path, input.size(), error)) << error;
    EXPECT_EQ(read_header_lines(path),
              (std::vector<std::string>{
                  "# .PCD v0.7 - Point Cloud Data file format",
                  "VERSION 0.7",
                  "FIELDS x y z rgb",
                  "SIZE 4 4 4 4",
                  "TYPE F F F F",
                  "COUNT 1 1 1 1",
                  "WIDTH 3",
                  "HEIGHT 1",
                  "VIEWPOINT 0 0 0 1 0 0 0",
                  "POINTS 3",
                  "DATA binary",
              }));

    std::vector<RgbFinalRecord> output;
    ASSERT_TRUE(read_rgb_final(path, output, error)) << error;
    ASSERT_EQ(output.size(), input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
        EXPECT_FLOAT_EQ(output[i].x, input[i].x);
        EXPECT_FLOAT_EQ(output[i].y, input[i].y);
        EXPECT_FLOAT_EQ(output[i].z, input[i].z);
        EXPECT_EQ(output[i].rgb, input[i].rgb);
    }
}

TEST_F(RgbPcdIoTest, WriteFailureDoesNotPublishFormalFile)
{
    const auto parent_file = temp_dir / "not-a-directory";
    {
        std::ofstream output(parent_file);
        ASSERT_TRUE(output.good());
    }
    const auto formal = parent_file / "chunk.pcd";
    std::string error;
    const std::vector<RgbChunkRecord> input{
        {.x = 1.0F, .y = 2.0F, .z = 3.0F, .rgb = 0x00112233U, .quality = 1.0},
    };

    EXPECT_FALSE(write_rgb_chunk_transactional(formal, input, error));
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(fs::exists(formal));
}

TEST_F(RgbPcdIoTest, TruncatedPayloadIsRejected)
{
    const fs::path path = temp_dir / "truncated.pcd";
    write_valid_chunk_then_truncate(path);

    std::vector<RgbChunkRecord> records;
    std::string error;
    EXPECT_FALSE(read_rgb_chunk(path, records, error));
    EXPECT_FALSE(validate_rgb_chunk_file(path, 1U, error));
}

TEST_F(RgbPcdIoTest, ExtraPayloadIsRejected)
{
    const fs::path path = temp_dir / "extra.pcd";
    const std::vector<RgbFinalRecord> input{
        {.x = 1.0F, .y = 2.0F, .z = 3.0F, .rgb = 0x00112233U},
    };
    std::string error;
    ASSERT_TRUE(write_rgb_final_transactional(path, input, error)) << error;

    std::ofstream output(path, std::ios::binary | std::ios::app);
    ASSERT_TRUE(output.good());
    output.put('\0');
    output.close();

    std::vector<RgbFinalRecord> records;
    EXPECT_FALSE(read_rgb_final(path, records, error));
}

TEST_F(RgbPcdIoTest, RejectsNonFiniteInputRecords)
{
    const fs::path path = temp_dir / "non-finite.pcd";
    const std::vector<RgbChunkRecord> input{
        {.x = 1.0F,
         .y = 2.0F,
         .z = 3.0F,
         .rgb = 0x00112233U,
         .quality = std::numeric_limits<double>::quiet_NaN()},
    };
    std::string error;

    EXPECT_FALSE(write_rgb_chunk_transactional(path, input, error));
    EXPECT_FALSE(fs::exists(path));
}

TEST_F(RgbPcdIoTest, PreservesExistingFormalFileWhenValidationFails)
{
    const fs::path path = temp_dir / "existing.pcd";
    const std::vector<RgbFinalRecord> original{
        {.x = 1.0F, .y = 2.0F, .z = 3.0F, .rgb = 0x00FF0000U},
    };
    std::string error;
    ASSERT_TRUE(write_rgb_final_transactional(path, original, error)) << error;

    std::ifstream before(path, std::ios::binary);
    const std::string original_bytes((std::istreambuf_iterator<char>(before)),
                                     std::istreambuf_iterator<char>());
    before.close();

    const std::vector<RgbFinalRecord> invalid{
        {.x = std::numeric_limits<float>::infinity(),
         .y = 2.0F,
         .z = 3.0F,
         .rgb = 0x0000FF00U},
    };
    EXPECT_FALSE(write_rgb_final_transactional(path, invalid, error));
    EXPECT_FALSE(fs::exists(path.string() + ".tmp"));

    std::ifstream after(path, std::ios::binary);
    const std::string current_bytes((std::istreambuf_iterator<char>(after)),
                                    std::istreambuf_iterator<char>());
    EXPECT_EQ(current_bytes, original_bytes);
}

TEST_F(RgbPcdIoTest, RejectsMismatchedPointCount)
{
    const fs::path path = temp_dir / "mismatched.pcd";
    const std::vector<RgbFinalRecord> input{
        {.x = 1.0F, .y = 2.0F, .z = 3.0F, .rgb = 0x00112233U},
    };
    std::string error;
    ASSERT_TRUE(write_rgb_final_transactional(path, input, error)) << error;

    std::ifstream input_file(path, std::ios::binary);
    std::string contents((std::istreambuf_iterator<char>(input_file)),
                         std::istreambuf_iterator<char>());
    input_file.close();
    const std::string points = "POINTS 1";
    const auto points_position = contents.find(points);
    ASSERT_NE(points_position, std::string::npos);
    contents.replace(points_position, points.size(), "POINTS 2");
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(output.good());
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.close();

    std::vector<RgbFinalRecord> records;
    EXPECT_FALSE(read_rgb_final(path, records, error));
}

}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
