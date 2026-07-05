#include "radar_calibration/camera_lidar_calibration.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>

namespace {

auto print_usage(const char* prog) -> void {
    std::fprintf(stderr,
        "Usage: %s inject-initial-guess <calib.json> <initial_guess.yaml>\n"
        "       %s extract-result <calib.json> <output_extrinsic.yaml>\n",
        prog, prog);
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        print_usage(argv[0]);
        return 1;
    }

    const std::string command { argv[1] };
    const std::filesystem::path arg1 { argv[2] };
    const std::filesystem::path arg2 { argv[3] };

    if (command == "inject-initial-guess") {
        auto result = radar::calibration::inject_initial_guess(arg1, arg2);
        if (!result) {
            std::fprintf(stderr, "Failed to inject initial guess: %s\n", result.error().c_str());
            return 1;
        }
        std::printf("Injected initial guess into %s\n", arg1.string().c_str());
        return 0;
    }

    if (command == "extract-result") {
        auto t_map_camera = radar::calibration::load_t_map_camera(arg1);
        if (!t_map_camera) {
            std::fprintf(
                stderr, "Failed to load calibration result: %s\n", t_map_camera.error().c_str());
            return 1;
        }
        auto write_result = radar::calibration::write_extrinsic_yaml(arg2, *t_map_camera);
        if (!write_result) {
            std::fprintf(
                stderr, "Failed to write extrinsic YAML: %s\n", write_result.error().c_str());
            return 1;
        }
        std::printf("Wrote t_map_camera to %s\n", arg2.string().c_str());
        return 0;
    }

    print_usage(argv[0]);
    return 1;
}
