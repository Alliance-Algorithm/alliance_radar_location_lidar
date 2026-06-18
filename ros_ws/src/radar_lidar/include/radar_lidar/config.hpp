#pragma once

#include <string>

namespace radar::config {

struct LocalizationConfig {
    double voxel_leaf_size   = 0.1;
    double max_corr_distance = 1.0;
    int    max_iterations    = 48;
    double rotation_eps      = 0.001745;
    double translation_eps   = 0.001;
    int    num_threads       = 4;
    bool   verbose           = false;
};

}  // namespace radar::config
