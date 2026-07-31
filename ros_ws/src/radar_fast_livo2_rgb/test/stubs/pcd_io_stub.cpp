// Stub for pcl::io::savePCDFileBinary — enables test linking on systems
// without PCL installed. Replaced by the real PCL library on target builds.

#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace pcl {
namespace io {

    int savePCDFileBinary(
        const std::string& filename, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
        std::ofstream f(filename, std::ios::binary);
        if (!f.is_open()) return -1;

        f << "VERSION 0.7\n";
        f << "FIELDS x y z rgb\n";
        f << "SIZE 4 4 4 4\n";
        f << "TYPE F F F F\n";
        f << "COUNT 1 1 1 1\n";
        f << "WIDTH " << cloud.size() << "\n";
        f << "HEIGHT 1\n";
        f << "VIEWPOINT 0 0 0 1 0 0 0\n";
        f << "POINTS " << cloud.size() << "\n";
        f << "DATA binary\n";

        for (const auto& pt : cloud) {
            float data[4] = { pt.x, pt.y, pt.z, pt.rgb };
            f.write(reinterpret_cast<const char*>(data), sizeof(data));
        }

        if (f.fail()) return -1;

        f.close();
        return 0;
    }

} // namespace io
} // namespace pcl
