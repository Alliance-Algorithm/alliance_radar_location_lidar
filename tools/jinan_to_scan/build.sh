#!/bin/bash
# 构建 jinan_to_xyz 工具
# 用法: bash tools/jinan_to_scan/build.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
BUILD_DIR="${SCRIPT_DIR}/build"

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --parallel

echo "[jinan_to_scan] Built: ${BUILD_DIR}/jinan_to_xyz"
echo ""
echo "Usage examples:"
echo "  # 全量转换 (5.9M点 → 转成xyz, 无ROI过滤)"
echo "  ${BUILD_DIR}/jinan_to_xyz model/pcd_rmuc2026_jinan.pcd model/generated/jinan_xyz.pcd"
echo ""
echo "  # 限制高度范围 (只保留z<6m的点, 去掉屋顶/看台高处结构, 减小体积)"
echo "  ${BUILD_DIR}/jinan_to_xyz model/pcd_rmuc2026_jinan.pcd model/generated/jinan_xyz.pcd \\"
echo "    --roi -62,62,-62,62,-1,6"
echo ""
echo "  # 场地内及近周边 (|x|<20, |y|<12, z<5m)"
echo "  ${BUILD_DIR}/jinan_to_xyz model/pcd_rmuc2026_jinan.pcd model/generated/jinan_xyz.pcd \\"
echo "    --roi -20,20,-12,12,-1,5"
