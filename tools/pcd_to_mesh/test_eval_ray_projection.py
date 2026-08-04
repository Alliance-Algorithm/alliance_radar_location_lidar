import numpy as np
from eval_ray_projection import R_from_rpy, pixel_to_ray, mt_intersect
import trimesh

REPO = "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar"
FX, FY, CX, CY = 6753.698616, 6737.450110, 2620.748274, 1924.062270
ROT = [0.0, 1.8159, 3.14159]
TRANS = [13.9965, 0.0800, 3.9803]

def test_rpy_construction():
    R = R_from_rpy(*ROT)
    # 光轴 (+Z) 应朝下(负 z)并朝场地内(负 x, 蓝方在 +x 端)
    dz = R @ np.array([0.0, 0.0, 1.0])
    assert dz[2] < 0 and dz[0] < 0

def test_known_projection_hits_ground():
    mesh = trimesh.load(f"{REPO}/model/generated/field_zup.obj", process=False)
    tris = np.asarray(mesh.triangles).reshape(-1, 3, 3)
    origin, R = TRANS, R_from_rpy(*ROT)
    hit = mt_intersect(np.array(origin), pixel_to_ray(3836.81, 3578.17, R, FX, FY, CX, CY), tris)
    assert hit is not None
    assert abs(hit[2]) < 0.5          # 命中地面
    assert -14 <= hit[0] <= 14 and -7.5 <= hit[1] <= 7.5
