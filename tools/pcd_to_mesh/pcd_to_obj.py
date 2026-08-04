#!/usr/bin/env python3
"""PCD -> OBJ via open3d voxel downsample + normals + Ball Pivoting."""
import argparse, sys, time
import numpy as np
import open3d as o3d

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("input_pcd")
    p.add_argument("output_obj")
    p.add_argument("--voxel", type=float, default=0.1)
    p.add_argument("--radii", default="0.25,0.5,0.75")
    p.add_argument("--max-faces", type=int, default=300000)
    a = p.parse_args()
    t0 = time.time()
    cloud = o3d.io.read_point_cloud(a.input_pcd)
    if len(cloud.points) == 0:
        print(f"ERROR: no points in {a.input_pcd}", file=sys.stderr)
        return 1
    cloud = cloud.voxel_down_sample(a.voxel)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(30))
    radii = [float(x) for x in a.radii.split(",")]
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud, o3d.utility.DoubleVector(radii))
    if len(mesh.triangles) == 0:
        print(f"ERROR: ball pivoting produced no triangles (try smaller voxel/larger radii)",
              file=sys.stderr)
        return 1
    if len(mesh.triangles) > a.max_faces:
        print(f"WARNING: {len(mesh.triangles)} faces > {a.max_faces}; applying decimation")
        mesh = mesh.simplify_quadric_decimation(a.max_faces)
    pts = np.asarray(mesh.vertices)
    o3d.io.write_triangle_mesh(a.output_obj, mesh)
    print(f"points={len(cloud.points)} verts={len(mesh.vertices)} "
          f"faces={len(mesh.triangles)} bounds=[{pts.min(0).round(2)}]..[{pts.max(0).round(2)}] "
          f"t={time.time()-t0:.1f}s")
    return 0

if __name__ == "__main__":
    sys.exit(main())
