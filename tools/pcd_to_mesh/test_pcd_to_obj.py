# tools/pcd_to_mesh/test_pcd_to_obj.py
import subprocess, sys, os
REPO = "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar"
def run(args):
    return subprocess.run([sys.executable, os.path.join(REPO, "tools/pcd_to_mesh/pcd_to_obj.py")] + args,
                          capture_output=True, text=True)

def test_smoke_reg_pcd():
    r = run([os.path.join(REPO, "model/generated/jinan_field_map_reg.pcd"),
             "/tmp/opencode/test_reg.obj"])
    assert r.returncode == 0, r.stderr
    assert "faces=" in r.stdout
    assert "bounds" in r.stdout

def test_missing_input_fails():
    r = run(["/tmp/opencode/does_not_exist.pcd", "/tmp/opencode/x.obj"])
    assert r.returncode != 0
