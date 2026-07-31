import ast
import pathlib
import unittest


EXPECTED_DEFAULTS = {
    "enable_raw_recording": "false",
    "recording_output_dir": "/data/competition/recordings",
    "recording_width": "5472",
    "recording_height": "3648",
    "recording_fps": "20",
    "recording_bitrate": "40000000",
    "recording_gop": "20",
    "recording_encoder": "h264_nvenc",
    "recording_segment_duration_sec": "60",
    "recording_buffer_pool_frames": "8",
    "recording_max_buffer_bytes": "480000000",
}


class CompetitionLaunchContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        launch_path = pathlib.Path(__file__).parents[1] / "launch" / "competition.launch.py"
        cls.source = launch_path.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source, filename=str(launch_path))

    def test_declares_recording_defaults(self):
        declarations = {}
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "DeclareLaunchArgument" or not node.args:
                continue
            name = ast.literal_eval(node.args[0])
            defaults = {keyword.arg: ast.literal_eval(keyword.value)
                        for keyword in node.keywords if keyword.arg == "default_value"}
            declarations[name] = defaults.get("default_value")

        self.assertEqual(
            {name: declarations.get(name) for name in EXPECTED_DEFAULTS},
            EXPECTED_DEFAULTS,
        )

    def test_starts_camera_and_preserves_pipeline(self):
        packages = []
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            if node.func.id == "Node":
                keywords = {keyword.arg: keyword.value for keyword in node.keywords}
                if "package" in keywords:
                    packages.append(ast.literal_eval(keywords["package"]))

        self.assertIn("radar_camera", packages)
        self.assertIn("radar_fusion", packages)
        self.assertIn("hikcamera.launch.py", self.source)
        self.assertIn("localization.launch.py", self.source)
        self.assertIn("radar_bridge.launch.py", self.source)

    def test_forwards_every_recording_parameter(self):
        launch_configurations = {
            node.args[0].value
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "LaunchConfiguration"
            and len(node.args) == 1
            and isinstance(node.args[0], ast.Constant)
        }
        self.assertTrue(set(EXPECTED_DEFAULTS).issubset(launch_configurations))


if __name__ == "__main__":
    unittest.main()
