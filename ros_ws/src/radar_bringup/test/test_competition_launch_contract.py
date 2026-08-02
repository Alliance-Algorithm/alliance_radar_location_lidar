import ast
import pathlib
import unittest


EXPECTED_DEFAULTS = {
    "enable_raw_recording": "false",
    "recording_output_dir": "/workspace/model/video",
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
        recording_parameters = self._recording_parameters_assignment()
        parameter_names = {
            key.value
            for key, value in self._dict_entries(recording_parameters.value)
            if isinstance(key, ast.Constant)
            and isinstance(value, ast.Call)
            and isinstance(value.func, ast.Name)
            and value.func.id == "LaunchConfiguration"
            and len(value.args) == 1
            and isinstance(value.args[0], ast.Constant)
        }

        self.assertEqual(parameter_names, set(EXPECTED_DEFAULTS))

        radar_camera_node = self._radar_camera_node()
        parameters = self._keyword_value(radar_camera_node, "parameters")
        self.assertIsInstance(parameters, ast.List)
        self.assertTrue(self._contains_name(parameters.elts, "recording_parameters"))

    def test_disabled_mode_uses_exact_default_on_radar_camera_override_path(self):
        disabled_default = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "enable_raw_recording"
        )
        self.assertEqual(
            self._keyword_value(disabled_default, "default_value").value,
            "false",
        )

        radar_parameters = self._keyword_value(self._radar_camera_node(), "parameters")
        self.assertTrue(self._contains_name(radar_parameters.elts, "recording_parameters"))

    def _recording_parameters_assignment(self):
        return next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "recording_parameters"
                for target in node.targets
            )
        )

    def _radar_camera_node(self):
        return next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "Node"
            and self._keyword_string(node, "name") == "radar_camera_node"
        )

    @staticmethod
    def _dict_entries(dictionary):
        return zip(dictionary.keys, dictionary.values)

    @staticmethod
    def _keyword_value(call, name):
        return next(keyword.value for keyword in call.keywords if keyword.arg == name)

    def _keyword_string(self, call, name):
        value = self._keyword_value(call, name)
        return value.value if isinstance(value, ast.Constant) else None

    @staticmethod
    def _contains_name(nodes, name):
        return any(isinstance(node, ast.Name) and node.id == name for node in nodes)


if __name__ == "__main__":
    unittest.main()
