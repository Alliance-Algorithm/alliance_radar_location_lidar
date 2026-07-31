import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
import pytest


def load_launch_module():
    launch_path = (
        Path(get_package_share_directory("radar_bringup"))
        / "launch"
        / "competition.launch.py"
    )
    spec = importlib.util.spec_from_file_location("competition_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def runtime_files(tmp_path):
    paths = {}
    for name in ("map_path", "camera_config", "fusion_config", "bridge_config"):
        path = tmp_path / name
        path.touch()
        paths[name] = str(path)
    return paths


def launch_context(runtime_files, **overrides):
    values = {
        "side": "red",
        "sensor": "odin",
        "registration_timeout_sec": "30.0",
        **runtime_files,
        **overrides,
    }
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"side": "green"}, "Unsupported side 'green'"),
        ({"sensor": "velodyne"}, "Unsupported sensor 'velodyne'"),
        ({"registration_timeout_sec": "0"}, "must be a positive number"),
        ({"registration_timeout_sec": "nan"}, "must be a positive number"),
        ({"map_path": "/missing/map.pcd"}, "map_path does not exist"),
        ({"camera_config": "/missing/camera.yaml"}, "camera_config does not exist"),
        ({"fusion_config": "/missing/fusion.yaml"}, "fusion_config does not exist"),
        ({"bridge_config": "/missing/bridge.yaml"}, "bridge_config does not exist"),
    ],
)
def test_invalid_input_is_rejected(runtime_files, overrides, message):
    module = load_launch_module()

    with pytest.raises(RuntimeError, match=message):
        module.validated_startup(launch_context(runtime_files, **overrides))


def test_opaque_validation_prevents_scheduled_hardware_stub(runtime_files, tmp_path):
    module = load_launch_module()
    marker = tmp_path / "hardware-started"
    values = {
        "side": "green",
        "sensor": "odin",
        "registration_timeout_sec": "30.0",
        **runtime_files,
    }
    service = LaunchService()
    service.include_launch_description(
        LaunchDescription([
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in values.items()
            ],
            OpaqueFunction(function=module.validated_startup),
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    "-c",
                    f"from pathlib import Path; Path({str(marker)!r}).touch()",
                ]
            ),
        ])
    )

    assert service.run() != 0
    assert not marker.exists()


class StubExit:
    def __init__(self, action, returncode):
        self.action = action
        self.returncode = returncode


def test_registration_failure_requests_shutdown(runtime_files):
    module = load_launch_module()
    gate = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    consumer = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    handler = module.critical_process_handler(gate, [consumer])

    actions = handler.event_handler._OnActionEventBase__on_event(
        StubExit(gate, 7), launch_context(runtime_files)
    )

    assert len(actions) == 1
    assert actions[0].event.reason == "registration gate failed with exit code 7"


def test_successful_registration_starts_consumers(runtime_files):
    module = load_launch_module()
    gate = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    consumers = [
        ExecuteProcess(cmd=[sys.executable, "-c", "pass"]),
        ExecuteProcess(cmd=[sys.executable, "-c", "pass"]),
    ]
    handler = module.critical_process_handler(gate, consumers)

    actions = handler.event_handler._OnActionEventBase__on_event(
        StubExit(gate, 0), launch_context(runtime_files)
    )

    assert actions == consumers


def test_critical_child_exit_requests_whole_launch_shutdown(runtime_files):
    module = load_launch_module()
    gate = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    child = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    handler = module.critical_process_handler(gate, [child])

    actions = handler.event_handler._OnActionEventBase__on_event(
        StubExit(child, 0), launch_context(runtime_files)
    )

    assert len(actions) == 1
    assert actions[0].event.reason == "critical process exited with code 0"


def test_critical_exit_during_coordinated_shutdown_is_ignored(runtime_files):
    module = load_launch_module()
    gate = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    child = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    handler = module.critical_process_handler(gate, [child])
    context = launch_context(runtime_files)
    context._set_is_shutdown(True)

    actions = handler.event_handler._OnActionEventBase__on_event(
        StubExit(child, -15), context
    )

    assert actions is None


def test_stubbed_critical_child_exit_stops_running_sibling(tmp_path):
    module = load_launch_module()
    ready = tmp_path / "sibling-ready"
    stopped = tmp_path / "sibling-stopped"
    gate = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    failing_child = ExecuteProcess(
        cmd=[
            sys.executable,
            "-c",
            (
                "import pathlib, time\n"
                f"ready = pathlib.Path({str(ready)!r})\n"
                "while not ready.exists():\n"
                "    time.sleep(0.01)\n"
                "raise SystemExit(3)\n"
            ),
        ]
    )
    sibling = ExecuteProcess(
        cmd=[
            sys.executable,
            "-c",
            (
                "import pathlib, signal, time; "
                f"signal.signal(signal.SIGINT, lambda *_: pathlib.Path({str(stopped)!r}).touch()); "
                f"pathlib.Path({str(ready)!r}).touch(); "
                "time.sleep(30)"
            ),
        ]
    )
    service = LaunchService()
    service.include_launch_description(
        LaunchDescription([
            module.critical_process_handler(gate, [failing_child, sibling]),
            gate,
        ])
    )

    assert service.run() == 0
    assert stopped.is_file()
