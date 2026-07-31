from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from radar_interfaces.msg import RegistrationStatus


@pytest.fixture
def status_publisher():
    rclpy.init()
    node = rclpy.create_node("registration_gate_test_publisher")
    publisher = node.create_publisher(
        RegistrationStatus,
        "/localization/registration_status",
        rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    yield publisher
    node.destroy_node()
    rclpy.shutdown()


def run_gate(status_publisher, state, reason=""):
    message = RegistrationStatus(state=state, reason=reason)
    status_publisher.publish(message)

    executable = (
        Path(get_package_prefix("radar_bringup"))
        / "lib"
        / "radar_bringup"
        / "registration_gate"
    )
    return subprocess.run(
        [str(executable)],
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )


def test_late_gate_exits_zero_for_latched_locked_status(status_publisher):
    result = run_gate(status_publisher, RegistrationStatus.LOCKED)

    assert result.returncode == 0


def test_gate_exits_nonzero_and_reports_latched_failure(status_publisher):
    result = run_gate(
        status_publisher,
        RegistrationStatus.FAILED,
        "registration quality below threshold",
    )

    assert result.returncode != 0
    assert result.stderr == "registration quality below threshold\n"


def test_gate_preserves_empty_latched_failure_reason(status_publisher):
    result = run_gate(status_publisher, RegistrationStatus.FAILED, "")

    assert result.returncode != 0
    assert result.stderr == "\n"
