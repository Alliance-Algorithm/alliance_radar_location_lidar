from radar_interfaces.msg import CameraDetection, CameraDetectionArray


def test_camera_detection_contract():
    assert CameraDetection.TEAM_UNKNOWN == 0
    assert CameraDetection.TEAM_RED == 1
    assert CameraDetection.TEAM_BLUE == 2
    assert CameraDetection.CLASS_UNKNOWN == 0
    assert CameraDetection.CLASS_HERO == 1
    assert CameraDetection.CLASS_ENGINEER == 2
    assert CameraDetection.CLASS_INFANTRY_3 == 3
    assert CameraDetection.CLASS_INFANTRY_4 == 4
    assert CameraDetection.CLASS_AERIAL == 5
    assert CameraDetection.CLASS_SENTRY == 6

    detection = CameraDetection()
    assert type(detection.position).__name__ == "Point"
    assert isinstance(detection.team, int)
    assert isinstance(detection.semantic_class, int)
    assert isinstance(detection.confidence, float)

    detection_array = CameraDetectionArray()
    assert type(detection_array.header).__name__ == "Header"
    assert detection_array.detections == []
