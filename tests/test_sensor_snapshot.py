"""Tests for sensor snapshot tools (bounty #23)."""

from gazebo_mcp.backend.mock import MockBackend


def test_sensor_snapshot_lidar_default():
    b = MockBackend()
    result = b.sensor_snapshot(sensor_type="lidar")

    assert result["ok"] is True
    assert result["sensor_type"] == "lidar"
    assert "timestamp" in result
    assert result["world"] == "shapes_demo"

    frame = result["frame"]
    assert frame["n_rays"] == 360
    assert frame["angle_min"] == 0.0
    assert frame["angle_max"] == 6.283185307179586
    assert len(frame["ranges"]) == 360
    assert frame["range_min"] == 0.1
    assert frame["range_max"] == 5.0
    # All ranges should be positive
    assert all(r > 0 for r in frame["ranges"])

    # Schema should be documented
    assert "schema" in result
    assert result["schema"]["type"] == "object"


def test_sensor_snapshot_lidar_has_model_reflections():
    b = MockBackend()
    result = b.sensor_snapshot(sensor_type="lidar")

    # Box_1 is at (1, 0, 0.5), so at angle 0 we should detect it
    frame = result["frame"]
    # Ray at angle 0 (pointing right) should see box_1 at distance ~1.0
    mid_idx = 0  # angle 0 = right
    mid_range = frame["ranges"][mid_idx]
    # Box is at x=1, so range should be ~1.0 (within tolerance)
    assert 0.5 < mid_range < 2.0, f"Expected box_1 at ~1.0m, got {mid_range}"


def test_sensor_snapshot_camera_default():
    b = MockBackend()
    result = b.sensor_snapshot(sensor_type="camera")

    assert result["ok"] is True
    assert result["sensor_type"] == "camera"
    assert "timestamp" in result

    frame = result["frame"]
    assert frame["width"] == 640
    assert frame["height"] == 480
    assert frame["encoding"] == "32FC1"
    assert "depth_data_summary" in frame
    summary = frame["depth_data_summary"]
    assert summary["min"] > 0
    assert summary["max"] > summary["min"]

    # Schema should be documented
    assert "schema" in result
    assert result["schema"]["type"] == "object"


def test_sensor_snapshot_unsupported_type():
    b = MockBackend()
    result = b.sensor_snapshot(sensor_type="infrared")
    assert result["ok"] is False
    assert "unsupported" in result["error"]


def test_sensor_snapshot_fleet_world():
    b = MockBackend()
    b.seed_demo(profile="fleet")
    result = b.sensor_snapshot(sensor_type="lidar")

    assert result["ok"] is True
    assert result["world"] == "fleet_demo"
    assert result["model_count"] == 4

    frame = result["frame"]
    assert len(frame["ranges"]) == 360


def test_sensor_snapshot_after_spawn():
    b = MockBackend()
    b.spawn("test_robot", "robot", 2.0, 0.0, 0.1)
    result = b.sensor_snapshot(sensor_type="lidar")

    assert result["ok"] is True
    assert result["model_count"] == 4  # ground_plane + box_1 + sphere_1 + test_robot