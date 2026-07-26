"""Tests for pose get/set, sensor snapshot, and model graph features."""

from __future__ import annotations

import json
import math

from gazebo_mcp import server as srv
from gazebo_mcp.backend.mock import MockBackend


# --- Pose get/set tests (bounty #20) ---

def test_pose_get_returns_pose_and_twist():
    b = MockBackend()
    b.seed_demo()
    pose = b.get_pose("box_1")
    assert pose["ok"] is True
    assert "pose" in pose
    assert "twist" in pose
    assert pose["name"] == "box_1"
    assert isinstance(pose["pose"]["x"], float)
    assert isinstance(pose["twist"]["linear"]["x"], float)


def test_pose_get_unknown_model():
    b = MockBackend()
    b.seed_demo()
    result = b.get_pose("nonexistent")
    assert result["ok"] is False
    assert "error" in result


def test_pose_set_updates_pose():
    b = MockBackend()
    b.seed_demo()
    result = b.set_pose("box_1", 2.0, 3.0, 0.5, 1.57)
    assert result["ok"] is True
    assert result["pose"]["x"] == 2.0
    assert result["pose"]["y"] == 3.0
    assert result["pose"]["yaw"] == 1.57
    # Verify via get_pose
    pose = b.get_pose("box_1")
    assert pose["pose"]["x"] == 2.0


def test_pose_set_unknown_model():
    b = MockBackend()
    b.seed_demo()
    result = b.set_pose("nonexistent", 0.0, 0.0, 0.0)
    assert result["ok"] is False


def test_pose_set_with_velocity():
    b = MockBackend()
    b.seed_demo()
    result = b.set_pose(
        "box_1", 1.0, 1.0, 0.5, 0.0,
        linear_velocity={"x": 0.5, "y": 0.0, "z": 0.0},
        angular_velocity={"x": 0.0, "y": 0.0, "z": 0.2},
    )
    assert result["twist"]["linear"]["x"] == 0.5
    assert result["twist"]["angular"]["z"] == 0.2


def test_server_pose_tools():
    """Server-level pose tools work end-to-end."""
    result = json.loads(srv.gazebo_get_pose("box_1"))
    assert result["ok"] is True
    assert result["pose"]["x"] > 0.0  # any valid pose x

    result2 = json.loads(srv.gazebo_set_pose("box_1", 3.0, 4.0, 0.5, 0.0))
    assert result2["ok"] is True
    assert result2["pose"]["x"] == 3.0


# --- Sensor snapshot tests (bounty #23) ---

def test_sensor_snapshot_lidar():
    """Sensor snapshot returns lidar frame data in 'frame' key."""
    b = MockBackend()
    b.seed_demo()
    result = b.sensor_snapshot("lidar")
    assert result["ok"] is True
    assert result["sensor_type"] == "lidar"
    assert "frame" in result
    frame = result["frame"]
    assert frame["n_rays"] == 360
    assert len(frame["ranges"]) == 360
    assert frame["range_min"] == 0.1
    assert frame["frame_id"] == "lidar_mock"
    assert all(isinstance(r, float) for r in frame["ranges"])


def test_sensor_snapshot_camera():
    """Sensor snapshot returns camera frame data in 'frame' key."""
    b = MockBackend()
    b.seed_demo()
    result = b.sensor_snapshot("camera")
    assert result["ok"] is True
    assert result["sensor_type"] == "camera"
    assert "frame" in result
    frame = result["frame"]
    assert frame["width"] == 640
    assert frame["height"] == 480
    assert frame["encoding"] == "32FC1"


def test_sensor_snapshot_all():
    """Sensor snapshot with 'all' returns lidar frame (last set)."""
    b = MockBackend()
    b.seed_demo()
    result = b.sensor_snapshot("all")
    assert result["ok"] is True
    assert result["sensor_type"] == "all"
    assert "frame" in result  # last frame set is lidar


def test_sensor_snapshot_default_is_lidar():
    b = MockBackend()
    b.seed_demo()
    result = b.sensor_snapshot()
    assert result["sensor_type"] == "lidar"
    assert "frame" in result


def test_server_sensor_snapshot_tool():
    result = json.loads(srv.gazebo_sensor_snapshot("camera"))
    assert result["ok"] is True
    assert result["sensor_type"] == "camera"
    assert "frame" in result


# --- Model graph tests (bounty #19) ---

def test_model_graph_initial_state():
    b = MockBackend()
    b.seed_demo()
    graph = b.model_graph()
    assert graph["ok"] is True
    assert graph["mode"] == "mock"
    # shapes_demo has 3 models: ground_plane, box_1, sphere_1
    assert graph["node_count"] == 3
    assert len(graph["graph"]) == 3
    names = {n["name"] for n in graph["graph"]}
    assert names == {"ground_plane", "box_1", "sphere_1"}


def test_model_graph_updates_on_spawn():
    b = MockBackend()
    b.seed_demo()
    b.spawn("new_robot", "robot", 0.0, 0.0, 0.1)
    graph = b.model_graph()
    assert graph["node_count"] == 4
    assert any(n["name"] == "new_robot" for n in graph["graph"])


def test_model_graph_updates_on_delete():
    b = MockBackend()
    b.seed_demo()
    b.delete("box_1")
    graph = b.model_graph()
    assert graph["node_count"] == 2
    names = {n["name"] for n in graph["graph"]}
    assert "box_1" not in names


def test_model_graph_state_consistent_with_models():
    """Graph node count matches model count after spawn/delete."""
    b = MockBackend()
    b.seed_demo()
    assert b.model_graph()["node_count"] == len(b.list_models())
    b.spawn("extra", "box", 1.0, 1.0, 0.5)
    assert b.model_graph()["node_count"] == len(b.list_models())
    b.delete("extra")
    assert b.model_graph()["node_count"] == len(b.list_models())


def test_server_model_graph_tool():
    result = json.loads(srv.gazebo_model_graph())
    assert result["ok"] is True
    assert result["node_count"] >= 2
