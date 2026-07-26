"""Offline Gazebo-style world mock."""

from __future__ import annotations

import time
import math
from typing import Any
import random


class MockBackend:
    name = "mock"

    def __init__(self) -> None:
        self.seed_demo()

    def seed_demo(self, profile: str = "default") -> dict[str, Any]:
        profile = (profile or "default").strip().lower()
        self._world = "fleet_demo" if profile == "fleet" else "shapes_demo"
        self._paused = False
        self._t0 = time.time()
        self._sim_time = 0.0
        self._models = self._seed_models(profile)
        self._graph = self._seed_graph()
        return {
            "ok": True,
            "profile": profile,
            "world": self._world,
            "models": list(self._models),
        }

    def _seed_models(self, profile: str) -> dict[str, dict[str, Any]]:
        models: dict[str, dict[str, Any]] = {
            "ground_plane": {
                "name": "ground_plane",
                "type": "plane",
                "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                "twist": self._twist(),
            },
        }
        if profile == "fleet":
            models.update(
                {
                    "robot_0": {
                        "name": "robot_0",
                        "type": "robot",
                        "pose": {"x": -1.0, "y": -1.0, "z": 0.1, "yaw": 0.0},
                    },
                    "robot_1": {
                        "name": "robot_1",
                        "type": "robot",
                        "pose": {"x": 0.0, "y": 1.0, "z": 0.1, "yaw": 1.57},
                    },
                    "robot_2": {
                        "name": "robot_2",
                        "type": "robot",
                        "pose": {"x": 1.0, "y": -1.0, "z": 0.1, "yaw": 3.14},
                    },
                }
            )
            return models
        models.update(
            {
            "box_1": {
                "name": "box_1",
                "type": "box",
                "pose": {"x": 1.0, "y": 0.0, "z": 0.5, "yaw": 0.0},
                "twist": self._twist(),
            },
            "sphere_1": {
                "name": "sphere_1",
                "type": "sphere",
                "pose": {"x": -1.0, "y": 0.5, "z": 0.5, "yaw": 0.0},
                "twist": self._twist(),
            },
            }
        )
        return models

    def _seed_graph(self) -> dict[str, dict[str, Any]]:
        """Seed parent-child relationship graph for models."""
        graph: dict[str, dict[str, Any]] = {}
        for name, m in self._models.items():
            graph[name] = {
                "name": name,
                "parent": None,
                "children": [],
                "type": m.get("type", "unknown"),
                "joint_type": "fixed",
            }
        return graph

    def _synthetic_lidar(self) -> dict[str, Any]:
        """Generate synthetic lidar scan data."""
        n_rays = 360
        angle_min = 0.0
        angle_max = 2.0 * math.pi
        angle_increment = (angle_max - angle_min) / n_rays
        ranges = []
        for i in range(n_rays):
            angle = angle_min + i * angle_increment
            base = 5.0 + random.uniform(-0.5, 0.5)
            for model_name, m in self._models.items():
                if model_name == "ground_plane":
                    continue
                # Simple ray-to-model distance check
                dx = m["pose"]["x"]
                dy = m["pose"]["y"]
                ray_angle = math.atan2(dy, dx)
                angle_diff = abs(angle - ray_angle)
                if angle_diff < 0.1:  # Ray roughly pointing at model
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist < base and dist > 0.3:
                        base = dist + random.uniform(-0.05, 0.05)
            ranges.append(round(base, 3))
        return {
            "n_rays": n_rays,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "range_min": 0.1,
            "range_max": 5.0,
            "ranges": ranges,
            "frame_id": "lidar_mock",
            "timestamp_sec": round(self._sim_time, 3),
        }

    def _synthetic_camera(self) -> dict[str, Any]:
        """Generate synthetic camera depth data."""
        # Generate mock depth frame
        width, height = 640, 480
        depth_data = []
        for y in range(height):
            row = []
            for x in range(width):
                # Simple depth based on position
                d = 2.0 + 0.5 * math.sin(x * 0.01) + 0.3 * math.cos(y * 0.01)
                row.append(round(d, 3))
            depth_data.append(row)
        return {
            "width": width,
            "height": height,
            "encoding": "32FC1",
            "depth_data_summary": {
                "min": min(min(row) for row in depth_data),
                "max": max(max(row) for row in depth_data),
            },
            "frame_id": "camera_mock",
            "timestamp_sec": round(self._sim_time, 3),
        }

    def sensor_snapshot(self, sensor_type: str = "lidar") -> dict[str, Any]:
        """Return synthetic sensor data for agent workflows.

        Args:
            sensor_type: One of "lidar", "camera", or "all".

        Returns:
            Dictionary with sensor frame data.
        """
        if not self._paused:
            self._sim_time = time.time() - self._t0
        st = sensor_type.strip().lower()
        if st not in ("lidar", "camera", "all"):
            return {
                "ok": False,
                "error": f"unsupported sensor type: {sensor_type}",
            }
        result: dict[str, Any] = {
            "ok": True,
            "sensor_type": st,
            "world": self._world,
            "model_count": len(self._models),
            "timestamp": round(self._sim_time, 3),
            "schema": {"type": "object", "properties": {}},
        }
        if st in ("lidar", "all"):
            result["frame"] = self._synthetic_lidar()
        if st in ("camera", "all"):
            result["frame"] = self._synthetic_camera()
        return result

    def model_graph(self) -> dict[str, Any]:
        """Return the parent-child relationship graph of models."""
        return {
            "ok": True,
            "world": self._world,
            "mode": "mock",
            "graph": list(self._graph.values()),
            "node_count": len(self._graph),
            "edge_count": sum(len(n["children"]) for n in self._graph.values()),
        }

    def _twist(
        self,
        linear_velocity: dict[str, Any] | None = None,
        angular_velocity: dict[str, Any] | None = None,
    ) -> dict[str, dict[str, float]]:
        linear_velocity = linear_velocity or {}
        angular_velocity = angular_velocity or {}
        return {
            "linear": {
                "x": float(linear_velocity.get("x", 0.0)),
                "y": float(linear_velocity.get("y", 0.0)),
                "z": float(linear_velocity.get("z", 0.0)),
            },
            "angular": {
                "x": float(angular_velocity.get("x", 0.0)),
                "y": float(angular_velocity.get("y", 0.0)),
                "z": float(angular_velocity.get("z", 0.0)),
            },
        }

    def doctor(self) -> dict[str, Any]:
        return {
            "ok": True,
            "connected": True,
            "mode": "mock",
            "gazebo_required": False,
            "message": "Mock Gazebo world active — no Gazebo install needed",
            "world": self._world,
            "model_count": len(self._models),
            "paused": self._paused,
            "sim_time_sec": round(self._sim_time, 3),
        }

    def world_info(self) -> dict[str, Any]:
        if not self._paused:
            self._sim_time = time.time() - self._t0
        return {
            "world": self._world,
            "paused": self._paused,
            "sim_time_sec": round(self._sim_time, 3),
            "model_count": len(self._models),
            "physics": "ode-mock",
        }

    def list_worlds(self) -> dict[str, Any]:
        """List deterministic offline world fixtures and mark the active world."""
        worlds = [
            {
                "name": "shapes_demo",
                "profile": "default",
                "model_count": 3,
                "active": self._world == "shapes_demo",
            },
            {
                "name": "fleet_demo",
                "profile": "fleet",
                "model_count": 4,
                "active": self._world == "fleet_demo",
            },
        ]
        return {
            "ok": True,
            "mode": "mock",
            "current_world": self._world,
            "worlds": worlds,
        }

    def list_models(self) -> list[dict[str, Any]]:
        return list(self._models.values())

    def snapshot(self) -> dict[str, Any]:
        """Full world snapshot: models, poses, sim time, physics params."""
        if not self._paused:
            self._sim_time = time.time() - self._t0
        return {
            "ok": True,
            "world": self._world,
            "paused": self._paused,
            "sim_time_sec": round(self._sim_time, 3),
            "model_count": len(self._models),
            "models": list(self._models.values()),
            "physics": {
                "engine": "ode-mock",
                "max_step_size": 0.001,
                "real_time_factor": 1.0,
                "gravity": {"x": 0.0, "y": 0.0, "z": -9.8},
            },
        }

    def spawn(
        self,
        name: str,
        model_type: str,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
    ) -> dict[str, Any]:
        if name in self._models:
            return {"ok": False, "error": f"model {name} already exists"}
        self._models[name] = {
            "name": name,
            "type": model_type or "box",
            "pose": {"x": float(x), "y": float(y), "z": float(z), "yaw": float(yaw)},
            "twist": self._twist(),
        }
        self._graph[name] = {
            "name": name,
            "parent": None,
            "children": [],
            "type": model_type or "box",
            "joint_type": "fixed",
        }
        return {"ok": True, "model": self._models[name]}

    def delete(self, name: str) -> dict[str, Any]:
        if name not in self._models:
            return {"ok": False, "error": f"unknown model {name}"}
        if name == "ground_plane":
            return {"ok": False, "error": "cannot delete ground_plane"}
        del self._models[name]
        # Remove from graph and detach from parent
        node = self._graph.pop(name, None)
        if node and node.get("parent"):
            parent_name = node["parent"]
            if parent_name in self._graph:
                self._graph[parent_name]["children"] = [
                    c for c in self._graph[parent_name]["children"] if c != name
                ]
        # Orphan children
        if node:
            for child_name in node.get("children", []):
                if child_name in self._graph:
                    self._graph[child_name]["parent"] = None
        return {"ok": True, "deleted": name}

    def get_pose(self, name: str) -> dict[str, Any]:
        m = self._models.get(name)
        if not m:
            return {"ok": False, "error": f"unknown model {name}"}
        return {"ok": True, "name": name, "pose": m["pose"], "twist": m.get("twist", self._twist())}

    def set_pose(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
        linear_velocity: dict[str, Any] | None = None,
        angular_velocity: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        m = self._models.get(name)
        if not m:
            return {"ok": False, "error": f"unknown model {name}"}
        m["pose"] = {"x": float(x), "y": float(y), "z": float(z), "yaw": float(yaw)}
        m["twist"] = self._twist(linear_velocity, angular_velocity)
        return {"ok": True, "name": name, "pose": m["pose"], "twist": m["twist"]}

    def pause(self) -> dict[str, Any]:
        self._paused = True
        return {"ok": True, "paused": True, "sim_time_sec": round(self._sim_time, 3)}

    def unpause(self) -> dict[str, Any]:
        self._paused = False
        self._t0 = time.time() - self._sim_time
        return {"ok": True, "paused": False}

    def step(self, steps: int = 1) -> dict[str, Any]:
        n = max(1, int(steps))
        self._sim_time += 0.001 * n
        self._paused = True
        return {"ok": True, "steps": n, "sim_time_sec": round(self._sim_time, 3), "paused": True}
