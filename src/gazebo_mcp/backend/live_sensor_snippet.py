def get_pose(self, name: str) -> dict[str, Any]:
        return self._unsupported("get_pose")

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
        return self._unsupported("set_pose")

    def sensor_snapshot(
        self,
        sensor_type: str = "lidar",
        model_name: str | None = None,
    ) -> dict[str, Any]:
        return self._unsupported("sensor_snapshot")

    def pause(self) -> dict[str, Any]:
        return self._unsupported("pause")