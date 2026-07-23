from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence
import math


@dataclass(frozen=True)
class MotionCommand:
    action: str
    linear: float
    angular: float


class DepthCameraNavigator:
    def __init__(self, safe_distance: float = 0.8, max_speed: float = 0.4, turn_rate: float = 0.8) -> None:
        if safe_distance <= 0:
            raise ValueError("safe_distance must be > 0")
        self.safe_distance = safe_distance
        self.max_speed = max_speed
        self.turn_rate = turn_rate

    def decide(self, depth_frame: Sequence[Sequence[float]]) -> MotionCommand:
        if not depth_frame or not depth_frame[0]:
            raise ValueError("depth_frame must be a non-empty 2D array")

        width = len(depth_frame[0])
        for row in depth_frame:
            if len(row) != width:
                raise ValueError("depth_frame rows must have equal width")

        left, center, right = self._sector_clearance(depth_frame)

        if center >= self.safe_distance:
            speed_scale = min(1.0, center / (self.safe_distance * 2.0))
            return MotionCommand(action="FORWARD", linear=self.max_speed * speed_scale, angular=0.0)

        if right > left and right >= self.safe_distance * 0.8:
            return MotionCommand(action="TURN_RIGHT", linear=0.0, angular=-self.turn_rate)

        if left >= self.safe_distance * 0.8:
            return MotionCommand(action="TURN_LEFT", linear=0.0, angular=self.turn_rate)

        return MotionCommand(action="STOP", linear=0.0, angular=0.0)

    def _sector_clearance(self, frame: Sequence[Sequence[float]]) -> tuple[float, float, float]:
        width = len(frame[0])
        one_third = max(1, width // 3)

        left_values = []
        center_values = []
        right_values = []

        for row in frame:
            left_values.extend(row[:one_third])
            center_values.extend(row[one_third : width - one_third])
            right_values.extend(row[width - one_third :])

        return (
            self._min_valid(left_values),
            self._min_valid(center_values),
            self._min_valid(right_values),
        )

    @staticmethod
    def _min_valid(values: Sequence[float]) -> float:
        valid = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v) and v > 0]
        return min(valid) if valid else 0.0
