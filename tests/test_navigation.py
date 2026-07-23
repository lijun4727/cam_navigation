import unittest

from navigation import DepthCameraNavigator


class DepthCameraNavigatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.navigator = DepthCameraNavigator(safe_distance=1.0, max_speed=0.5, turn_rate=1.0)

    def test_moves_forward_when_center_is_clear(self) -> None:
        frame = [
            [2.0, 2.0, 2.0],
            [2.0, 2.0, 2.0],
        ]
        command = self.navigator.decide(frame)
        self.assertEqual(command.action, "FORWARD")
        self.assertGreater(command.linear, 0)
        self.assertEqual(command.angular, 0.0)

    def test_turns_right_when_center_blocked_and_right_clearer(self) -> None:
        frame = [
            [0.9, 0.3, 1.5],
            [0.8, 0.4, 1.6],
        ]
        command = self.navigator.decide(frame)
        self.assertEqual(command.action, "TURN_RIGHT")
        self.assertLess(command.angular, 0)

    def test_turns_left_when_center_blocked_and_left_clearer(self) -> None:
        frame = [
            [1.5, 0.3, 0.5],
            [1.6, 0.4, 0.5],
        ]
        command = self.navigator.decide(frame)
        self.assertEqual(command.action, "TURN_LEFT")
        self.assertGreater(command.angular, 0)

    def test_stops_when_all_sectors_blocked(self) -> None:
        frame = [
            [0.3, 0.2, 0.3],
            [0.3, 0.2, 0.3],
        ]
        command = self.navigator.decide(frame)
        self.assertEqual(command.action, "STOP")
        self.assertEqual(command.linear, 0.0)
        self.assertEqual(command.angular, 0.0)

    def test_rejects_invalid_frame(self) -> None:
        with self.assertRaises(ValueError):
            self.navigator.decide([])


if __name__ == "__main__":
    unittest.main()
