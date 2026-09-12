"""Check report displays without starting Gazebo or commanding motion."""
import unittest
from types import SimpleNamespace
import numpy as np
from builtin_interfaces.msg import Time
from erc_solution.navigation_view import NavigationView
from erc_solution.robot import Robot


class NavigationViewTests(unittest.TestCase):
    def make_view(self):
        messages = {}
        def publisher(kind, topic, qos):
            messages[topic] = []
            return SimpleNamespace(publish=messages[topic].append)
        node = SimpleNamespace(create_publisher=publisher,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=10))))
        return NavigationView(node), messages

    def test_target_is_the_command_not_an_invented_plan(self):
        view, messages = self.make_view()
        view.target([2., 3.], np.pi)
        msg = messages['/erc/navigation/targets'][-1]
        self.assertEqual(msg.header.frame_id, 'arena')
        self.assertEqual(len(msg.poses), 1)
        self.assertEqual(msg.poses[0].position.x, 2.)
        self.assertAlmostEqual(msg.poses[0].orientation.z, 1.)
        self.assertEqual(messages['/erc/navigation/executed'], [])

    def test_path_and_scans_use_same_localized_frame(self):
        view, messages = self.make_view()
        view.update([2., 3., np.pi / 2], np.array([[1., 0.]]), 10.)
        path = messages['/erc/navigation/executed'][-1]
        self.assertEqual(path.header.frame_id, 'arena')
        self.assertEqual(path.poses[0].pose.position.y, 3.)
        scan = messages['/erc/navigation/scene'][-1].markers[1]
        self.assertAlmostEqual(scan.points[0].x, 2.)
        self.assertAlmostEqual(scan.points[0].y, 4.)
        view.update([2., 3., 0.], np.array([[1., 0.]]), 10.1)
        self.assertEqual(len(messages['/erc/navigation/executed']), 1)
        self.assertEqual(view.poses.maxlen, 4000)
        self.assertEqual(view.targets.maxlen, 100)

    def test_display_failure_does_not_abort_control(self):
        def broken(*args):
            raise RuntimeError('display failed')
        warnings = []
        robot = SimpleNamespace(navigation_view=SimpleNamespace(target=broken),
            get_logger=lambda: SimpleNamespace(warning=warnings.append))
        Robot.show_navigation(robot, 'target', [1., 2.], 0.)
        self.assertIsNone(robot.navigation_view)
        self.assertEqual(len(warnings), 1)


if __name__ == '__main__':
    unittest.main()
