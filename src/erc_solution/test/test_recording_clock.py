"""The bottom timer must follow /clock rather than the host clock."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import rclpy
from python_qt_binding import QtWidgets
from rosgraph_msgs.msg import Clock
from erc_solution.recording_view import RecordingView


class RecordingClockTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch('erc_solution.recording_view.XWindows', side_effect=RuntimeError('offscreen test')):
            self.view = RecordingView()
        self.view.poller.stop()

    def tearDown(self):
        self.view.close()

    def tick(self, seconds):
        msg = Clock()
        msg.clock.sec = int(seconds)
        msg.clock.nanosec = round((seconds - int(seconds)) * 1e9)
        self.view.on_clock(msg)
        self.view.update_sim_clock()

    def test_slow_clock_and_pause(self):
        self.tick(20)
        self.view.started, self.view.sim_start = 1, 20
        self.tick(20.2)
        self.assertIn('00:00.2', self.view.sim_clock.text())
        self.tick(20.2)
        self.assertIn('00:00.2', self.view.sim_clock.text())
        self.tick(22.7)
        self.assertIn('00:02.7', self.view.sim_clock.text())

    def test_finish_freezes_sim_timer(self):
        self.view.started, self.view.sim_start = 1, 10
        self.tick(15)
        self.view.state(SimpleNamespace(data='SUCCESS: held book contacted collection bin'))
        self.tick(30)
        self.assertIn('00:05.0', self.view.sim_clock.text())

    def test_missing_clock_and_reset(self):
        self.view.update_sim_clock()
        self.assertIn('waiting', self.view.sim_clock.text())
        self.view.started = 1
        self.tick(20)
        self.tick(0)
        self.assertIn('reset', self.view.sim_clock.text())
