"""Optional recording UI: live RGB, wall timer and X11 Gazebo fullscreen.

This observer publishes no robot commands and does not affect scoring evidence.
"""
import ctypes as C
import ctypes.util
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from python_qt_binding import QtCore, QtGui, QtWidgets


def timer_path():
    return Path('/tmp') / ('erc_trial_start_' + os.environ.get('ROS_DOMAIN_ID', '0') + '.json')


class XWindows:
    """Request fullscreen through the window manager, without editing the world."""
    def __init__(self):
        self.lib = C.CDLL(ctypes.util.find_library('X11'))
        self.lib.XOpenDisplay.argtypes = [C.c_char_p]
        self.lib.XOpenDisplay.restype = C.c_void_p
        self.display = self.lib.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('X11 display unavailable')
        signatures = {
            'XDefaultRootWindow': ([C.c_void_p], C.c_ulong),
            'XInternAtom': ([C.c_void_p, C.c_char_p, C.c_int], C.c_ulong),
            'XQueryTree': ([C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.POINTER(C.c_ulong)), C.POINTER(C.c_uint)], C.c_int),
            'XFetchName': ([C.c_void_p, C.c_ulong, C.POINTER(C.c_void_p)], C.c_int),
            'XFree': ([C.c_void_p], C.c_int),
            'XSendEvent': ([C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.c_void_p], C.c_int),
            'XFlush': ([C.c_void_p], C.c_int),
            'XCloseDisplay': ([C.c_void_p], C.c_int),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, result
        self.root = self.lib.XDefaultRootWindow(self.display)

    def gazebo(self, window=None, depth=0):
        window = self.root if window is None else window
        name = C.c_void_p()
        if self.lib.XFetchName(self.display, window, C.byref(name)) and name.value:
            title = C.string_at(name).decode(errors='replace')
            self.lib.XFree(name)
            if title.lower() in ('gazebo', 'gazebo sim', 'ignition gazebo'):
                return window
        if depth >= 3:
            return None
        root, parent, count = C.c_ulong(), C.c_ulong(), C.c_uint()
        children = C.POINTER(C.c_ulong)()
        if self.lib.XQueryTree(self.display, window, C.byref(root), C.byref(parent), C.byref(children), C.byref(count)):
            ids = list(children[:count.value])
            if children:
                self.lib.XFree(children)
            for child in ids:
                found = self.gazebo(child, depth + 1)
                if found:
                    return found
        return None

    def fullscreen(self, window):
        class Message(C.Structure):
            _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
                        ('display', C.c_void_p), ('window', C.c_ulong), ('message_type', C.c_ulong),
                        ('format', C.c_int), ('data', C.c_long * 5)]
        # XEvent is a union padded to 24 longs.
        class Event(C.Union):
            _fields_ = [('message', Message), ('pad', C.c_long * 24)]
        event = Event()
        event.message = Message(33, 0, 1, self.display, window,
            self.lib.XInternAtom(self.display, b'_NET_WM_STATE', 0), 32,
            (C.c_long * 5)(1, self.lib.XInternAtom(self.display, b'_NET_WM_STATE_FULLSCREEN', 0), 0, 1, 0))
        self.lib.XSendEvent(self.display, self.root, 0, (1 << 20) | (1 << 19), C.byref(event))
        self.lib.XFlush(self.display)

    def close(self):
        self.lib.XCloseDisplay(self.display)


class RecordingView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.node = Node('erc_recording_view')
        self.bridge = CvBridge()
        self.opened = self.started = self.finished = None
        self.sim_now = self.sim_start = self.sim_finish = None
        self.sim_reset = False
        self.created = time.monotonic_ns()
        self.window_missing = 0
        self.status = 'Waiting for Gazebo'
        self.setWindowTitle('ERC camera and wall-clock timer')
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
        layout = QtWidgets.QVBoxLayout(self)
        self.clock = QtWidgets.QLabel(self.status)
        self.clock.setStyleSheet('font-size: 22px; font-weight: bold;')
        self.picture = QtWidgets.QLabel('Waiting for RGB camera')
        self.picture.setFixedSize(480, 270)
        layout.addWidget(self.clock)
        layout.addWidget(self.picture)
        self.sim_clock = QtWidgets.QLabel('Simulation time: waiting for /clock')
        self.sim_clock.setStyleSheet('font-size: 20px; font-weight: bold;')
        layout.addWidget(self.sim_clock)
        self.node.create_subscription(Image, '/head_front_camera/head_front_camera/color/image_raw', self.frame, qos_profile_sensor_data)
        self.node.create_subscription(String, '/erc/mission_status', self.state, 10)
        # Keep only the newest clock sample if rendering falls behind.
        self.node.create_subscription(Clock, '/clock', self.on_clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        try:
            self.windows = XWindows()
        except Exception as exc:
            self.windows = None
            self.node.get_logger().warning(f'Fullscreen unavailable: {exc}')
        self.poller = QtCore.QTimer(self)
        self.poller.timeout.connect(self.update_view)
        self.poller.start(100)

    def frame(self, message):
        rgb = self.bridge.imgmsg_to_cv2(message, 'rgb8')
        h, w = rgb.shape[:2]
        image = QtGui.QImage(rgb.data, w, h, rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        self.picture.setPixmap(QtGui.QPixmap.fromImage(image).scaled(480, 270, QtCore.Qt.KeepAspectRatio))

    def state(self, message):
        self.status = message.data
        if message.data.startswith(('SUCCESS:', 'FAILED:')) and self.started is not None:
            self.finished = time.monotonic_ns()
            self.sim_finish = self.sim_now

    def on_clock(self, message):
        """Use simulator ticks directly; a pause or slowdown needs no estimate."""
        value = message.clock.sec + message.clock.nanosec / 1e9
        if self.finished is None:
            if self.sim_now is not None and value < self.sim_now and self.started is not None:
                self.sim_reset = True
            if self.started is not None and self.sim_start is None:
                self.sim_start = value
        self.sim_now = value

    def update_sim_clock(self):
        if self.sim_now is None:
            self.sim_clock.setText('Simulation time: waiting for /clock')
            return
        if self.sim_reset:
            self.sim_clock.setText('Simulation clock reset — start a new trial')
            return
        if self.started is None:
            elapsed, label = self.sim_now, 'Simulation clock'
        elif self.sim_start is None:
            self.sim_clock.setText('Trial simulation time: waiting for /clock')
            return
        else:
            end = self.sim_finish if self.sim_finish is not None else self.sim_now
            elapsed, label = max(0., end - self.sim_start), 'Trial simulation time'
        minutes, seconds = divmod(elapsed, 60)
        self.sim_clock.setText(f'{label}: {int(minutes):02d}:{seconds:04.1f}')

    def update_view(self):
        """Refresh the camera and wall timer without blocking the UI."""
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.opened is None and self.windows:
            window = self.windows.gazebo()
            if window:
                self.windows.fullscreen(window)
                self.opened = time.monotonic_ns()
                self.show()
                self.raise_()
        elif self.opened is not None and self.windows:
            self.window_missing = 0 if self.windows.gazebo() else self.window_missing + 1
            if self.window_missing >= 20:
                self.close()
                return
        try:
            started = json.loads(timer_path().read_text())['monotonic_ns']
            # Ignore a leftover timestamp from a previous simulator session.
            if started >= self.created and started != self.started:
                self.started, self.finished = started, None
                self.sim_start, self.sim_finish = self.sim_now, None
                self.sim_reset = False
        except (OSError, ValueError, KeyError):
            pass
        self.update_sim_clock()
        if self.started is not None:
            elapsed = ((self.finished or time.monotonic_ns()) - self.started) / 1e9
            label = 'Trial wall time'
        elif self.opened is not None:
            elapsed = (time.monotonic_ns() - self.opened) / 1e9
            label = 'Gazebo open; awaiting solution'
        else:
            self.clock.setText('Waiting for Gazebo window')
            return
        minutes, seconds = divmod(elapsed, 60)
        self.clock.setText(f'{label}\n{int(minutes):02d}:{seconds:04.1f}\n{self.status}')

    def closeEvent(self, event):
        self.poller.stop()
        self.node.destroy_node()
        if self.windows:
            self.windows.close()
        event.accept()


def main():
    rclpy.init()
    app = QtWidgets.QApplication([])
    view = RecordingView()
    view.show()
    try:
        app.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
