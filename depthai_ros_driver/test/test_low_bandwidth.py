import os
import unittest
import time
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import launch_testing.asserts
import pytest
import rclpy
import rclpy.node

from sensor_msgs.msg import Image, CameraInfo
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode


@pytest.mark.rostest
def generate_test_description():
    name = "oak"
    params = {"rgb": {"i_low_bandwidth": True}}

    driver = ComposableNodeContainer(
        name=f"{name}_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Driver",
                name=name,
                parameters=[
                    params,
                ],
            )
        ],
        output="both",
    )

    return launch.LaunchDescription(
        [
            driver,
            launch.actions.TimerAction(
                period=1.0, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    ), {"driver_node": driver}


class TestDriverLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test")

    def tearDown(self):
        self.node.destroy_node()

    def test_driver_output(self, proc_output):
        proc_output.assertWaitFor("Driver ready!", timeout=10.0, stream="stderr")

    def test_published_rgb_image(self, proc_output):
        images_received = []
        info_received = []
        sub = self.node.create_subscription(
            Image, "/oak/rgb/image_raw", lambda msg: images_received.append(msg), 10
        )
        sub_info = self.node.create_subscription(
            CameraInfo,
            "/oak/rgb/camera_info",
            lambda msg: info_received.append(msg),
            10,
        )
        try:
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if len(images_received) > 30 and len(info_received) > 30:
                    break
            self.assertGreater(len(images_received), 30)
            self.assertGreater(len(info_received), 30)
        finally:
            self.node.destroy_subscription(sub)

    def test_published_stereo_image(self, proc_output):
        images_received = []
        info_received = []
        sub = self.node.create_subscription(
            Image, "/oak/stereo/image_raw", lambda msg: images_received.append(msg), 10
        )
        sub_info = self.node.create_subscription(
            CameraInfo,
            "/oak/stereo/camera_info",
            lambda msg: info_received.append(msg),
            10,
        )
        try:
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if len(images_received) > 30 and len(info_received) > 30:
                    break
            self.assertGreater(len(images_received), 30)
            self.assertGreater(len(info_received), 30)
        finally:
            self.node.destroy_subscription(sub)
@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
