import os
import unittest
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import launch_testing.asserts
import pytest
import rclpy
import rclpy.node
import rclpy.parameter
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import (
    Parameter,
    ParameterDescriptor,
    ParameterType,
    ParameterValue,
)
import time
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.rostest
def generate_test_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    params_file = os.path.join(depthai_prefix, "config", "driver.yaml")
    rviz_config = os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz")

    driver_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, "launch", "driver.launch.py")
        ),
        launch_arguments={
            "name": "oak",
            "namespace": "",
            "parent_frame": "oak_parent_frame",
            "camera_model": "OAK-D-PRO",
            "cam_pos_x": "0.0",
            "cam_pos_y": "0.0",
            "cam_pos_z": "0.0",
            "cam_roll": "0.0",
            "cam_pitch": "0.0",
            "cam_yaw": "0.0",
            "params_file": params_file,
            "use_rviz": "false",
            "rviz_config": rviz_config,
            "rsp_use_composition": "true",
            "publish_tf_from_calibration": "true",
            "imu_from_descr": "false",
            "override_cam_model": "false",
            "use_gdb": "false",
            "use_valgrind": "false",
            "use_perf": "false",
            "rs_compat": "true",
            "pointcloud.enable": "false",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "depth_module.depth_profile": "640,400,30",
            "rgb_camera.color_profile": "640,400,30",
            "depth_module.infra_profile": "640,400,30",
        }.items(),
    )

    return launch.LaunchDescription(
        [
            driver_launch,
            launch.actions.TimerAction(
                period=1.0, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    ), {"driver_launch": driver_launch}


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
            Image, "/camera/camera/color/image_raw", lambda msg: images_received.append(msg), 10
        )
        sub_info = self.node.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
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
            Image, "/camera/camera/depth/image_rect_raw", lambda msg: images_received.append(msg), 10
        )
        sub_info = self.node.create_subscription(
            CameraInfo,
            "/camera/camera/depth/camera_info",
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

    def test_published_imu_messages(self, proc_output):
        imu_received = []
        sub = self.node.create_subscription(
            Imu, "/camera/camera/imu", lambda msg: imu_received.append(msg), 10
        )
        try:
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if len(imu_received) > 30:
                    break
            self.assertGreater(len(imu_received), 30)
        finally:
            self.node.destroy_subscription(sub)
@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
