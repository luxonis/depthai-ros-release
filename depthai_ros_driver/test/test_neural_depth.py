import os
import unittest
import time
from functools import partial
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import launch_testing.asserts
import pytest
import rclpy
import rclpy.node

from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from rclpy.node import Node
from depthai_ros_driver.test_helper import TestHelper

IS_RVC4 = os.getenv("DEPTHAI_PLATFORM") == "rvc4"


@pytest.mark.rostest
@unittest.skipUnless(IS_RVC4, reason="Test not supported on RVC2")
def generate_test_description():
    name = "oak"
    params = {
        "pipeline_gen": {"i_pipeline_type": "DEPTH"},
        "stereo": {
            "i_use_neural_depth": True,
            "i_aligned": False,
        },
    }

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
        self.testHelper = TestHelper(self.node)

    def tearDown(self):
        self.node.destroy_node()

    @unittest.skipUnless(IS_RVC4, reason="Test not supported on RVC2")
    def test_driver_output(self, proc_output):
        proc_output.assertWaitFor("Driver ready!", timeout=10.0, stream="stderr")

    @unittest.skipUnless(IS_RVC4, reason="Test not supported on RVC2")
    def testSize(self, width=0, height=0, msg=Image()):
        if width == 0 or height == 0:
            return

        self.assertEqual(msg.width, width)
        self.assertEqual(msg.height, height)
        return msg

    @unittest.skipUnless(IS_RVC4, reason="Test not supported on RVC2")
    def test_published_stereo_image(self, proc_output):
        parameters = [
            Parameter(
                name="stereo.i_neural_depth_model",
                value=ParameterValue(type=4, string_value="NEURAL_DEPTH_LARGE"),
            )
        ]
        self.assertTrue(self.testHelper.setParameters(parameters))
        cb = partial(self.testSize, 768, 480)
        self.testHelper.testIncomingMessages(Image, "/oak/stereo/image_raw", cb)

    @unittest.skipUnless(IS_RVC4, reason="Test not supported on RVC2")
    def test_change_neural_depth_model(self, proc_output):
        parameters = [
            Parameter(
                name="stereo.i_neural_depth_model",
                value=ParameterValue(type=4, string_value="NEURAL_DEPTH_NANO"),
            )
        ]
        self.assertTrue(self.testHelper.setParameters(parameters))
        cb = partial(self.testSize, 384, 240)
        self.testHelper.testIncomingMessages(Image, "/oak/stereo/image_raw", cb)

    @unittest.skipUnless(IS_RVC4, reason="Test not supported on RVC2")
    def test_change_neural_depth_runtime_parameters(self, proc_output):
        parameters = [
            Parameter(
                name="stereo.r_edge_threshold",
                value=ParameterValue(type=2, integer_value=5),
            ),
            Parameter(
                name="stereo.r_confidence_threshold",
                value=ParameterValue(type=2, integer_value=100),
            )
        ]
        self.assertTrue(self.testHelper.setParameters(parameters, False))

        value = self.testHelper.getParameter("stereo.r_edge_threshold")
        self.assertAlmostEqual(value.integer_value, 5)
        value = self.testHelper.getParameter("stereo.r_confidence_threshold")
        self.assertAlmostEqual(value.integer_value, 100)



@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    @unittest.skipUnless(IS_RVC4, reason="Test not supported on RVC2")
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
