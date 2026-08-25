import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from std_srvs.srv import Trigger


class TestHelper:
    def __init__(self, node: Node, rsMode=False) -> None:
        self.node = node
        prefix = "/oak"
        if rsMode:
            prefix="/camera/camera"
        self.stopSrv = self.node.create_client(Trigger, f"{prefix}/stop_driver")
        while not self.stopSrv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("service not available, waiting again...")
        self.startSrv = self.node.create_client(Trigger, f"{prefix}/start_driver")
        while not self.startSrv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("service not available, waiting again...")
        self.setParamSrv = self.node.create_client(SetParameters, f"{prefix}/set_parameters")
        while not self.setParamSrv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("service not available, waiting again...")
        self.getParamSrv = self.node.create_client(GetParameters, f"{prefix}/get_parameters")
        while not self.getParamSrv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("service not available, waiting again...")

    def setParameters(self, params: list[Parameter], restart: bool = True) -> bool:
        req = SetParameters.Request()
        req.parameters = params
        future = self.setParamSrv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info(str(future.result().results[0].reason))
        if future.result().results[0].successful:
            if restart:
                return self.restartDriver()
            else:
                return True
        else:
            return False
    def getParameter(self, param: str) -> ParameterValue:
        req = GetParameters.Request()
        req.names = [param]
        future = self.getParamSrv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().values[0]

    def restartDriver(self) -> bool:
        req = Trigger.Request()
        future = self.stopSrv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if not future.result().success:
            return False
        req = Trigger.Request()
        future = self.startSrv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if not future.result().success:
            return False
        return True

    def testIncomingMessages(
        self, msg_type, topic, callback=None, min_messages=30, timeout=5
    ):
        """
        Test incoming messages on a specified topic.

        Args:
            msg_type: The type of the message to subscribe to.
            topic: The topic to subscribe to.
            callback: A callback function to process each incoming message.
            min_messages: The minimum number of messages expected.
            timeout: The maximum time to wait for messages in seconds.
        """
        success = False
        messages_received = []
        sub = None
        if callback is None:
            sub = self.node.create_subscription(
                msg_type, topic, lambda msg: messages_received.append(msg), 10
            )
        else:
            sub = self.node.create_subscription(
                msg_type, topic, lambda msg: messages_received.append(callback(msg)), 10
            )
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=1)
            if len(messages_received) > min_messages:
                return True
        return False

    def testTriggerService(self, service_name, timeout=5)->bool:
        """
        Test a trigger service.

        Args:
            service_name: The name of the service to call.
            timeout: The maximum time to wait for the service response in seconds.
        """
        client = self.node.create_client(Trigger, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f"Service {service_name} not available, waiting again..."
            )
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if future.result() is not None:
            return future.result().success
        else:
            return False
