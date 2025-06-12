import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_services_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor


class TestServiceClient(Node):
    """! Simple Publisher node, for testing purposes"""

    __test__ = False

    def __init__(
        self,
        service: str,
        srv_type: any,
        qos: QoSProfile = qos_profile_services_default,
    ):
        """!
        @param service (str) service name
        @param srv_type (any) service client
        """

        node_name = "srv_client" + service.replace("/", "_")
        super().__init__(node_name)
        self.service_name = service

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(
            srv_type,
            self.service_name,
            callback_group=self.callback_group,
            qos_profile=qos,
        )

        self.__response = None

    def call(self, request, timeout: float = 1.0, timeout_log: bool = True):
        """!
        @param request (ServiceRequest)
        @param timeout (float)
        """
        self.__response = None

        if not self.client.wait_for_service(timeout):
            if timeout_log:
                self.get_logger().error(
                    f"Service {self.service_name} was not found after {timeout} seconds"
                )
            return None
        # call service and spin executor for the timeout
        call_time = time.time()
        future_request = self.client.call_async(request)

        # so we can call services in constructor before calling rclpy.spin
        if self.executor is None:
            temp_executor = SingleThreadedExecutor()
            temp_executor.add_node(self)

        try:
            self.spin_until_complete(future_request, timeout)
        except Exception as e:
            self.get_logger().warn(
                f"Service call to {self.service_name} failed while spin. {str(e)}"
            )
            return None

        # Check if service responded
        if not future_request.done():
            if timeout_log:
                self.get_logger().error(
                    f"Service call to {self.service_name} timeouted after waiting for {time.time() - call_time} seconds"
                )
            return None

        # return response
        self.__response = future_request.result()
        return self.__response

    def spin_until_complete(self, future_request, timeout):
        """!
        Function to wait for the service execution accordingly
        depending on the type of executor to be used
        @param future_request the future to spin
        @param timeout the max time to wait for future completion
        """
        if isinstance(self.executor, SingleThreadedExecutor):
            self.executor.spin_until_future_complete(future_request, timeout)
            return

        # For some reason the spin until future complete function seems to be unstable
        # with multithreaded executors, so we need to fall back to this horrible logic
        request_time = time.time()
        while not future_request.done() and (time.time() - request_time < timeout):
            time.sleep(0.05)

    def get_response(self):
        return self.__response
