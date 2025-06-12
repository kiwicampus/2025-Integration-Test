from rclpy.node import Node


class TestServiceServer(Node):
    """! Simple Publisher node, for testing purposes"""

    __test__ = False

    def __init__(self, service: str, srv_type: any):
        """!
        @param service (str) service name
        @param srv_type (any) service client
        """

        node_name = "srv_server" + service.replace("/", "_")
        super().__init__(node_name)

        self.__server = self.create_service(
            srv_type=srv_type, srv_name=service, callback=self.callback
        )
        self.__response = self.__server.srv_type.Response()
        self.__last_request = None

    def callback(self, request, response):
        """!
        @param request (SrvType.Request) Service request object
        @param response (SrvType.Response) Service response object
        """
        self.__last_request = request
        response = self.__response
        return response

    def set_response(self, response):
        self.__response = response

    def get_last_request(self):
        return self.__last_request
