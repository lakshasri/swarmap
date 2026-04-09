import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, SetParametersResult
from std_srvs.srv import SetBool


class ParamServiceNode(Node):

    def __init__(self):
        super().__init__('param_service_node')
        self.declare_parameter('num_robots', 10)

        self._num_robots = self.get_parameter('num_robots').value

        self.create_service(
            SetParameters,
            '/swarm/set_param',
            self._handle_set_param,
        )

        self.create_service(
            SetBool,
            '/swarm/set_num_robots',
            self._handle_set_num_robots,
        )

        self.get_logger().info('ParamServiceNode ready')

    def _handle_set_param(
        self, request: SetParameters.Request, response: SetParameters.Response
    ) -> SetParameters.Response:
        for i in range(self._num_robots):
            robot_node_name = f'/robot_{i}/robot_node'
            client = self.create_client(SetParameters, f'{robot_node_name}/set_parameters')
            if client.service_is_ready():
                future = client.call_async(request)
                future.add_done_callback(
                    lambda f, rid=f'robot_{i}': self._log_result(f, rid)
                )
            else:
                self.get_logger().warning(f'{robot_node_name} param service not ready')

        for _ in request.parameters:
            result = SetParametersResult()
            result.successful = True
            response.results.append(result)
        return response

    def _log_result(self, future, robot_id: str):
        try:
            res = future.result()
            for r in res.results:
                if not r.successful:
                    self.get_logger().warning(
                        f'Param set failed for {robot_id}: {r.reason}'
                    )
        except Exception as e:
            self.get_logger().error(f'Param service call to {robot_id} raised: {e}')

    def _handle_set_num_robots(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        new_count = int(request.data)
        self.get_logger().info(f'set_num_robots called with data={request.data}')
        response.success = True
        response.message = f'num_robots target acknowledged (current={self._num_robots})'
        return response


def main():
    rclpy.init()
    node = ParamServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
