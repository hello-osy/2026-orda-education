"""주행 도구가 같은 차량에 서로 다른 명령을 보내지 않도록 입력 모드를 선택한다."""
import argparse


def set_mode(node, mode, timeout=8):
    """이미 준비된 ROS 노드에서 주행 입력 모드를 바꾼다."""
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rcl_interfaces.srv import SetParameters
    if mode not in ('auto', 'manual'):
        raise ValueError('auto/manual 중 선택하세요.')
    client = node.create_client(SetParameters, '/session4_sim_control/set_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('시뮬레이터를 먼저 켜세요: bash scripts/session_4.sh sim')
        request = SetParameters.Request(parameters=[Parameter(name='mode', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=mode))])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('주행 모드 변경 응답이 없습니다.')
        if not all(result.successful for result in future.result().results):
            raise RuntimeError('주행 모드를 변경하지 못했습니다.')
    finally:
        node.destroy_client(client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['auto', 'manual'])
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=[])
    node = Node('session4_choose_control_mode')
    try:
        set_mode(node, args.mode)
        print('주행 모드: ' + ('직접 운전' if args.mode == 'manual' else '작성한 코드로 운전'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
