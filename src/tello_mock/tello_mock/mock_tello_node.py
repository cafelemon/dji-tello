"""ROS2-managed UDP Mock Tello with fault-injection services."""

from __future__ import annotations

import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from .protocol import MockTelloProtocol


class MockTelloNode(Node):
    def __init__(self) -> None:
        super().__init__('mock_tello')
        self.bind_ip = str(self.declare_parameter('bind_ip', '127.0.0.1').value)
        self.command_port = int(self.declare_parameter('command_port', 8889).value)
        self.state_target_ip = str(self.declare_parameter('state_target_ip', '127.0.0.1').value)
        self.state_target_port = int(self.declare_parameter('state_target_port', 8890).value)
        self.state_rate_hz = float(self.declare_parameter('state_rate_hz', 10.0).value)
        self.battery_percent = int(self.declare_parameter('battery_percent', 80).value)
        self.height_cm = int(self.declare_parameter('height_cm', 180).value)
        self.response_delay_s = float(self.declare_parameter('response_delay_s', 0.0).value)
        self.drop_ack = False
        self.error_ack = False
        self.pause_state = False
        self.running = True
        self.protocol = MockTelloProtocol()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.bind_ip, self.command_port))
        self.socket.settimeout(0.2)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.drop_ack_service = self.create_service(SetBool, '/mock/drop_ack', self._set_drop_ack)
        self.error_ack_service = self.create_service(SetBool, '/mock/error_ack', self._set_error_ack)
        self.pause_state_service = self.create_service(SetBool, '/mock/pause_state', self._set_pause_state)
        self.reset_service = self.create_service(Trigger, '/mock/reset_faults', self._reset_faults)
        self.command_thread = threading.Thread(target=self._command_loop, daemon=True)
        self.state_thread = threading.Thread(target=self._state_loop, daemon=True)
        self.command_thread.start()
        self.state_thread.start()
        self.get_logger().info(f'Mock Tello listening on {self.bind_ip}:{self.command_port}')

    def _set_drop_ack(self, request: SetBool.Request, response: SetBool.Response):
        self.drop_ack = bool(request.data)
        response.success = True
        response.message = f'drop_ack={self.drop_ack}'
        return response

    def _set_error_ack(self, request: SetBool.Request, response: SetBool.Response):
        self.error_ack = bool(request.data)
        response.success = True
        response.message = f'error_ack={self.error_ack}'
        return response

    def _set_pause_state(self, request: SetBool.Request, response: SetBool.Response):
        self.pause_state = bool(request.data)
        response.success = True
        response.message = f'pause_state={self.pause_state}'
        return response

    def _reset_faults(self, _request: Trigger.Request, response: Trigger.Response):
        self.drop_ack = False
        self.error_ack = False
        self.pause_state = False
        response.success = True
        response.message = 'faults cleared'
        return response

    def _command_loop(self) -> None:
        while self.running:
            try:
                payload, sender = self.socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            command = payload.decode('utf-8', errors='replace').strip()
            response = self.protocol.handle(command)
            if response is None or self.drop_ack:
                continue
            if self.response_delay_s > 0.0:
                time.sleep(self.response_delay_s)
            if self.error_ack:
                response = 'error injected'
            self.socket.sendto(response.encode('utf-8'), sender)

    def _state_loop(self) -> None:
        started = time.monotonic()
        period = 1.0 / max(1.0, self.state_rate_hz)
        while self.running:
            if not self.pause_state:
                flight_time = int(time.monotonic() - started) if self.protocol.airborne else 0
                height = self.height_cm if self.protocol.airborne else 0
                payload = (
                    f'pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:30;temph:32;'
                    f'tof:{height};h:{height};bat:{self.battery_percent};baro:0.0;'
                    f'time:{flight_time};agx:0.0;agy:0.0;agz:0.0;'
                )
                self.state_socket.sendto(
                    payload.encode('utf-8'), (self.state_target_ip, self.state_target_port)
                )
            time.sleep(period)

    def destroy_node(self) -> bool:
        self.running = False
        self.socket.close()
        self.state_socket.close()
        self.command_thread.join(timeout=1.0)
        self.state_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockTelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
