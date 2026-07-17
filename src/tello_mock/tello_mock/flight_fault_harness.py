"""ROS fault harness for deterministic FlightManager integration tests."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger
from tello_interfaces.msg import TelloTelemetry, TrackingStatus
from tello_interfaces.srv import TelloCommand


class FlightFaultHarness(Node):
    def __init__(self) -> None:
        super().__init__('flight_fault_harness')
        self.pause_telemetry = False
        self.pause_video = False
        self.pause_tracking = False
        self.airborne = False

        self.telemetry_pub = self.create_publisher(
            TelloTelemetry, '/tello/telemetry', qos_profile_sensor_data
        )
        self.link_pub = self.create_publisher(DiagnosticArray, '/tello/link_status', 10)
        self.tracking_status_pub = self.create_publisher(
            TrackingStatus, '/tracking/status', 10
        )
        self.tracking_cmd_pub = self.create_publisher(
            TwistStamped, '/tracking/cmd_vel', 1
        )
        self.command_service = self.create_service(
            TelloCommand, '/tello/execute_command', self._command
        )
        self.create_service(SetBool, '/mock/pause_telemetry', self._pause_telemetry)
        self.create_service(SetBool, '/mock/pause_video', self._pause_video)
        self.create_service(SetBool, '/mock/pause_tracking', self._pause_tracking)
        self.create_service(Trigger, '/mock/reset_faults', self._reset_faults)
        self.create_timer(0.05, self._publish_tracking)
        self.create_timer(0.1, self._publish_telemetry)
        self.create_timer(0.1, self._publish_links)

    @staticmethod
    def _set_flag(response, name: str, value: bool):
        response.success = True
        response.message = f'{name}={value}'
        return response

    def _pause_telemetry(self, request, response):
        self.pause_telemetry = bool(request.data)
        return self._set_flag(response, 'pause_telemetry', self.pause_telemetry)

    def _pause_video(self, request, response):
        self.pause_video = bool(request.data)
        return self._set_flag(response, 'pause_video', self.pause_video)

    def _pause_tracking(self, request, response):
        self.pause_tracking = bool(request.data)
        return self._set_flag(response, 'pause_tracking', self.pause_tracking)

    def _reset_faults(self, _request, response):
        self.pause_telemetry = False
        self.pause_video = False
        self.pause_tracking = False
        response.success = True
        response.message = 'faults cleared'
        return response

    def _command(self, request, response):
        if request.command == TelloCommand.Request.TAKEOFF:
            if self.airborne:
                response.success = False
                response.message = 'already airborne'
                return response
            self.airborne = True
        elif request.command in {TelloCommand.Request.LAND, TelloCommand.Request.EMERGENCY}:
            self.airborne = False
        response.success = True
        response.message = 'ok'
        response.sdk_response = 'ok'
        return response

    def _publish_telemetry(self) -> None:
        if self.pause_telemetry:
            return
        message = TelloTelemetry()
        message.stamp = self.get_clock().now().to_msg()
        message.battery_percent = 80.0
        message.height_cm = 180.0 if self.airborne else 0.0
        self.telemetry_pub.publish(message)

    def _publish_links(self) -> None:
        status = DiagnosticStatus()
        status.name = 'mock/links'
        status.hardware_id = 'flight-fault-harness'
        status.level = DiagnosticStatus.OK
        status.message = 'fault-injection link state'
        status.values = [
            KeyValue(key='control_healthy', value='true'),
            KeyValue(key='state_healthy', value=str(not self.pause_telemetry).lower()),
            KeyValue(key='video_healthy', value=str(not self.pause_video).lower()),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.link_pub.publish(message)

    def _publish_tracking(self) -> None:
        if self.pause_tracking:
            return
        now = self.get_clock().now().to_msg()
        status = TrackingStatus()
        status.stamp = now
        status.locked = True
        status.visible = True
        status.target_id = 1
        status.confidence = 0.9
        status.reason = 'mock target visible'
        self.tracking_status_pub.publish(status)
        command = TwistStamped()
        command.header.stamp = now
        command.twist.linear.x = 0.2
        command.twist.angular.z = 0.1
        self.tracking_cmd_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightFaultHarness()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
