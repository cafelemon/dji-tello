"""ROS2 node that owns flight state and final Tello velocity commands."""

from __future__ import annotations

import time
from typing import Callable

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tello_interfaces.msg import FlightStatus, TelloTelemetry, TrackingStatus
from tello_interfaces.srv import TelloCommand

from .pid import Limits, PIDController
from .state_machine import FlightState, FlightStateMachine


class FlightManagerNode(Node):
    """Arbitrate vision commands using transport health and a safety state machine."""

    def __init__(self) -> None:
        super().__init__('tello_flight_manager')
        self.control_rate_hz = float(self.declare_parameter('control_rate_hz', 20.0).value)
        self.cmd_stale_s = float(self.declare_parameter('cmd_stale_s', 0.5).value)
        self.link_status_stale_s = float(self.declare_parameter('link_status_stale_s', 1.5).value)
        self.tracking_status_stale_s = float(
            self.declare_parameter('tracking_status_stale_s', 1.0).value
        )
        self.zero_before_land_cycles = int(
            self.declare_parameter('zero_before_land_cycles', 3).value
        )
        if self.zero_before_land_cycles < 1:
            raise ValueError('zero_before_land_cycles must be at least 1')
        self.minimum_takeoff_battery = float(
            self.declare_parameter('minimum_takeoff_battery', 20.0).value
        )
        self.vertical_control_mode = str(
            self.declare_parameter('vertical_control_mode', 'altitude_hold').value
        )
        if self.vertical_control_mode not in {'altitude_hold', 'visual_follow'}:
            raise ValueError('vertical_control_mode must be altitude_hold or visual_follow')

        reacquire_timeout_s = float(self.declare_parameter('reacquire_timeout_s', 2.0).value)
        lost_land_timeout_s = float(self.declare_parameter('lost_land_timeout_s', 5.0).value)
        self.machine = FlightStateMachine(reacquire_timeout_s, lost_land_timeout_s)

        target_altitude_cm = float(self.declare_parameter('target_altitude_cm', 180.0).value)
        self.altitude_pid = PIDController(
            float(self.declare_parameter('altitude_pid.kp', 0.005).value),
            float(self.declare_parameter('altitude_pid.ki', 0.0).value),
            float(self.declare_parameter('altitude_pid.kd', 0.001).value),
            setpoint=target_altitude_cm,
            integral_limits=Limits(
                float(self.declare_parameter('altitude_pid.integral_min', -100.0).value),
                float(self.declare_parameter('altitude_pid.integral_max', 100.0).value),
            ),
            output_limits=Limits(
                float(self.declare_parameter('altitude_pid.output_min', -1.0).value),
                float(self.declare_parameter('altitude_pid.output_max', 1.0).value),
            ),
            deadband=float(self.declare_parameter('altitude_pid.deadband_cm', 5.0).value),
            filter_alpha=float(self.declare_parameter('altitude_pid.filter_alpha', 0.3).value),
        )

        self.battery_percent = 0.0
        self.height_cm = 0.0
        self.last_telemetry_monotonic = 0.0
        self.last_link_monotonic = 0.0
        self.last_tracking_status_monotonic = 0.0
        self.last_tracking_cmd_monotonic = 0.0
        self.link_health = {'control_healthy': False, 'state_healthy': False, 'video_healthy': False}
        self.latest_tracking_cmd = TwistStamped()
        self.pending_commands: set[int] = set()
        self.pending_safety_land_reason: str | None = None
        self.safety_zero_cycles = 0

        self.cmd_pub = self.create_publisher(Twist, '/tello/cmd_vel', 1)
        self.status_pub = self.create_publisher(FlightStatus, '/flight/status', 10)
        self.telemetry_sub = self.create_subscription(
            TelloTelemetry, '/tello/telemetry', self._on_telemetry, qos_profile_sensor_data
        )
        self.link_sub = self.create_subscription(
            DiagnosticArray, '/tello/link_status', self._on_link_status, 10
        )
        self.tracking_status_sub = self.create_subscription(
            TrackingStatus, '/tracking/status', self._on_tracking_status, 10
        )
        self.tracking_cmd_sub = self.create_subscription(
            TwistStamped, '/tracking/cmd_vel', self._on_tracking_cmd, 1
        )
        self.transport_client = self.create_client(TelloCommand, '/tello/execute_command')

        self.connect_service = self.create_service(Trigger, '/flight/connect', self._connect)
        self.takeoff_service = self.create_service(Trigger, '/flight/takeoff', self._takeoff)
        self.land_service = self.create_service(Trigger, '/flight/land', self._land)
        self.emergency_service = self.create_service(
            Trigger, '/flight/emergency_stop', self._emergency
        )
        self.control_timer = self.create_timer(1.0 / max(1.0, self.control_rate_hz), self._tick)
        self.get_logger().info('Flight manager ready; automatic takeoff is disabled')

    def _on_telemetry(self, message: TelloTelemetry) -> None:
        self.battery_percent = float(message.battery_percent)
        self.height_cm = float(message.height_cm)
        self.last_telemetry_monotonic = time.monotonic()

    def _on_link_status(self, message: DiagnosticArray) -> None:
        if not message.status:
            return
        values = {item.key: item.value.lower() == 'true' for item in message.status[0].values}
        for key in self.link_health:
            self.link_health[key] = bool(values.get(key, False))
        self.last_link_monotonic = time.monotonic()

    def _on_tracking_status(self, message: TrackingStatus) -> None:
        now = time.monotonic()
        self.last_tracking_status_monotonic = now
        self.machine.update_tracking(bool(message.locked and message.visible), now)

    def _on_tracking_cmd(self, message: TwistStamped) -> None:
        self.latest_tracking_cmd = message
        self.last_tracking_cmd_monotonic = time.monotonic()

    def _command_name(self, command: int) -> str:
        return {
            TelloCommand.Request.CONNECT: 'connect',
            TelloCommand.Request.STREAM_ON: 'streamon',
            TelloCommand.Request.TAKEOFF: 'takeoff',
            TelloCommand.Request.LAND: 'land',
            TelloCommand.Request.EMERGENCY: 'emergency',
        }.get(command, f'command-{command}')

    def _send_transport_command(
        self,
        command: int,
        completed: Callable[[bool, str], None] | None = None,
    ) -> bool:
        if command in self.pending_commands:
            return False
        if not self.transport_client.wait_for_service(timeout_sec=0.0):
            if completed:
                completed(False, 'transport service unavailable')
            return False
        request = TelloCommand.Request()
        request.command = command
        self.pending_commands.add(command)
        future = self.transport_client.call_async(request)

        def done(result_future) -> None:
            self.pending_commands.discard(command)
            try:
                result = result_future.result()
                success = bool(result.success)
                reason = result.message or result.sdk_response
            except Exception as exc:  # rclpy propagates transport errors here
                success = False
                reason = str(exc)
            if completed:
                completed(success, reason)
            log = self.get_logger().info if success else self.get_logger().error
            log(f'{self._command_name(command)}: {reason}')

        future.add_done_callback(done)
        return True

    def _connect(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        accepted = self._send_transport_command(
            TelloCommand.Request.CONNECT,
            lambda success, _reason: self._send_transport_command(TelloCommand.Request.STREAM_ON)
            if success else None,
        )
        response.success = accepted
        response.message = 'connect request accepted' if accepted else 'connect request already pending or unavailable'
        return response

    def _takeoff(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        decision = self.machine.request_takeoff(
            self.battery_percent, self.minimum_takeoff_battery
        )
        if not decision.accepted:
            response.success = False
            response.message = decision.reason
            return response

        accepted = self._send_transport_command(
            TelloCommand.Request.TAKEOFF,
            lambda success, reason: self.machine.takeoff_result(success, reason),
        )
        if not accepted:
            self.machine.takeoff_result(False, 'transport service unavailable')
        response.success = accepted
        response.message = 'takeoff request accepted' if accepted else 'transport service unavailable'
        return response

    def _begin_land(self, reason: str) -> bool:
        decision = self.machine.request_land(reason)
        if not decision.accepted and self.machine.state != FlightState.LANDING:
            return False
        return self._send_transport_command(
            TelloCommand.Request.LAND,
            lambda success, message: self.machine.land_result(success, message),
        )

    def _land(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        accepted = self._begin_land('manual land requested')
        response.success = accepted
        response.message = 'land request accepted' if accepted else 'land request rejected or pending'
        return response

    def _emergency(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.cmd_pub.publish(Twist())
        self.machine.emergency()
        accepted = self._send_transport_command(
            TelloCommand.Request.EMERGENCY,
            lambda success, message: self.machine.land_result(success, message),
        )
        response.success = accepted
        response.message = 'emergency request accepted' if accepted else 'emergency request unavailable'
        return response

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self.last_link_monotonic > self.link_status_stale_s:
            links = (False, False, False)
        else:
            links = (
                self.link_health['control_healthy'],
                self.link_health['state_healthy'],
                self.link_health['video_healthy'],
            )
        self.machine.update_links(*links)

        if (
            self.machine.state in {FlightState.TRACKING, FlightState.LOST_TARGET}
            and now - self.last_tracking_status_monotonic > self.tracking_status_stale_s
        ):
            self.machine.update_tracking(False, now)

        action = self.machine.tick(now)
        if action == 'land' and self.pending_safety_land_reason is None:
            self.pending_safety_land_reason = self.machine.reason
            self.safety_zero_cycles = 0

        command = Twist()
        tracking_command_fresh = now - self.last_tracking_cmd_monotonic <= self.cmd_stale_s
        if self.machine.motion_allowed and tracking_command_fresh:
            suggested = self.latest_tracking_cmd.twist
            command.linear.x = suggested.linear.x
            command.linear.y = suggested.linear.y
            command.angular.z = suggested.angular.z
            if self.vertical_control_mode == 'visual_follow':
                command.linear.z = suggested.linear.z
            elif now - self.last_telemetry_monotonic <= self.link_status_stale_s:
                command.linear.z = self.altitude_pid.update(self.height_cm, now=now)
        else:
            self.altitude_pid.reset()
        self.cmd_pub.publish(command)

        if self.pending_safety_land_reason is not None:
            self.safety_zero_cycles += 1
            if self.safety_zero_cycles >= self.zero_before_land_cycles:
                reason = self.pending_safety_land_reason
                self.pending_safety_land_reason = None
                self.safety_zero_cycles = 0
                self._begin_land(reason)
        self._publish_status()

    def _publish_status(self) -> None:
        message = FlightStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = int(self.machine.state)
        message.state_name = self.machine.state.name
        message.reason = self.machine.reason
        message.airborne = self.machine.airborne
        self.status_pub.publish(message)

    def destroy_node(self) -> bool:
        for _ in range(3):
            self.cmd_pub.publish(Twist())
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
