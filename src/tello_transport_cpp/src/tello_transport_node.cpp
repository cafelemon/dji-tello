#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <tello_interfaces/msg/tello_telemetry.hpp>
#include <tello_interfaces/srv/tello_command.hpp>

#include "tello_transport_cpp/transport_core.hpp"

using namespace std::chrono_literals;

namespace tello_transport_cpp
{
namespace
{
int64_t steady_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool fresh(int64_t stamp_ns, double timeout_s)
{
  if (stamp_ns <= 0) {
    return false;
  }
  const auto age_ns = steady_nanoseconds() - stamp_ns;
  return age_ns <= static_cast<int64_t>(timeout_s * 1e9);
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, bool value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value ? "true" : "false";
  return item;
}
}  // namespace

class TelloTransportNode final : public rclcpp::Node
{
public:
  TelloTransportNode()
  : Node("tello_transport"), running_(true)
  {
    drone_ip_ = declare_parameter<std::string>("drone_ip", "192.168.10.1");
    command_port_ = declare_parameter<int>("command_port", 8889);
    local_command_port_ = declare_parameter<int>("local_command_port", 9000);
    state_port_ = declare_parameter<int>("state_port", 8890);
    video_port_ = declare_parameter<int>("video_port", 11111);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 2.0);
    command_retries_ = declare_parameter<int>("command_retries", 3);
    rc_rate_hz_ = declare_parameter<double>("rc_rate_hz", 20.0);
    rc_limit_ = declare_parameter<int>("rc_limit", 100);
    cmd_stale_s_ = declare_parameter<double>("cmd_stale_s", 0.5);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
    video_timeout_s_ = declare_parameter<double>("video_timeout_s", 1.0);
    video_required_ = declare_parameter<bool>("video_required", true);
    auto_connect_ = declare_parameter<bool>("auto_connect", true);
    gstreamer_pipeline_ = declare_parameter<std::string>(
      "gstreamer_pipeline",
      "udpsrc port=" + std::to_string(video_port_) +
      " ! queue max-size-buffers=1 leaky=downstream ! h264parse ! avdec_h264 ! "
      "videoconvert ! appsink drop=true max-buffers=1 sync=false");

    telemetry_pub_ = create_publisher<tello_interfaces::msg::TelloTelemetry>(
      "/tello/telemetry", rclcpp::SensorDataQoS());
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/tello/image_raw", rclcpp::SensorDataQoS().keep_last(1));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/tello/link_status", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/tello/cmd_vel", rclcpp::QoS(1).best_effort(),
      std::bind(&TelloTransportNode::on_cmd_vel, this, std::placeholders::_1));
    command_service_ = create_service<tello_interfaces::srv::TelloCommand>(
      "/tello/execute_command",
      std::bind(
        &TelloTransportNode::on_command, this, std::placeholders::_1,
        std::placeholders::_2));

    open_sockets();
    state_thread_ = std::thread(&TelloTransportNode::state_loop, this);
    if (video_required_) {
      video_thread_ = std::thread(&TelloTransportNode::video_loop, this);
    }

    const auto rc_period = std::chrono::duration<double>(1.0 / std::max(1.0, rc_rate_hz_));
    rc_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(rc_period),
      std::bind(&TelloTransportNode::send_rc_tick, this));
    diagnostics_timer_ = create_wall_timer(500ms, std::bind(&TelloTransportNode::publish_diagnostics, this));
    reconnect_timer_ = create_wall_timer(2s, std::bind(&TelloTransportNode::reconnect_tick, this));

    if (!video_required_) {
      last_video_ns_.store(steady_nanoseconds());
    }
    RCLCPP_INFO(get_logger(), "Tello transport ready for %s:%d", drone_ip_.c_str(), command_port_);
  }

  ~TelloTransportNode() override
  {
    safe_shutdown();
  }

private:
  void open_sockets()
  {
    command_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    state_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (command_socket_ < 0 || state_socket_ < 0) {
      throw std::runtime_error("failed to create UDP sockets");
    }

    int reuse = 1;
    setsockopt(command_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(state_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in local_command{};
    local_command.sin_family = AF_INET;
    local_command.sin_addr.s_addr = htonl(INADDR_ANY);
    local_command.sin_port = htons(static_cast<uint16_t>(local_command_port_));
    if (::bind(command_socket_, reinterpret_cast<sockaddr *>(&local_command), sizeof(local_command)) < 0) {
      throw std::runtime_error("failed to bind command response socket");
    }

    sockaddr_in local_state{};
    local_state.sin_family = AF_INET;
    local_state.sin_addr.s_addr = htonl(INADDR_ANY);
    local_state.sin_port = htons(static_cast<uint16_t>(state_port_));
    if (::bind(state_socket_, reinterpret_cast<sockaddr *>(&local_state), sizeof(local_state)) < 0) {
      throw std::runtime_error("failed to bind state socket");
    }

    std::memset(&drone_address_, 0, sizeof(drone_address_));
    drone_address_.sin_family = AF_INET;
    drone_address_.sin_port = htons(static_cast<uint16_t>(command_port_));
    if (inet_pton(AF_INET, drone_ip_.c_str(), &drone_address_.sin_addr) != 1) {
      throw std::runtime_error("invalid drone_ip parameter");
    }

    timeval receive_timeout{};
    receive_timeout.tv_sec = 0;
    receive_timeout.tv_usec = 200000;
    setsockopt(state_socket_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));
  }

  std::pair<bool, std::string> execute_sdk_command(const std::string & command)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    std::string last_response = "timeout";
    for (int attempt = 1; attempt <= std::max(1, command_retries_); ++attempt) {
      const auto sent = ::sendto(
        command_socket_, command.data(), command.size(), 0,
        reinterpret_cast<sockaddr *>(&drone_address_), sizeof(drone_address_));
      if (sent < 0) {
        last_response = std::strerror(errno);
        continue;
      }

      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(command_socket_, &read_set);
      timeval timeout{};
      timeout.tv_sec = static_cast<int>(command_timeout_s_);
      timeout.tv_usec = static_cast<int>((command_timeout_s_ - timeout.tv_sec) * 1e6);
      const auto ready = ::select(command_socket_ + 1, &read_set, nullptr, nullptr, &timeout);
      if (ready <= 0) {
        last_response = "timeout";
        continue;
      }

      char buffer[1024]{};
      const auto received = ::recvfrom(command_socket_, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
      if (received <= 0) {
        last_response = "empty response";
        continue;
      }
      last_response.assign(buffer, static_cast<std::size_t>(received));
      if (is_success_ack(last_response)) {
        control_healthy_.store(true);
        last_control_ack_ns_.store(steady_nanoseconds());
        return {true, trim_copy(last_response)};
      }
      if (last_response.rfind("error", 0) == 0) {
        break;
      }
    }
    control_healthy_.store(false);
    return {false, trim_copy(last_response)};
  }

  void send_without_ack(const std::string & command)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    ::sendto(
      command_socket_, command.data(), command.size(), 0,
      reinterpret_cast<sockaddr *>(&drone_address_), sizeof(drone_address_));
  }

  void on_command(
    const std::shared_ptr<tello_interfaces::srv::TelloCommand::Request> request,
    std::shared_ptr<tello_interfaces::srv::TelloCommand::Response> response)
  {
    std::string command;
    switch (request->command) {
      case tello_interfaces::srv::TelloCommand::Request::CONNECT:
        command = "command";
        break;
      case tello_interfaces::srv::TelloCommand::Request::STREAM_ON:
        command = "streamon";
        break;
      case tello_interfaces::srv::TelloCommand::Request::TAKEOFF:
        command = "takeoff";
        break;
      case tello_interfaces::srv::TelloCommand::Request::LAND:
        command = "land";
        break;
      case tello_interfaces::srv::TelloCommand::Request::EMERGENCY:
        command = "emergency";
        break;
      default:
        response->success = false;
        response->message = "unsupported command";
        return;
    }

    const auto [success, sdk_response] = execute_sdk_command(command);
    response->success = success;
    response->sdk_response = sdk_response;
    response->message = success ? command + " accepted" : command + " failed";
    if (success && command == "takeoff") {
      airborne_.store(true);
    } else if (success && (command == "land" || command == "emergency")) {
      airborne_.store(false);
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    current_lr_.store(static_cast<int>(message->linear.y * 100.0));
    current_fb_.store(static_cast<int>(message->linear.x * 100.0));
    current_ud_.store(static_cast<int>(message->linear.z * 100.0));
    current_yaw_.store(static_cast<int>(message->angular.z * 100.0));
    last_cmd_ns_.store(steady_nanoseconds());
  }

  void send_rc_tick()
  {
    const bool command_is_fresh = fresh(last_cmd_ns_.load(), cmd_stale_s_);
    const auto command = command_is_fresh ?
      format_rc_command(
      current_lr_.load(), current_fb_.load(), current_ud_.load(), current_yaw_.load(), rc_limit_) :
      format_rc_command(0, 0, 0, 0, rc_limit_);
    send_without_ack(command);
  }

  void reconnect_tick()
  {
    if (auto_connect_ && !control_healthy_.load() && running_.load()) {
      const auto [success, response] = execute_sdk_command("command");
      if (success) {
        RCLCPP_INFO(get_logger(), "SDK command mode connected: %s", response.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "SDK reconnect failed: %s", response.c_str());
      }
    }
  }

  void state_loop()
  {
    std::vector<char> buffer(2048);
    while (running_.load()) {
      const auto received = ::recvfrom(state_socket_, buffer.data(), buffer.size() - 1, 0, nullptr, nullptr);
      if (received <= 0) {
        continue;
      }
      const std::string payload(buffer.data(), static_cast<std::size_t>(received));
      const auto parsed = parse_telemetry(payload);
      if (!parsed) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Malformed telemetry ignored");
        continue;
      }
      last_state_ns_.store(steady_nanoseconds());
      tello_interfaces::msg::TelloTelemetry message;
      message.stamp = now();
      message.pitch_deg = parsed->pitch_deg;
      message.roll_deg = parsed->roll_deg;
      message.yaw_deg = parsed->yaw_deg;
      message.velocity_x_cm_s = parsed->velocity_x_cm_s;
      message.velocity_y_cm_s = parsed->velocity_y_cm_s;
      message.velocity_z_cm_s = parsed->velocity_z_cm_s;
      message.temperature_low_c = parsed->temperature_low_c;
      message.temperature_high_c = parsed->temperature_high_c;
      message.tof_cm = parsed->tof_cm;
      message.height_cm = parsed->height_cm;
      message.battery_percent = parsed->battery_percent;
      message.barometer_cm = parsed->barometer_cm;
      message.flight_time_s = parsed->flight_time_s;
      message.acceleration_x = parsed->acceleration_x;
      message.acceleration_y = parsed->acceleration_y;
      message.acceleration_z = parsed->acceleration_z;
      telemetry_pub_->publish(message);
    }
  }

  void video_loop()
  {
    while (running_.load()) {
      cv::VideoCapture capture(gstreamer_pipeline_, cv::CAP_GSTREAMER);
      if (!capture.isOpened()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unable to open GStreamer video pipeline");
        std::this_thread::sleep_for(1s);
        continue;
      }
      cv::Mat frame;
      while (running_.load() && capture.read(frame)) {
        if (frame.empty()) {
          continue;
        }
        last_video_ns_.store(steady_nanoseconds());
        auto message = cv_bridge::CvImage(
          std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        message->header.stamp = now();
        message->header.frame_id = "tello_camera";
        image_pub_->publish(*message);
      }
      capture.release();
      std::this_thread::sleep_for(500ms);
    }
  }

  void publish_diagnostics()
  {
    const bool state_healthy = fresh(last_state_ns_.load(), state_timeout_s_);
    const bool video_healthy = !video_required_ || fresh(last_video_ns_.load(), video_timeout_s_);
    const bool control_healthy = control_healthy_.load();

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "tello_transport";
    status.hardware_id = drone_ip_;
    status.level = control_healthy && state_healthy && video_healthy ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ?
      "all required links healthy" : "one or more required links unhealthy";
    status.values.push_back(key_value("control_healthy", control_healthy));
    status.values.push_back(key_value("state_healthy", state_healthy));
    status.values.push_back(key_value("video_healthy", video_healthy));
    status.values.push_back(key_value("airborne", airborne_.load()));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  void safe_shutdown()
  {
    if (!running_.exchange(false)) {
      return;
    }
    for (int i = 0; i < 3; ++i) {
      send_without_ack(format_rc_command(0, 0, 0, 0, rc_limit_));
      std::this_thread::sleep_for(50ms);
    }
    if (airborne_.load()) {
      execute_sdk_command("land");
      airborne_.store(false);
    }
    if (state_socket_ >= 0) {
      ::shutdown(state_socket_, SHUT_RDWR);
    }
    if (state_thread_.joinable()) {
      state_thread_.join();
    }
    if (video_thread_.joinable()) {
      video_thread_.join();
    }
    if (command_socket_ >= 0) {
      ::close(command_socket_);
      command_socket_ = -1;
    }
    if (state_socket_ >= 0) {
      ::close(state_socket_);
      state_socket_ = -1;
    }
  }

  std::string drone_ip_;
  std::string gstreamer_pipeline_;
  int command_port_{8889};
  int local_command_port_{9000};
  int state_port_{8890};
  int video_port_{11111};
  int command_retries_{3};
  int rc_limit_{100};
  double command_timeout_s_{2.0};
  double rc_rate_hz_{20.0};
  double cmd_stale_s_{0.5};
  double state_timeout_s_{1.0};
  double video_timeout_s_{1.0};
  bool video_required_{true};
  bool auto_connect_{true};

  int command_socket_{-1};
  int state_socket_{-1};
  sockaddr_in drone_address_{};
  std::mutex command_mutex_;
  std::atomic<bool> running_;
  std::atomic<bool> control_healthy_{false};
  std::atomic<bool> airborne_{false};
  std::atomic<int> current_lr_{0};
  std::atomic<int> current_fb_{0};
  std::atomic<int> current_ud_{0};
  std::atomic<int> current_yaw_{0};
  std::atomic<int64_t> last_cmd_ns_{0};
  std::atomic<int64_t> last_control_ack_ns_{0};
  std::atomic<int64_t> last_state_ns_{0};
  std::atomic<int64_t> last_video_ns_{0};
  std::thread state_thread_;
  std::thread video_thread_;

  rclcpp::Publisher<tello_interfaces::msg::TelloTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<tello_interfaces::srv::TelloCommand>::SharedPtr command_service_;
  rclcpp::TimerBase::SharedPtr rc_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

}  // namespace tello_transport_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tello_transport_cpp::TelloTransportNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
