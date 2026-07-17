#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>

namespace tello_transport_cpp
{

struct TelemetryValues
{
  float pitch_deg{0.0F};
  float roll_deg{0.0F};
  float yaw_deg{0.0F};
  float velocity_x_cm_s{0.0F};
  float velocity_y_cm_s{0.0F};
  float velocity_z_cm_s{0.0F};
  float temperature_low_c{0.0F};
  float temperature_high_c{0.0F};
  float tof_cm{0.0F};
  float height_cm{0.0F};
  float battery_percent{0.0F};
  float barometer_cm{0.0F};
  float flight_time_s{0.0F};
  float acceleration_x{0.0F};
  float acceleration_y{0.0F};
  float acceleration_z{0.0F};
};

std::optional<TelemetryValues> parse_telemetry(const std::string & payload);
std::string format_rc_command(int lr, int fb, int ud, int yaw, int limit = 100);
bool is_success_ack(const std::string & response);
std::string trim_copy(const std::string & value);

class RateGate
{
public:
  explicit RateGate(std::chrono::milliseconds minimum_interval);
  bool allow(std::chrono::steady_clock::time_point now);

private:
  std::chrono::milliseconds minimum_interval_;
  std::optional<std::chrono::steady_clock::time_point> last_allowed_;
};

}  // namespace tello_transport_cpp
