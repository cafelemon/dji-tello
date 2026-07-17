#include "tello_transport_cpp/transport_core.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace tello_transport_cpp
{
namespace
{
float value_or(const std::unordered_map<std::string, float> & values, const std::string & key)
{
  const auto found = values.find(key);
  return found == values.end() ? 0.0F : found->second;
}
}  // namespace

std::string trim_copy(const std::string & value)
{
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();
  return begin < end ? std::string(begin, end) : std::string{};
}

std::optional<TelemetryValues> parse_telemetry(const std::string & payload)
{
  std::unordered_map<std::string, float> values;
  std::stringstream fields(payload);
  std::string field;
  try {
    while (std::getline(fields, field, ';')) {
      const auto separator = field.find(':');
      if (separator == std::string::npos) {
        continue;
      }
      const auto key = trim_copy(field.substr(0, separator));
      const auto value = trim_copy(field.substr(separator + 1));
      if (!key.empty() && !value.empty()) {
        values[key] = std::stof(value);
      }
    }
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  } catch (const std::out_of_range &) {
    return std::nullopt;
  }

  if (values.find("bat") == values.end() || values.find("h") == values.end()) {
    return std::nullopt;
  }

  TelemetryValues result;
  result.pitch_deg = value_or(values, "pitch");
  result.roll_deg = value_or(values, "roll");
  result.yaw_deg = value_or(values, "yaw");
  result.velocity_x_cm_s = value_or(values, "vgx");
  result.velocity_y_cm_s = value_or(values, "vgy");
  result.velocity_z_cm_s = value_or(values, "vgz");
  result.temperature_low_c = value_or(values, "templ");
  result.temperature_high_c = value_or(values, "temph");
  result.tof_cm = value_or(values, "tof");
  result.height_cm = value_or(values, "h");
  result.battery_percent = value_or(values, "bat");
  result.barometer_cm = value_or(values, "baro") * 100.0F;
  result.flight_time_s = value_or(values, "time");
  result.acceleration_x = value_or(values, "agx");
  result.acceleration_y = value_or(values, "agy");
  result.acceleration_z = value_or(values, "agz");
  return result;
}

std::string format_rc_command(int lr, int fb, int ud, int yaw, int limit)
{
  const auto clamp_value = [limit](int value) {
      return std::clamp(value, -std::abs(limit), std::abs(limit));
    };
  std::ostringstream command;
  command << "rc " << clamp_value(lr) << ' ' << clamp_value(fb) << ' '
          << clamp_value(ud) << ' ' << clamp_value(yaw);
  return command.str();
}

bool is_success_ack(const std::string & response)
{
  auto normalized = trim_copy(response);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return normalized == "ok";
}

RateGate::RateGate(std::chrono::milliseconds minimum_interval)
: minimum_interval_(minimum_interval)
{
}

bool RateGate::allow(std::chrono::steady_clock::time_point now)
{
  if (!last_allowed_ || now - *last_allowed_ >= minimum_interval_) {
    last_allowed_ = now;
    return true;
  }
  return false;
}

}  // namespace tello_transport_cpp
