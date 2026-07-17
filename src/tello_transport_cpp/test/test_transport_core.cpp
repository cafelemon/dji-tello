#include <gtest/gtest.h>

#include <chrono>

#include "tello_transport_cpp/transport_core.hpp"

using namespace std::chrono_literals;

TEST(TransportCore, ParsesTelemetry)
{
  const auto value = tello_transport_cpp::parse_telemetry(
    "pitch:1;roll:-2;yaw:3;vgx:4;vgy:5;vgz:6;templ:30;temph:40;tof:50;"
    "h:180;bat:76;baro:1.23;time:9;agx:0.1;agy:0.2;agz:0.3;");
  ASSERT_TRUE(value.has_value());
  EXPECT_FLOAT_EQ(value->height_cm, 180.0F);
  EXPECT_FLOAT_EQ(value->battery_percent, 76.0F);
  EXPECT_FLOAT_EQ(value->barometer_cm, 123.0F);
}

TEST(TransportCore, RejectsMalformedTelemetry)
{
  EXPECT_FALSE(tello_transport_cpp::parse_telemetry("pitch:nope;h:10;bat:50;").has_value());
  EXPECT_FALSE(tello_transport_cpp::parse_telemetry("pitch:1;").has_value());
}

TEST(TransportCore, ClampsRcAndNormalizesAck)
{
  EXPECT_EQ(tello_transport_cpp::format_rc_command(120, -120, 20, 0, 100), "rc 100 -100 20 0");
  EXPECT_TRUE(tello_transport_cpp::is_success_ack(" OK\r\n"));
  EXPECT_FALSE(tello_transport_cpp::is_success_ack("error"));
}

TEST(TransportCore, RateGateUsesMonotonicIntervals)
{
  tello_transport_cpp::RateGate gate(100ms);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(gate.allow(start));
  EXPECT_FALSE(gate.allow(start + 99ms));
  EXPECT_TRUE(gate.allow(start + 100ms));
}

TEST(TransportCore, BuildsSoftwareVideoPipeline)
{
  const auto pipeline = tello_transport_cpp::build_gstreamer_pipeline(11111, "software");
  EXPECT_NE(pipeline.find("udpsrc port=11111"), std::string::npos);
  EXPECT_NE(pipeline.find("avdec_h264"), std::string::npos);
  EXPECT_EQ(pipeline.find("nvv4l2decoder"), std::string::npos);
}

TEST(TransportCore, BuildsJetsonVideoPipelineWithoutFallback)
{
  const auto pipeline = tello_transport_cpp::build_gstreamer_pipeline(11111, "nvv4l2");
  EXPECT_NE(pipeline.find("nvv4l2decoder"), std::string::npos);
  EXPECT_NE(pipeline.find("nvvidconv"), std::string::npos);
  EXPECT_EQ(pipeline.find("avdec_h264"), std::string::npos);
}

TEST(TransportCore, RejectsUnknownVideoDecoder)
{
  EXPECT_THROW(
    tello_transport_cpp::build_gstreamer_pipeline(11111, "auto"), std::invalid_argument);
  EXPECT_THROW(
    tello_transport_cpp::build_gstreamer_pipeline(0, "software"), std::invalid_argument);
}
