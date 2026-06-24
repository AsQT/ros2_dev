#include "robot_hardware_interface/tcp_client.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void append_i32_le(std::vector<uint8_t> & out, int32_t value)
{
  const auto v = static_cast<uint32_t>(value);
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

void append_u32_le(std::vector<uint8_t> & out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void append_u16_le(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

std::vector<uint8_t> make_new_payload()
{
  const int32_t positions[] = {1000, -1000, 0, 10, 30, 50, 70, 90};
  const uint32_t velocities[] = {2000, 3000, 0, 20, 40, 60, 80, 100};
  const uint32_t flags[] = {
    0x00100000,
    0x08000000,
    0x02000000,
    0x00200000,
    0x00000008 | 0x00000010,
    0x00010000 | 0x00000001,
    0x00000000,
    0x00000000,
  };

  std::vector<uint8_t> payload;
  payload.reserve(robot_hardware_interface::ROBOT_STATUS_ALL_PAYLOAD_SIZE);
  payload.push_back(robot_hardware_interface::ROBOT_CMD_OK);
  for (size_t axis = 0; axis < robot_hardware_interface::ROBOT_STATUS_FRAME_AXIS_COUNT; ++axis) {
    append_i32_le(payload, positions[axis]);
    append_u32_le(payload, velocities[axis]);
    append_u32_le(payload, flags[axis]);
  }
  return payload;
}

std::vector<uint8_t> make_legacy_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(60);
  for (size_t axis = 0; axis < robot_hardware_interface::ROBOT_FLAGS_PUBLISH_AXIS_COUNT; ++axis) {
    append_i32_le(payload, 0);
    append_u32_le(payload, 0);
    append_u16_le(payload, 0xFFFF);
  }
  return payload;
}

std::vector<uint8_t> make_six_axis_u32_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(72);
  for (size_t axis = 0; axis < robot_hardware_interface::ROBOT_FLAGS_PUBLISH_AXIS_COUNT; ++axis) {
    append_i32_le(payload, 0);
    append_u32_le(payload, 0);
    append_u32_le(payload, 0);
  }
  return payload;
}

std::vector<uint8_t> make_eight_axis_without_status_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(robot_hardware_interface::ROBOT_STATUS_ALL_AXIS_PAYLOAD_SIZE);
  for (size_t axis = 0; axis < robot_hardware_interface::ROBOT_STATUS_FRAME_AXIS_COUNT; ++axis) {
    append_i32_le(payload, 0);
    append_u32_le(payload, 0);
    append_u32_le(payload, 0);
  }
  return payload;
}

}  // namespace

int main()
{
  {
    const auto [pos_deg, vel_deg_s, flags] =
      robot_hardware_interface::RobotTcpClient::parse_status_all_payload(make_new_payload());

    assert(pos_deg.size() == robot_hardware_interface::ROBOT_STATUS_FRAME_AXIS_COUNT);
    assert(vel_deg_s.size() == robot_hardware_interface::ROBOT_STATUS_FRAME_AXIS_COUNT);
    assert(flags.size() == robot_hardware_interface::ROBOT_STATUS_FRAME_AXIS_COUNT);

    assert(pos_deg[0] == 1.0);
    assert(pos_deg[1] == -1.0);
    assert(std::fabs(pos_deg[6] - 0.07) < 1e-12);
    assert(std::fabs(pos_deg[7] - 0.09) < 1e-12);
    assert(vel_deg_s[0] == 2.0);
    assert(vel_deg_s[1] == 3.0);
    assert(std::fabs(vel_deg_s[6] - 0.08) < 1e-12);
    assert(std::fabs(vel_deg_s[7] - 0.1) < 1e-12);
    assert(flags[0] == 0x00100000u);
    assert(flags[1] == 0x08000000u);
    assert(flags[2] == 0x02000000u);
    assert(flags[3] == 0x00200000u);
    assert(flags[4] == 0x00000018u);
    assert(flags[5] == 0x00010001u);
    assert(flags[6] == 0x00000000u);
    assert(flags[7] == 0x00000000u);
  }

  {
    bool rejected = false;
    try {
      (void)robot_hardware_interface::RobotTcpClient::parse_status_all_payload(make_legacy_payload());
    } catch (const std::runtime_error & error) {
      rejected = std::string(error.what()).find("length mismatch") != std::string::npos;
    }
    assert(rejected);
  }

  {
    bool rejected = false;
    try {
      (void)robot_hardware_interface::RobotTcpClient::parse_status_all_payload(
        make_six_axis_u32_payload());
    } catch (const std::runtime_error & error) {
      rejected = std::string(error.what()).find("length mismatch") != std::string::npos;
    }
    assert(rejected);
  }

  {
    bool rejected = false;
    try {
      (void)robot_hardware_interface::RobotTcpClient::parse_status_all_payload(
        make_eight_axis_without_status_payload());
    } catch (const std::runtime_error & error) {
      rejected = std::string(error.what()).find("length mismatch") != std::string::npos;
    }
    assert(rejected);
  }

  {
    bool rejected = false;
    auto payload = make_new_payload();
    payload[0] = 0x01;
    try {
      (void)robot_hardware_interface::RobotTcpClient::parse_status_all_payload(payload);
    } catch (const std::runtime_error & error) {
      rejected = std::string(error.what()).find("status not OK") != std::string::npos;
    }
    assert(rejected);
  }

  std::cout << "tcp_get_all_parser_test OK\n";
  return 0;
}
