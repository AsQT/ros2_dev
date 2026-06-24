#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace robot_hardware_interface
{

namespace cmd
{
inline constexpr uint8_t SERVO_ON_AXIS = 0xA0;
inline constexpr uint8_t GET_POS_AXIS = 0xA2;
inline constexpr uint8_t SERVO_HOME_AX = 0xA3;
inline constexpr uint8_t RUN_AXIS = 0xA5;
inline constexpr uint8_t JOG = 0xA7;

inline constexpr uint8_t SERVO_ON_ALL = 0xF1;
inline constexpr uint8_t STATUS_ALL = 0xF2;
inline constexpr uint8_t RUN_ALL = 0xF3;
inline constexpr uint8_t GET_POS_ALL = 0xF4;
}  // namespace cmd

inline constexpr uint16_t ROBOT_TCP_MAGIC = 0x55AA;
inline constexpr size_t ROBOT_TCP_HEADER_SIZE = 7;
inline constexpr uint8_t ROBOT_CMD_OK = 0x00;
inline constexpr size_t ROBOT_FLAGS_PUBLISH_AXIS_COUNT = 6;
inline constexpr size_t ROBOT_STATUS_FRAME_AXIS_COUNT = 8;
inline constexpr size_t ROBOT_STATUS_ALL_STATUS_BYTES = 1;
inline constexpr size_t ROBOT_STATUS_ALL_AXIS_BYTES = 12;
inline constexpr size_t ROBOT_STATUS_ALL_AXIS_PAYLOAD_SIZE =
  ROBOT_STATUS_FRAME_AXIS_COUNT * ROBOT_STATUS_ALL_AXIS_BYTES;
inline constexpr size_t ROBOT_STATUS_ALL_PAYLOAD_SIZE =
  ROBOT_STATUS_ALL_STATUS_BYTES + ROBOT_STATUS_ALL_AXIS_PAYLOAD_SIZE;

struct Frame
{
  uint8_t cmd{0};
  std::vector<uint8_t> payload;
};

class RobotTcpClient
{
public:
  RobotTcpClient() = default;
  ~RobotTcpClient();

  RobotTcpClient(const RobotTcpClient &) = delete;
  RobotTcpClient & operator=(const RobotTcpClient &) = delete;

  bool connect(const std::string & ip, int port, int timeout_ms = 2);
  void disconnect();
  bool is_connected() const;
  std::string last_error() const;

  bool send_frame(uint8_t cmd, const std::vector<uint8_t> & payload = {}, int timeout_ms = -1);
  Frame read_frame(std::optional<uint8_t> expected_cmd = std::nullopt, int timeout_ms = -1);

  std::optional<std::vector<uint8_t>> exchange(
    uint8_t cmd,
    const std::vector<uint8_t> & payload = {},
    bool wait_reply = true,
    std::optional<uint8_t> expected_cmd = std::nullopt,
    int timeout_ms = -1);

  void servo_on_axis(uint8_t axis_id);
  void servo_off_axis(uint8_t axis_id);
  void servo_home_axis(uint8_t axis_id);
  void servo_all(bool on);

  double get_pos_axis_deg(uint8_t axis_id, int timeout_ms = -1);

  void run_axis(uint8_t axis_id, double pos_deg, double vel_deg_s);
  void jog(uint8_t axis_id, bool direction_plus, double vel_deg_s);

  std::vector<uint32_t> status_all(int timeout_ms = -1);
  void run_all(const std::vector<double> & pos_deg, const std::vector<double> & vel_deg_s);
  std::pair<std::vector<double>, std::vector<double>> get_pos_all(int timeout_ms = -1);

  std::tuple<std::vector<double>, std::vector<double>, std::vector<uint32_t>> get_all_state(
    int timeout_ms = -1);
  static std::tuple<std::vector<double>, std::vector<double>, std::vector<uint32_t>>
  parse_status_all_payload(const std::vector<uint8_t> & payload);
  size_t last_state_payload_length() const;
  size_t last_state_payload_offset() const;
  size_t last_state_axis_bytes() const;

private:
  std::vector<uint8_t> build_frame_locked(uint8_t cmd, const std::vector<uint8_t> & payload) const;
  bool send_frame_locked(uint8_t cmd, const std::vector<uint8_t> & payload, int timeout_ms);
  Frame read_frame_locked(std::optional<uint8_t> expected_cmd, int timeout_ms);
  bool read_byte_locked(uint8_t & out, int timeout_ms);
  bool read_exact_locked(uint8_t * out, size_t len, int timeout_ms);
  bool wait_fd_locked(bool want_write, int timeout_ms) const;
  void drain_input_locked();
  void close_locked();

  static void append_u16_le(std::vector<uint8_t> & out, uint16_t v);
  static std::vector<uint8_t> pack_i32_le(int32_t v);
  static std::vector<uint8_t> pack_u32_le(uint32_t v);
  static int32_t unpack_i32_le(const uint8_t * p);
  static uint32_t unpack_u32_le(const uint8_t * p);
  static double clamp(double v, double lo, double hi);
  static int sanitize_timeout_ms(int timeout_ms);

  mutable std::mutex io_mtx_;
  int fd_{-1};
  int timeout_ms_{2};
  uint16_t seq_{0};
  size_t last_state_payload_length_{0};
  size_t last_state_payload_offset_{0};
  size_t last_state_axis_bytes_{0};
  std::string last_error_;
};

}  // namespace robot_hardware_interface
