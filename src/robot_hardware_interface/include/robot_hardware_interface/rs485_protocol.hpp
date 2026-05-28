#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <mutex>

namespace robot_hardware_interface {

  namespace cmd {
  inline constexpr uint8_t SERVO_ON_AXIS  = 0xA0;
  inline constexpr uint8_t GET_POS_AXIS   = 0xA2;
  inline constexpr uint8_t SERVO_HOME_AX  = 0xA3;
  inline constexpr uint8_t RUN_AXIS       = 0xA5;
  inline constexpr uint8_t JOG            = 0xA7;

  inline constexpr uint8_t SERVO_ON_ALL   = 0xF1;
  inline constexpr uint8_t STATUS_ALL     = 0xF2;
  inline constexpr uint8_t RUN_ALL        = 0xF3;
  inline constexpr uint8_t GET_POS_ALL    = 0xF4; 
  } // namespace cmd

  struct ProtocolBytes {
    uint8_t header0{0xAA};
    uint8_t header1{0xBB};
    uint8_t tail0{0xAA};
    uint8_t tail1{0xFF};
    uint8_t stuff{0xAA}; };

  struct Frame {
    uint8_t cmd{0};
    std::vector<uint8_t> payload; };

  uint16_t crc16_modbus(const uint8_t* data, size_t len);

  std::vector<uint8_t> stuff_bytes(const std::vector<uint8_t>& raw, uint8_t stuff_byte = 0xAA);
  std::vector<uint8_t> destuff_bytes(const std::vector<uint8_t>& stuffed, uint8_t stuff_byte = 0xAA);
/*__________________________________________________________________________________*/
  class SerialPort {
  public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void open(const std::string& port, int baudrate, double timeout_s);
    void close();
    bool is_open() const;

    void write_all(const uint8_t* data, size_t len);
    bool read_byte(uint8_t& out);

    void flush_input();
    void set_timeout(double timeout_s);

  private:
    int fd_{-1};
    double timeout_s_{0.2};
  };
/*__________________________________________________________________________________*/
  class Rs485Client {
  public:
    Rs485Client() = default;

    void set_protocol(const ProtocolBytes & p);
    ProtocolBytes protocol() const;

    void connect(const std::string& port, int baudrate = 115200, double timeout_s = 0.2);
    void disconnect();
    bool is_connected() const;

    std::optional<std::vector<uint8_t>> exchange(
                  uint8_t cmd,
                  const std::vector<uint8_t>& payload = {},
                  bool wait_reply = true,
                  std::optional<uint8_t> expected_cmd = std::nullopt,
                  double timeout_s = 0.5);

    void servo_on_axis(uint8_t axis_id);
    void servo_off_axis(uint8_t axis_id);
    void servo_home_axis(uint8_t axis_id);
    void servo_all(bool on);

    double get_pos_axis_deg(uint8_t axis_id, double timeout_s = 0.35);

    void run_axis(uint8_t axis_id, double pos_deg, double vel_deg_s);
    void jog(uint8_t axis_id, bool direction_plus, double vel_deg_s);

    std::vector<uint32_t> status_all(double timeout_s = 0.5);
    void run_all(const std::vector<double>& pos6_deg, const std::vector<double>& vel6_deg_s);
    std::pair<std::vector<double>, std::vector<double>> get_pos_all(double timeout_s);

    std::tuple<
                std::vector<double>, 
                std::vector<double>, 
                std::vector<uint16_t>
            > get_all_state(double timeout_s);

  private:
    std::vector<uint8_t> build_frame(uint8_t cmd, const std::vector<uint8_t>& payload);
    Frame recv_frame(std::optional<uint8_t> expected_cmd, double timeout_s);

    static std::vector<uint8_t> pack_i32_le(int32_t v);
    static std::vector<uint8_t> pack_u32_le(uint32_t v);
    static int32_t unpack_i32_le(const uint8_t* p);
    static uint32_t unpack_u32_le(const uint8_t* p);
    static uint16_t unpack_u16_le(const uint8_t* p);
    static double clamp(double v, double lo, double hi);

    mutable std::mutex io_mtx_;
    SerialPort sp_;

    ProtocolBytes proto_{};
  };

}  // namespace robot_hardware_interface
