#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>

#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>

// Rs485Client is implemented in this package
#include "robot_hardware_interface/rs485_protocol.hpp"

namespace robot_hardware_interface
{

class RobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  // Tránh hide overload base on_init()
  using hardware_interface::SystemInterface::on_init;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool state_synced_{false};              // đã sync cmd = state sau activate chưa
  int warmup_cycles_{0};                  // đếm số chu kỳ bỏ qua write
  int warmup_cycles_cfg_{30};             // ví dụ bỏ qua 30 chu kỳ đầu (tùy bạn)

  double write_period_s_{0.02};           // 50Hz (0.02s). Dù update_rate cao vẫn chỉ gửi 50Hz
  double cmd_eps_rad_{1e-4};              // ngưỡng coi như command không đổi

  int consec_read_fail_{0};
  int max_consec_read_fail_{50};  // tuỳ bạn (ví dụ 50 vòng)

  rclcpp::Time last_write_time_{0,0,RCL_ROS_TIME};

  std::vector<double> last_sent_pos_;     // lưu cmd_pos_ lần cuối đã gửi xuống STM32


private:
  static std::vector<int> parse_csv_ints(const std::string & s);
  static std::vector<double> parse_csv_doubles(const std::string & s);

private:
  Rs485Client client_;
  bool connected_{false};

  std::string port_{"/dev/ttyUSB0"};
  int baudrate_{115200};
  double serial_timeout_s_{0.2};
  double pos_timeout_s_{0.2};
  double status_timeout_s_{0.2};
  int all_token_{255};
  double default_vel_deg_s_{30.0};

  std::vector<int> axis_ids_;
  std::vector<double> direction_sign_;
  std::vector<double> rad_offset_;

  std::vector<double> hw_pos_, hw_vel_;
  std::vector<double> cmd_pos_, cmd_vel_;
  //std::vector<double> hw_pos_, hw_vel_, hw_eff_;
  //std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;
};

}  // namespace robot_hardware_interface
