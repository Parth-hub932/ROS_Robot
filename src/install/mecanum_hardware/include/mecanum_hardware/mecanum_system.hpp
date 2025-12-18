#ifndef MECANUM_HARDWARE_MECANUM_SYSTEM_HPP
#define MECANUM_HARDWARE_MECANUM_SYSTEM_HPP

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

// LibSerial for UART
#include <libserial/SerialPort.h>

// IMU Message
#include "sensor_msgs/msg/imu.hpp"

namespace mecanum_hardware
{

class MecanumSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumSystem)

  // Lifecycle Methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Read / Write Loop
  hardware_interface::return_type read(const rclcpp::Time & time,
                                       const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time,
                                        const rclcpp::Duration & period) override;

  // ROS2 Control Interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // =========================
  // SERIAL INTERFACE
  // =========================
  LibSerial::SerialPort serial_port_;
  std::string port_name_;
  double ticks_per_rev_;

  // =========================
  // ROS2 CONTROL STORAGE
  // =========================
  std::vector<double> hw_commands_;    // [rad/s] from ROS controllers
  std::vector<double> hw_velocities_;  // [rad/s] feedback from encoders
  std::vector<double> hw_positions_;   // [rad] integrated encoder positions

  // =========================
  // ENCODER TIMING
  // =========================
  std::vector<long> prev_ticks_;
  rclcpp::Time prev_time_;
  bool first_read_pass_;

  // =========================
  // IMU
  // =========================
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<rclcpp::Node> node_;

  // =========================
  // ACK SAFETY VERIFICATION
  // =========================
  int last_tx_cmd_[4] = {0, 0, 0, 0};   // Last M,m1,m2,m3,m4 sent
  int last_rx_ack_[4] = {0, 0, 0, 0};   // Last R,m1,m2,m3,m4 received

  bool ack_received_ = false;          // True when valid ACK received
  rclcpp::Time last_tx_time_;          // For timeout watchdog
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE_MECANUM_SYSTEM_HPP