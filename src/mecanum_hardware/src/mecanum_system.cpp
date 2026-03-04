#include "mecanum_hardware/mecanum_system.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include <array>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mecanum_hardware
{
static constexpr std::array<int, 4> MOTOR_SIGN = { +1, -1, +1, -1 };

static constexpr double MIN_DT_FOR_VEL = 0.01;     // 10ms minimum for velocity calc
static constexpr double MAX_VEL_RAD_S  = 50.0;     // clamp only
static constexpr int64_t ROLLOVER_32   = 4294967296LL;
static constexpr int64_t ROLLOVER_THRESH = 1000000000LL;

hardware_interface::CallbackReturn MecanumSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 4) {
    RCLCPP_FATAL(rclcpp::get_logger("MecanumSystem"), "Expected 4 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters["port"];

  ticks_per_rev_ = 1980.0;
  if (info_.hardware_parameters.count("ticks_per_rev") > 0) {
    ticks_per_rev_ = std::stod(info_.hardware_parameters["ticks_per_rev"]);
  }

  hw_positions_.assign(4, 0.0);
  hw_velocities_.assign(4, 0.0);
  hw_commands_.assign(4, 0.0);
  prev_ticks_.assign(4, 0);

  RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"),
              "Initialized: port=%s, ticks_per_rev=%.1f, signs=[%d,%d,%d,%d]",
              port_name_.c_str(), ticks_per_rev_,
              MOTOR_SIGN[0], MOTOR_SIGN[1], MOTOR_SIGN[2], MOTOR_SIGN[3]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    serial_port_.Open(port_name_);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    node_ = rclcpp::Node::make_shared("mecanum_hardware_node");
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    feedback_pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_feedback", 20);

    RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Connected to PSoC on %s", port_name_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("MecanumSystem"), "Serial configure failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  try {
    if (serial_port_.IsOpen()) {
      serial_port_.Write("M,0,0,0,0\r\n");
      serial_port_.DrainWriteBuffer();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      serial_port_.Close();
    }
  } catch (...) {}
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(prev_ticks_.begin(), prev_ticks_.end(), 0);

  first_read_pass_ = true;
  first_write_ = true;
  prev_time_ = rclcpp::Clock().now();

  try {
    while (serial_port_.IsDataAvailable()) {
      std::string discard;
      serial_port_.ReadLine(discard, '\n', 5);
    }
    serial_port_.Write("START\r\n");
    serial_port_.DrainWriteBuffer();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } catch (...) {}

  RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (int attempt = 0; attempt < 3; attempt++) {
    try {
      serial_port_.Write("M,0,0,0,0\r\n");
      serial_port_.DrainWriteBuffer();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    } catch (...) {}
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!serial_port_.IsDataAvailable()) {
    return hardware_interface::return_type::OK;
  }

  std::string latest_data_line;
  int packets_drained = 0;

  // Drain buffer, keep only latest D packet
  while (serial_port_.IsDataAvailable()) {
    std::string line;
    try {
      serial_port_.ReadLine(line, '\n', 20);
      packets_drained++;

      while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == '\0')) {
        line.pop_back();
      }
      if (line.empty()) continue;
      if (line[0] == 'R') continue;

      if (line.size() >= 2 && line[0] == 'D' && line[1] == ',') {
        size_t pos_R = line.find(",R,");
        if (pos_R != std::string::npos) line.erase(pos_R);
        latest_data_line = line;
      } else {
        std_msgs::msg::String fb;
        fb.data = line;
        feedback_pub_->publish(fb);
      }
    } catch (...) {
      break;
    }
  }

  // Draining 4–5 packets is NORMAL if PSoC sends ~100Hz and ros2_control reads 25Hz.
  if (packets_drained > 15) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MecanumSystem"),
                         *node_->get_clock(), 5000,
                         "Buffer backlog: %d packets drained (CPU/serial issue?)", packets_drained);
  }

  if (latest_data_line.empty()) {
    return hardware_interface::return_type::OK;
  }

  // Parse CSV
  std::stringstream ss(latest_data_line);
  std::string segment;
  std::vector<std::string> parts;
  parts.reserve(15);
  while (std::getline(ss, segment, ',')) {
    parts.push_back(segment);
  }
  if (parts.size() != 15) {
    return hardware_interface::return_type::OK;
  }

  // Publish IMU
  try {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.orientation.w = std::stod(parts[1]);
    imu_msg.orientation.x = std::stod(parts[2]);
    imu_msg.orientation.y = std::stod(parts[3]);
    imu_msg.orientation.z = std::stod(parts[4]);

    imu_msg.linear_acceleration.x = std::stod(parts[5]);
    imu_msg.linear_acceleration.y = std::stod(parts[6]);
    imu_msg.linear_acceleration.z = std::stod(parts[7]);

    imu_msg.angular_velocity.x = std::stod(parts[8]);
    imu_msg.angular_velocity.y = std::stod(parts[9]);
    imu_msg.angular_velocity.z = std::stod(parts[10]);

    imu_msg.orientation_covariance[0] = 0.0012;
    imu_msg.orientation_covariance[4] = 0.0012;
    imu_msg.orientation_covariance[8] = 0.0012;

    imu_msg.angular_velocity_covariance[0] = 0.0003;
    imu_msg.angular_velocity_covariance[4] = 0.0003;
    imu_msg.angular_velocity_covariance[8] = 0.0003;

    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    imu_pub_->publish(imu_msg);
  } catch (...) {}

  // Encoder integration (IMPORTANT)
  rclcpp::Time current_time = rclcpp::Clock().now();
  const double dt_read = (current_time - prev_time_).seconds();
  const double dt_vel = std::max(dt_read, MIN_DT_FOR_VEL);  // only for velocity

  if (!first_read_pass_) {
    for (size_t i = 0; i < 4; i++) {
      try {
        int64_t current_tick = std::stoll(parts[11 + i]) * MOTOR_SIGN[i];
        int64_t delta_ticks = current_tick - prev_ticks_[i];

        // rollover protection
        if (delta_ticks > ROLLOVER_THRESH) delta_ticks -= ROLLOVER_32;
        else if (delta_ticks < -ROLLOVER_THRESH) delta_ticks += ROLLOVER_32;

        // integrate position ALWAYS
        const double delta_rad = (static_cast<double>(delta_ticks) / ticks_per_rev_) * (2.0 * M_PI);
        hw_positions_[i] += delta_rad;

        // compute velocity (clamp only)
        double velocity = delta_rad / dt_vel;
        if (std::abs(velocity) > MAX_VEL_RAD_S) {
          velocity = std::copysign(MAX_VEL_RAD_S, velocity);
        }
        hw_velocities_[i] = velocity;

        // ALWAYS update prev_ticks baseline
        prev_ticks_[i] = current_tick;

      } catch (...) {
        // If parsing fails, do not change prev_ticks_ (best effort)
      }
    }
  } else {
    // First valid read: seed prev_ticks_
    for (size_t i = 0; i < 4; i++) {
      try {
        prev_ticks_[i] = std::stoll(parts[11 + i]) * MOTOR_SIGN[i];
      } catch (...) {
        prev_ticks_[i] = 0;
      }
    }
    first_read_pass_ = false;
  }

  prev_time_ = current_time;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  rclcpp::Time now = rclcpp::Clock().now();

  // keep ~25Hz writes; do not overskip
  if (first_write_) {
    last_write_time_ = now;
    first_write_ = false;
  } else {
    if ((now - last_write_time_).seconds() < 0.035) {
      return hardware_interface::return_type::OK;
    }
    last_write_time_ = now;
  }

  constexpr double dt_calc = 0.02; // motor expects ticks per 20ms
  int cmd_values[4];

  for (size_t i = 0; i < 4; i++) {
    double cmd_vel = hw_commands_[i];
    if (!std::isfinite(cmd_vel)) cmd_vel = 0.0;

    cmd_vel *= MOTOR_SIGN[i];

    double speed_point = (cmd_vel / (2.0 * M_PI)) * ticks_per_rev_ * dt_calc;
    int cmd = static_cast<int>(std::round(speed_point));
    if (cmd > 127) cmd = 127;
    if (cmd < -127) cmd = -127;

    cmd_values[i] = cmd;
  }

  char cmd_buffer[64];
  int len = snprintf(cmd_buffer, sizeof(cmd_buffer),
                     "M,%d,%d,%d,%d\r\n",
                     cmd_values[0], cmd_values[1], cmd_values[2], cmd_values[3]);
  if (len <= 0 || len >= static_cast<int>(sizeof(cmd_buffer))) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    serial_port_.Write(cmd_buffer);
    serial_port_.DrainWriteBuffer();
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MecanumSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

}  // namespace mecanum_hardware

PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumSystem, hardware_interface::SystemInterface)