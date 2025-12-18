#include "mecanum_hardware/mecanum_system.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#define M_PI 3.14159265358979323846

namespace mecanum_hardware
{

// 1. INITIALIZATION
hardware_interface::CallbackReturn MecanumSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get Parameters from URDF
  port_name_ = info_.hardware_parameters["port"];

  // Default to 1980 ticks/rev (M5Stack 4EncoderMotor standard)
  ticks_per_rev_ = 1980.0;
  if (info_.hardware_parameters.count("ticks_per_rev") > 0) {
    ticks_per_rev_ = std::stod(info_.hardware_parameters["ticks_per_rev"]);
  }

  // Allocate memory for 4 wheels
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  prev_ticks_.resize(info_.joints.size(), 0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// 2. CONFIGURATION (Open Port)
hardware_interface::CallbackReturn MecanumSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    serial_port_.Open(port_name_);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);

    // Create a node helper to publish IMU data directly
    node_ = rclcpp::Node::make_shared("mecanum_hardware_node");
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Connected to PSoC on %s", port_name_.c_str());
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("MecanumSystem"), "Failed to open serial port %s", port_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 3. ACTIVATE (Reset)
hardware_interface::CallbackReturn MecanumSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Reset software values
  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
    prev_ticks_[i] = 0;
  }
  
  first_read_pass_ = true;
  prev_time_ = rclcpp::Clock().now(); 

  // Send START signal to PSoC (triggers 'usb_rx_task.c' to enable streams)
  try {
      serial_port_.Write("START\r\n");
      serial_port_.DrainWriteBuffer();
      // Give PSoC a moment to wake up
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Failed to send START command");
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Hardware system activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 4. READ LOOP (Robust Packet Handling)
hardware_interface::return_type MecanumSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // If no data is waiting, don't block
  if (!serial_port_.IsDataAvailable()) {
    return hardware_interface::return_type::OK;
  }

  // Process all available lines
  while (serial_port_.IsDataAvailable()) 
  {
      std::string line;
      try 
      {
          // TIMEOUT FIX: Increased to 10ms. 
          // The PSoC sends ~120 chars per line. At 230400 baud, that takes ~5ms.
          // 1ms was too short and was cutting packets in half.
          serial_port_.ReadLine(line, '\n', 20); 
          RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "RAW RX: %s", line.c_str());

          // Strip whitespace (CR/LF)
          while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == '\0')) {
              line.pop_back();
          }

          if (line.empty()) continue; 

          // --- SPY MODE: PRINT EVERYTHING ---
          // We want to see errors like "NAK:PARSE_ERROR"
          RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "RX: %s", line.c_str());
          
          // (Keep the filter only for the 'R' packet to avoid parsing errors below)
          if (line[0] == 'R') continue;

          // --- FIX FUSED PACKETS ---
          // Problem: PSoC sometimes sends "D,0.92...,R,32,32..." in one burst.
          // Solution: Detect the hidden 'R' and chop it off.
          size_t pos_R = line.find(",R,");
          if (pos_R != std::string::npos) {
              line.erase(pos_R); // Delete everything after the data
          }

          // --- DATA PARSING (D-Packet) ---
          if (line[0] == 'D' && line[1] == ',') {
              std::stringstream ss(line);
              std::string segment;
              std::vector<std::string> parts;

              while (std::getline(ss, segment, ',')) {
                  parts.push_back(segment);
              }

              // PSoC sends 15 parts: D + 4 Quat + 3 Accel + 3 Gyro + 4 Encoders
              if (parts.size() != 15) {
                  continue; // Skip malformed packets
              }

              // 1. IMU Publishing
              sensor_msgs::msg::Imu imu_msg;
              imu_msg.header.stamp = rclcpp::Clock().now();
              imu_msg.header.frame_id = "imu_link";

              try {
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
                  
                  imu_pub_->publish(imu_msg);
              } catch (...) { continue; } // Ignore stod errors

              // 2. Encoder Processing
              rclcpp::Time current_time = rclcpp::Clock().now();

              static const int enc_sign[4] = { +1, -1, +1, -1 };

if (!first_read_pass_) 
{
    double dt_read = (current_time - prev_time_).seconds();
    if (dt_read > 0.0) 
    { 
        for (int i = 0; i < 4; i++) {
            try {
                long current_tick =
                    std::stol(parts[11 + i]) * enc_sign[i];

                long delta_ticks = current_tick - prev_ticks_[i];

                double delta_rad =
                    (delta_ticks / ticks_per_rev_) * (2.0 * M_PI);

                hw_velocities_[i] = delta_rad / dt_read;
                hw_positions_[i] += delta_rad;

                prev_ticks_[i] = current_tick;
            } catch (...) {}
        }
    }
} 
else 
{
    // 🔧 FIRST READ — MUST APPLY SAME SIGN CONVENTION
    for (int i = 0; i < 4; i++) {
        try {
            prev_ticks_[i] =
                std::stol(parts[11 + i]) * enc_sign[i];
        } catch (...) {}
    }
    first_read_pass_ = false;
}


              prev_time_ = current_time;
          } 
      } 
      catch (...) {
          // Catch serial read errors to keep the node alive
      }
  } 

  return hardware_interface::return_type::OK;
}

// 5. WRITE LOOP (Throttled Control)
hardware_interface::return_type MecanumSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // --- FREQUENCY THROTTLE (20Hz) ---
  // The PSoC logs specific messages ("# MOTOR:...") on every command.
  // Sending at 100Hz floods the bus. We limit sending to 20Hz (every 50ms).
  // The PSoC has no watchdog, so this is safe.
  auto current_time = rclcpp::Clock().now();
  static auto last_write_time = current_time;
  
  if ((current_time - last_write_time).seconds() < 0.05) {
      return hardware_interface::return_type::OK; // Too soon, skip write
  }
  last_write_time = current_time;
  // ---------------------------------

  std::stringstream ss;
  ss << "M";
  
  // --- VELOCITY CALCULATION ---
  // CRITICAL: M5Stack driver calculates speed based on a 20ms loop (MOTOR_TASK_PERIOD_MS).
  // Even though we write every 50ms, the 'speed' value must be calculated
  // for a 20ms duration (dt = 0.02) or the robot will go 2.5x too fast.
  double dt_calc = 0.02; 

 for (int i = 0; i < 4; i++)
{
  double cmd_vel = hw_commands_[i];

  
  if (i == 1 || i == 3) {   
    cmd_vel = -cmd_vel;
  }

  double speed_point =
      (cmd_vel / (2.0 * M_PI)) * ticks_per_rev_ * dt_calc;

  int cmd = static_cast<int>(speed_point);

  if (cmd > 127)  cmd = 127;
  if (cmd < -127) cmd = -127;

  ss << "," << cmd;
}
  
  // Use \r\n to ensure PSoC 'usb_rx_task.c' detects the end of line properly
  ss << "\r\n"; 

  try {
      serial_port_.Write(ss.str());
      serial_port_.DrainWriteBuffer(); // Flush immediately
  } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("MecanumSystem"), "Serial Write Failed");
      return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// 6. EXPORT INTERFACES (CRITICAL - DO NOT DELETE)
std::vector<hardware_interface::StateInterface> MecanumSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

} // namespace mecanum_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumSystem, hardware_interface::SystemInterface)