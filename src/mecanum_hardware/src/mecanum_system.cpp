#include "mecanum_hardware/mecanum_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <sstream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

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
  
  // Default to 3960 ticks/rev if not set in URDF
  ticks_per_rev_ = 3960.0;
  if (info_.hardware_parameters.count("ticks_per_rev") > 0) {
      ticks_per_rev_ = std::stod(info_.hardware_parameters["ticks_per_rev"]);
  }

  // Allocate memory for 4 wheels
  // NOTE: Initializing to 0.0 instead of NaN to prevent controller crash on startup
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
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    
    // Create a node helper to publish IMU data
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
hardware_interface::CallbackReturn MecanumSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset values
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
    prev_ticks_[i] = 0;
  }
  first_read_pass_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 4. READ LOOP (Parse Data - 8 Value Version)
hardware_interface::return_type MecanumSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_port_.IsDataAvailable()) {
    try {
      std::string line;
      serial_port_.ReadLine(line);
      while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();

      if (line.rfind("START", 0) == 0) { 
          std::stringstream ss(line);
          std::string segment;
          std::vector<std::string> parts;
          while(std::getline(ss, segment, ',')) parts.push_back(segment);

          // CRITICAL CHANGE: Now we check for 10 parts (START + 8 Data + END)
          if (parts.size() >= 10 && parts.back() == "END") {
              
              // --- A. PARSE IMU (Indices 1-4: Quaternion only) ---
              auto imu_msg = sensor_msgs::msg::Imu();
              imu_msg.header.frame_id = "imu_link";
              imu_msg.header.stamp = rclcpp::Clock().now();
              
              imu_msg.orientation.w = std::stod(parts[1]);
              imu_msg.orientation.x = std::stod(parts[2]);
              imu_msg.orientation.y = std::stod(parts[3]);
              imu_msg.orientation.z = std::stod(parts[4]);
              
              // Set others to 0.0 (Missing Data)
              imu_msg.linear_acceleration.x = 0.0; imu_msg.linear_acceleration.y = 0.0; imu_msg.linear_acceleration.z = 0.0;
              imu_msg.angular_velocity.x = 0.0; imu_msg.angular_velocity.y = 0.0; imu_msg.angular_velocity.z = 0.0;
              
              // Set Covariance (Trust Orientation, Ignore others)
              imu_msg.orientation_covariance = {0.01,0,0, 0,0.01,0, 0,0,0.01};
              imu_msg.linear_acceleration_covariance = {0.0,0,0, 0,0.0,0, 0,0,0.0};
              imu_msg.angular_velocity_covariance = {0.0,0,0, 0,0.0,0, 0,0,0.0};
              
              imu_pub_->publish(imu_msg);

              // --- B. PARSE ENCODERS (Indices 5-8) ---
              // Note: Encoders moved from index 11 to index 5 because we deleted 6 IMU values
              rclcpp::Time current_time = rclcpp::Clock().now();
              
              if (!first_read_pass_) {
                  double dt = (current_time - prev_time_).seconds();
                  if (dt > 0.0) {
                      for (int i = 0; i < 4; i++) {
                          long current_tick = std::stol(parts[5 + i]); // Index is 5, 6, 7, 8
                          long delta_ticks = current_tick - prev_ticks_[i];
                          
                          double delta_rad = (delta_ticks / ticks_per_rev_) * (2.0 * M_PI);
                          hw_velocities_[i] = delta_rad / dt;
                          hw_positions_[i] += delta_rad; 
                          
                          prev_ticks_[i] = current_tick;
                      }
                  }
              } else {
                  for (int i = 0; i < 4; i++) prev_ticks_[i] = std::stol(parts[5 + i]);
                  first_read_pass_ = false;
              }
              prev_time_ = current_time;
          }
      } 
      else if (!line.empty()) {
          RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "[PSoC]: %s", line.c_str());
      }
    } catch (...) {}
  }
  return hardware_interface::return_type::OK;
}
// 5. WRITE LOOP (Send Commands)
hardware_interface::return_type MecanumSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert ROS Rad/s -> PSoC Speed Points (Ticks/10ms)
  std::stringstream ss;
  ss << "CMD";
  
  // CRITICAL FIX: Match the 100 Hz Controller Rate
  double dt = 0.01; // 10ms Loop
  
  for (int i = 0; i < 4; i++) {
      // Math: (Rad/s / 2PI) * TPR * dt
      double speed_point = (hw_commands_[i] / (2.0 * M_PI)) * ticks_per_rev_ * dt;
      
      // Clamp to int8 range (-127 to 127) for M5Stack register
      int cmd = static_cast<int>(speed_point);
      if (cmd > 127) cmd = 127;
      if (cmd < -127) cmd = -127;
      
      ss << "," << cmd;
  }
  ss << "\n";
  
  serial_port_.Write(ss.str());
  
  return hardware_interface::return_type::OK;
}

// 6. EXPORT INTERFACES
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

}  // namespace mecanum_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumSystem, hardware_interface::SystemInterface)