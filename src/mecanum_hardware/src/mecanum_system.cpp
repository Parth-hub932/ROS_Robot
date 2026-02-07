#include "mecanum_hardware/mecanum_system.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include <array>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace mecanum_hardware
{
// ═══════════════════════════════════════════════════════════════════════════════
// SIGN CONFIGURATION (YOUR CORRECT VALUES)
// M1=FL(+), M2=FR(-), M3=RL(+), M4=RR(-)
// ═══════════════════════════════════════════════════════════════════════════════
static constexpr std::array<int, 4> MOTOR_SIGN = { +1, -1, +1, -1 }; // Used for BOTH encoder and command
// ═══════════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn MecanumSystem::on_init(const hardware_interface::HardwareInfo & info)
{
if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
{
return hardware_interface::CallbackReturn::ERROR;
}
if (info_.joints.size() != 4) {
RCLCPP_FATAL(rclcpp::get_logger("MecanumSystem"),
"Expected 4 joints, got %zu", info_.joints.size());
return hardware_interface::CallbackReturn::ERROR;
}
port_name_ = info_.hardware_parameters["port"];
ticks_per_rev_ = 1980.0;
if (info_.hardware_parameters.count("ticks_per_rev") > 0) {
ticks_per_rev_ = std::stod(info_.hardware_parameters["ticks_per_rev"]);
}
hw_positions_.resize(4, 0.0);
hw_velocities_.resize(4, 0.0);
hw_commands_.resize(4, 0.0);
prev_ticks_.resize(4, 0);
RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"),
"Initialized: port=%s, ticks_per_rev=%.1f, signs=[%d,%d,%d,%d]",
port_name_.c_str(), ticks_per_rev_,
MOTOR_SIGN[0], MOTOR_SIGN[1], MOTOR_SIGN[2], MOTOR_SIGN[3]);
return hardware_interface::CallbackReturn::SUCCESS;
}
// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURE - Open Serial Port
// ═══════════════════════════════════════════════════════════════════════════════
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
feedback_pub_ =node_->create_publisher<std_msgs::msg::String>("/robot_feedback", 20);

RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Connected to PSoC on %s", port_name_.c_str());
} catch (const LibSerial::OpenFailed& e) {
RCLCPP_FATAL(rclcpp::get_logger("MecanumSystem"), "Failed to open %s: %s", port_name_.c_str(), e.what());
return hardware_interface::CallbackReturn::ERROR;
} catch (const std::exception& e) {
RCLCPP_FATAL(rclcpp::get_logger("MecanumSystem"), "Serial config error: %s", e.what());
return hardware_interface::CallbackReturn::ERROR;
}
return hardware_interface::CallbackReturn::SUCCESS;
}
// ═══════════════════════════════════════════════════════════════════════════════
// CLEANUP - Close Serial Port
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn MecanumSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Cleaning up...");
try {
if (serial_port_.IsOpen()) {
serial_port_.Write("M,0,0,0,0\r\n");
serial_port_.DrainWriteBuffer();
std::this_thread::sleep_for(std::chrono::milliseconds(100));
serial_port_.Close();
RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Serial port closed.");
}
} catch (const std::exception& e) {
RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Cleanup warning: %s", e.what());
}
return hardware_interface::CallbackReturn::SUCCESS;
}
// ═══════════════════════════════════════════════════════════════════════════════
// ACTIVATE - Reset State & Start PSoC Stream
// ═══════════════════════════════════════════════════════════════════════════════
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
// Clear any stale data in serial buffer
while (serial_port_.IsDataAvailable()) {
std::string discard;
serial_port_.ReadLine(discard, '\n', 5);
}
serial_port_.Write("START\r\n");
serial_port_.DrainWriteBuffer();
std::this_thread::sleep_for(std::chrono::milliseconds(100));
} catch (const std::exception& e) {
RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Activation warning: %s", e.what());
}
RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Hardware activated.");
return hardware_interface::CallbackReturn::SUCCESS;
}
// ═══════════════════════════════════════════════════════════════════════════════
// DEACTIVATE - STOP MOTORS (Critical Safety!)
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn MecanumSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Deactivating - stopping motors...");
// Send stop command multiple times for reliability
for (int attempt = 0; attempt < 3; attempt++) {
try {
serial_port_.Write("M,0,0,0,0\r\n");
serial_port_.DrainWriteBuffer();
std::this_thread::sleep_for(std::chrono::milliseconds(30));
} catch (const std::exception& e) {
RCLCPP_ERROR(rclcpp::get_logger("MecanumSystem"),
"Stop command failed (attempt %d): %s", attempt + 1, e.what());
}
}
RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"), "Motors stopped.");
return hardware_interface::CallbackReturn::SUCCESS;
}
// ═══════════════════════════════════════════════════════════════════════════════
// READ - THE MAIN FIX: Process ONLY the latest packet
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::return_type MecanumSystem::read(
const rclcpp::Time & /*time*/,
const rclcpp::Duration & /*period*/)
{
if (!serial_port_.IsDataAvailable()) {
return hardware_interface::return_type::OK;
}
std::string latest_data_line;
int packets_drained = 0;
// ═══════════════════════════════════════════════════════════════════════════
// FIX: Drain ENTIRE buffer, keep ONLY the LAST valid D-packet
// This prevents velocity spikes from stale buffered packets
// ═══════════════════════════════════════════════════════════════════════════
while (serial_port_.IsDataAvailable())
{
  std::string line;
  try
  {
    serial_port_.ReadLine(line, '\n', 20);
    packets_drained++;

    // Strip trailing characters
    while (!line.empty() &&
          (line.back() == '\n' || line.back() == '\r' || line.back() == '\0')) {
      line.pop_back();
    }

    if (line.empty()) continue;

    // Skip R packets silently
    if (line[0] == 'R') {
      continue;
    }

    // VALID SENSOR PACKET
    if (line.size() >= 2 && line[0] == 'D' && line[1] == ',')
    {
      // Handle fused packets
      size_t pos_R = line.find(",R,");
      if (pos_R != std::string::npos) {
        line.erase(pos_R);
      }

      // Keep ONLY the latest D packet
      latest_data_line = line;
    }
    else
    {
      // 🚨 EVERYTHING ELSE → /robot_feedback
      std_msgs::msg::String feedback_msg;
      feedback_msg.data = line;
      feedback_pub_->publish(feedback_msg);
    }
  }
  catch (const LibSerial::ReadTimeout&) {
    break;
  }
  catch (const std::exception& e) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MecanumSystem"),
      "Serial read error: %s", e.what());
  }
}

// Warn if buffer is accumulating (indicates timing issues)
if (packets_drained > 3) {
RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MecanumSystem"),
*node_->get_clock(), 5000,
"Buffer backlog: %d packets drained", packets_drained);
}
if (latest_data_line.empty()) {
return hardware_interface::return_type::OK;
}
// ═══════════════════════════════════════════════════════════════════════════
// PARSE THE PACKET
// ═══════════════════════════════════════════════════════════════════════════
std::stringstream ss(latest_data_line);
std::string segment;
std::vector<std::string> parts;
parts.reserve(15);
while (std::getline(ss, segment, ',')) {
parts.push_back(segment);
}
if (parts.size() != 15) {
RCLCPP_DEBUG(rclcpp::get_logger("MecanumSystem"),
"Malformed packet: %zu parts (expected 15)", parts.size());
return hardware_interface::return_type::OK;
}
// ═══════════════════════════════════════════════════════════════════════════
// IMU PUBLISHING (with covariances for robot_localization)
// ═══════════════════════════════════════════════════════════════════════════
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
// Set covariances (BNO085 typical accuracy)
// Diagonal elements only (row-major 3x3 matrix)
imu_msg.orientation_covariance[0] = 0.0012; // ~2° accuracy
imu_msg.orientation_covariance[4] = 0.0012;
imu_msg.orientation_covariance[8] = 0.0012;
imu_msg.angular_velocity_covariance[0] = 0.0003; // ~1°/s accuracy
imu_msg.angular_velocity_covariance[4] = 0.0003;
imu_msg.angular_velocity_covariance[8] = 0.0003;
imu_msg.linear_acceleration_covariance[0] = 0.01; // ~0.1 m/s² accuracy
imu_msg.linear_acceleration_covariance[4] = 0.01;
imu_msg.linear_acceleration_covariance[8] = 0.01;
imu_pub_->publish(imu_msg);
} catch (const std::invalid_argument& e) {
RCLCPP_DEBUG(rclcpp::get_logger("MecanumSystem"), "IMU parse error: %s", e.what());
} catch (const std::out_of_range& e) {
RCLCPP_DEBUG(rclcpp::get_logger("MecanumSystem"), "IMU value overflow: %s", e.what());
}
// ═══════════════════════════════════════════════════════════════════════════
// ENCODER PROCESSING
// ═══════════════════════════════════════════════════════════════════════════
rclcpp::Time current_time = rclcpp::Clock().now();
if (!first_read_pass_)
{
double dt_read = (current_time - prev_time_).seconds();
// Sanity check dt - reject if unreasonable115200
if (dt_read < 0.01) { 
    // Less than 5ms - shouldnt happen with our fix
RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MecanumSystem"),
*node_->get_clock(), 1000,
"dt too small: %.3fms - skipping", dt_read * 1000);
return hardware_interface::return_type::OK;
}
if (dt_read > 1.25) { // More than 500ms - data gap
RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"),
"Large dt gap: %.1fms - possible data loss", dt_read * 1000);
// Continue processing but be aware
}
for (size_t i = 0; i < 4; i++)
{
try {
int64_t current_tick = std::stoll(parts[11 + i]) * MOTOR_SIGN[i];
int64_t delta_ticks = current_tick - prev_ticks_[i];
// Encoder overflow handling (32-bit counter)
constexpr int64_t ROLLOVER_THRESHOLD = 1000000000LL;
if (delta_ticks > ROLLOVER_THRESHOLD) {
delta_ticks -= 4294967296LL; // 2^32
} else if (delta_ticks < -ROLLOVER_THRESHOLD) {
delta_ticks += 4294967296LL;
}
double delta_rad = (static_cast<double>(delta_ticks) / ticks_per_rev_) * (2.0 * M_PI);
double velocity = delta_rad / dt_read;
// Velocity sanity check - reject spikes
constexpr double MAX_VELOCITY = 50.0; // rad/s, ~477 RPM
if (std::abs(velocity) > MAX_VELOCITY) {
RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MecanumSystem"),
*node_->get_clock(), 1000,
"Velocity spike rejected: wheel %zu = %.1f rad/s (max %.1f)",
i, velocity, MAX_VELOCITY);
// Don't update this wheel's state - keep previous value
continue;
}
hw_velocities_[i] = velocity;
hw_positions_[i] += delta_rad;
prev_ticks_[i] = current_tick;
} catch (const std::invalid_argument& e) {
RCLCPP_DEBUG(rclcpp::get_logger("MecanumSystem"),
"Encoder parse error wheel %zu: %s", i, e.what());
} catch (const std::out_of_range& e) {
RCLCPP_DEBUG(rclcpp::get_logger("MecanumSystem"),
"Encoder overflow wheel %zu: %s", i, e.what());
}
}
}
else
{
// First read - capture initial encoder values
for (size_t i = 0; i < 4; i++) {
try {
prev_ticks_[i] = std::stoll(parts[11 + i]) * MOTOR_SIGN[i];
} catch (const std::exception& e) {
RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"),
"Initial encoder failed wheel %zu: %s", i, e.what());
prev_ticks_[i] = 0;
}
}
first_read_pass_ = false;
RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"),
"Initial encoder ticks: [%ld, %ld, %ld, %ld]",
prev_ticks_[0], prev_ticks_[1], prev_ticks_[2], prev_ticks_[3]);
}
prev_time_ = current_time;
return hardware_interface::return_type::OK;
}
// ═══════════════════════════════════════════════════════════════════════════════
// WRITE - Send motor commands
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::return_type MecanumSystem::write(
const rclcpp::Time & /*time*/,
const rclcpp::Duration & /*period*/)
{
rclcpp::Time current_time = rclcpp::Clock().now();
// Throttle to 20Hz (every 50ms)
if (first_write_) {
last_write_time_ = current_time;
first_write_ = false;
// Continue to send first command immediately
} else if ((current_time - last_write_time_).seconds() < 0.04) {
return hardware_interface::return_type::OK; // Too soon, skip
}
last_write_time_ = current_time;
// M5Stack motor expects ticks-per-20ms as speed setpoint
constexpr double dt_calc = 0.02;
char cmd_buffer[64];
int cmd_values[4];
for (size_t i = 0; i < 4; i++)
{
double cmd_vel = hw_commands_[i];
// Check for NaN/Inf
if (!std::isfinite(cmd_vel)) {
RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MecanumSystem"),
*node_->get_clock(), 1000,
"Invalid command wheel %zu: %f", i, cmd_vel);
cmd_vel = 0.0;
}
// Apply motor sign (same sign array as encoder)
cmd_vel *= MOTOR_SIGN[i];
// Convert rad/s to ticks-per-20ms
double speed_point = (cmd_vel / (2.0 * M_PI)) * ticks_per_rev_ * dt_calc;
int cmd = static_cast<int>(std::round(speed_point));
// Clamp to valid range
if (cmd > 127) cmd = 127;
if (cmd < -127) cmd = -127;
cmd_values[i] = cmd;
}
// Build command string with snprintf for safety
int len = snprintf(cmd_buffer, sizeof(cmd_buffer), "M,%d,%d,%d,%d\r\n",
cmd_values[0], cmd_values[1], cmd_values[2], cmd_values[3]);
if (len < 0 || len >= static_cast<int>(sizeof(cmd_buffer))) {
RCLCPP_ERROR(rclcpp::get_logger("MecanumSystem"), "Command buffer overflow!");
return hardware_interface::return_type::ERROR;
}
try {
serial_port_.Write(cmd_buffer);
serial_port_.DrainWriteBuffer();
} catch (const std::exception& e) {
RCLCPP_ERROR(rclcpp::get_logger("MecanumSystem"), "Write failed: %s", e.what());
return hardware_interface::return_type::ERROR;
}
return hardware_interface::return_type::OK;
}
// ═══════════════════════════════════════════════════════════════════════════════
// INTERFACE EXPORTS
// ═══════════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface> MecanumSystem::export_state_interfaces()
{
std::vector<hardware_interface::StateInterface> state_interfaces;
for (size_t i = 0; i < info_.joints.size(); i++) {
state_interfaces.emplace_back(
info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
state_interfaces.emplace_back(
info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
}
return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> MecanumSystem::export_command_interfaces()
{
std::vector<hardware_interface::CommandInterface> command_interfaces;
for (size_t i = 0; i < info_.joints.size(); i++) {
command_interfaces.emplace_back(
info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
}
return command_interfaces;
}
} // namespace mecanum_hardware
PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumSystem, hardware_interface::SystemInterface)