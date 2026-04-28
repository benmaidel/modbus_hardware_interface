#include "modbus_hardware_interface/flexisoft_hardware_interface.hpp"
#include "modbus_hardware_interface/modbus_client.hpp"
#include "modbus_hardware_interface/modbus_exceptions.hpp"

#include <vector>

namespace modbus_hardware_interface
{

hardware_interface::CallbackReturn FlexisoftHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Use base class parsing (creates a ModbusClient we won't use directly for raw access but keeps configs)
  auto ret = ModbusHardwareInterface::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  flexisoft_client_ = std::make_unique<FlexisoftClient>(client_, 10.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexisoftHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ModbusHardwareInterface::on_configure(previous_state);
  if (ret != hardware_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  flexisoft_client_->start();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexisoftHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connection_established()) {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<uint16_t> raw_data;
  try {
    raw_data = flexisoft_client_->read_raw_register(raw_read_config_);
  } catch (const ModbusReadException & e) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("FlexisoftHardwareInterface"),
      "Failed to read raw data with error:" << e.what() << ".");
    read_failure_count_++;
    if (read_failure_count_ >= MAX_READ_FAILURES) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("FlexisoftHardwareInterface"),
        "Maximum read failures reached. Deactivating.");
      return hardware_interface::return_type::DEACTIVATE;
    }
    else {
      return hardware_interface::return_type::OK;
    }
  }

  if (raw_data.empty()) {
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & [name, config] : state_interface_to_config_) {
    if (config.get_register_address() < raw_read_config_.get_register_address() ||
        config.get_register_address() >= raw_read_config_.get_register_address() + raw_read_config_.number_of_bits()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("FlexisoftHardwareInterface"),
        "Register address " << config.get_register_address() << " out of range. Valid range is ["
                           << raw_read_config_.get_register_address() << ", "
                           << raw_read_config_.get_register_address() + raw_read_config_.number_of_bits() - 1 << "]");
      return hardware_interface::return_type::ERROR;
    }
    int bit_index = config.get_register_address() - raw_read_config_.get_register_address();
    size_t word_index = static_cast<size_t>(bit_index / 16);
    int bit_in_word_index = bit_index % 16;
    uint16_t word = raw_data[word_index];
    uint16_t bit_value = (word >> bit_in_word_index) & 0x0001;
    state_interface_to_states_[name] = static_cast<double>(bit_value);
  }

  disconnect_non_persistent_connection();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexisoftHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connection_established()) {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<uint16_t> raw_data(5, 0); // 5 registers

  for (const auto & [name, config] : command_interface_to_config_) {
    if (config.write_this_interface()) {
      if (config.get_register_address() < raw_write_config_.get_register_address() ||
          config.get_register_address() >= raw_write_config_.get_register_address() + raw_write_config_.number_of_bits()) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("FlexisoftHardwareInterface"),
          "Register address " << config.get_register_address() << " out of range. Valid range is ["
                             << raw_write_config_.get_register_address() << ", "
                             << raw_write_config_.get_register_address() + raw_write_config_.number_of_bits() - 1 << "]");
        return hardware_interface::return_type::ERROR;
      }
      int bit_index = config.get_register_address() - raw_write_config_.get_register_address();
      size_t word_index = static_cast<size_t>(bit_index / 16);
      int bit_in_word_index = bit_index % 16;
      uint16_t bit_value = static_cast<uint16_t>(command_interface_to_commands_.at(name)) & 0x0001;
      raw_data[word_index] |= (bit_value << bit_in_word_index);
    }
  }

  try {
    flexisoft_client_->write_raw_register(raw_write_config_, raw_data);
  } catch (const ModbusWriteException & e) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("FlexisoftHardwareInterface"),
      "Failed to write raw data with error:" << e.what() << ". Deactivating .");
    write_failure_count_++;
    if (write_failure_count_ >= MAX_WRITE_FAILURES) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("FlexisoftHardwareInterface"),
        "Maximum write failures reached. Deactivating.");
      return hardware_interface::return_type::DEACTIVATE;
    }
    else {
      return hardware_interface::return_type::OK;
    }
  }

  disconnect_non_persistent_connection();
  return hardware_interface::return_type::OK;
}

}  // namespace modbus_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  modbus_hardware_interface::FlexisoftHardwareInterface, hardware_interface::SystemInterface)
