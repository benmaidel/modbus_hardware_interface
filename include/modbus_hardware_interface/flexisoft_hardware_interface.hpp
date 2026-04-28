#pragma once

#include "modbus_hardware_interface.hpp"
#include "flexisoft_client.hpp"

namespace modbus_hardware_interface
{

class FlexisoftHardwareInterface : public ModbusHardwareInterface
{
public:
  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr int READ_REGISTER = 1099;
  static constexpr int READ_COUNT = 50 * 8;
  static constexpr int WRITE_REGISTER = 2099;
  static constexpr int WRITE_COUNT = 5 * 16;

  static constexpr int MAX_READ_FAILURES = 5;
  static constexpr int MAX_WRITE_FAILURES = 5;

  ModbusInterfaceReadConfig raw_read_config_{READ_REGISTER, READ_COUNT, REGISTER, CONVERSION_FN_DEFAULT};
  ModbusInterfaceWriteConfig raw_write_config_{WRITE_REGISTER, WRITE_COUNT, REGISTER, CONVERSION_FN_DEFAULT};

  int read_failure_count_{0};
  int write_failure_count_{0};

  std::unique_ptr<FlexisoftClient> flexisoft_client_;
};

}  // namespace modbus_hardware_interface
