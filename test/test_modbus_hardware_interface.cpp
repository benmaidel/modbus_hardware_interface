// Copyright (c) 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestModbusHardwareInterface : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  hardware_interface::ResourceManager make_resource_manager(const std::string & urdf)
  {
    return hardware_interface::ResourceManager(
      urdf, std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
      rclcpp::get_logger("test_modbus_hardware_interface"));
  }

  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    valid_modbus_hardware =
      R"(
        <ros2_control name="ModbusHardwareInterface2dof" type="system">
          <hardware>
            <plugin>modbus_hardware_interface/ModbusHardwareInterface</plugin>
            <param name="modbus_server_ip">127.0.0.1</param>
            <param name="modbus_server_port">1234</param>
            <param name="use_persistent_connection">true</param>
          </hardware>
          <joint name="joint1">
            <command_interface name="position">
              <param name="register">1</param>
              <param name="bits_to_read">32</param>
              <param name="conversion_fn">float_abcd</param>
              <param name="write_function">register</param>
            </command_interface>
            <command_interface name="velocity">
              <param name="register">2</param>
              <param name="bits_to_read">1</param>
              <param name="conversion_fn">float</param>
              <param name="write_function">bits</param>
            </command_interface>

            <state_interface name="position">
              <param name="register">2</param>
              <param name="bits_to_read">1</param>
              <param name="conversion_fn">float</param>
              <param name="read_function">input_bits</param>
            </state_interface>
            <state_interface name="velocity">
              <param name="register">32</param>
              <param name="bits_to_read">32</param>
              <param name="conversion_fn">float_abcd</param>
              <param name="read_function">input_register</param>
            </state_interface>
          </joint>
        </ros2_control>
    )";
  }
  std::string valid_modbus_hardware;
};

TEST_F(TestModbusHardwareInterface, load_modbus_hardware_interface_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + valid_modbus_hardware +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(make_resource_manager(urdf));
}

// When the modbus server is unreachable, configuring/activating the component must NOT throw.
// Otherwise a failed lifecycle transition during forced startup activation aborts the whole
// ros2_control_node. The interface is expected to come up and reconnect in the read/write cycle.
TEST_F(TestModbusHardwareInterface, configure_and_activate_succeeds_when_server_unreachable)
{
  // Port 1234 in the URDF above points at a server that is not running in this test.
  auto urdf = ros2_control_test_assets::urdf_head + valid_modbus_hardware +
              ros2_control_test_assets::urdf_tail;
  auto rm = make_resource_manager(urdf);

  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_NO_THROW({
    auto ret = rm.set_component_state("ModbusHardwareInterface2dof", active_state);
    EXPECT_EQ(ret, hardware_interface::return_type::OK);
  });
}
