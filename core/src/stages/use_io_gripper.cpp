#include <moveit/task_constructor/stages/use_io_gripper.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/utils.h>
#include <rviz_marker_tools/marker_creation.h>

#include "ur_msgs/srv/set_io.hpp"

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("UseIOGripper");

UseIOGripper::UseIOGripper(const std::string& name, const std::string& service_name)
  : PropagatingEitherWay(name), service_name_(service_name) {
	auto& p = properties();
	Property& timeout = p.property("timeout");
	timeout.setDefaultValue(1.0);
}

void UseIOGripper::init(const moveit::core::RobotModelConstPtr& robot_model) {
	PropagatingEitherWay::init(robot_model);
}

// Pin 1 is open
void UseIOGripper::setGripperState(const std::string command) {
	rclcpp::NodeOptions options;
	options.arguments(
	    { "--ros-args", "-r", "__node:=current_state_" + std::to_string(reinterpret_cast<std::size_t>(this)) });
	auto node = rclcpp::Node::make_shared("_", options);
	auto client = node->create_client<ur_msgs::srv::SetIO>(service_name_);

	auto timeout = std::chrono::duration<double>(this->timeout());
	int open = 1;
	if (command == "close") {
		open = 0;
	}

	if (client->wait_for_service(timeout)) {
		auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
		req->fun = req->FUN_SET_DIGITAL_OUT;
		req->state = 1;
		req->pin = static_cast<int8_t>(open);  // Pin 1 is open

		auto res_future = client->async_send_request(req);
		if (rclcpp::spin_until_future_complete(node, res_future) == rclcpp::FutureReturnCode::SUCCESS) {
			RCLCPP_INFO(LOGGER, "Open gripper initialise : %s",
			            ((res_future.get()->success == 0) ? "Failed" : "Succeeded"));
			req->state = 0;
			auto res_future = client->async_send_request(req);
			if (rclcpp::spin_until_future_complete(node, res_future) == rclcpp::FutureReturnCode::SUCCESS) {
				RCLCPP_INFO(LOGGER, "Open gripper conclude : %s",
				            ((res_future.get()->success == 0) ? "Failed" : "Succeeded"));
				return;
			} else {
				RCLCPP_ERROR(LOGGER, "Open gripper conclude : Failed");
				return;
			}
		} else {
			RCLCPP_ERROR(LOGGER, "Open gripper initialise : Failed");
			return;
		}
	}
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
