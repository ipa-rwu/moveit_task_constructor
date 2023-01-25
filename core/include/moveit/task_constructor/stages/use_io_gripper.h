#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "rclcpp/rclcpp.hpp"

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotModel);
}
}  // namespace moveit

namespace moveit {
namespace task_constructor {
namespace stages {

class UseIOGripper : public PropagatingEitherWay
{
public:
	UseIOGripper(const std::string& name = "use io gripper", const std::string& service_name = "set_io");

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	void setGripperState(const std::string command);

	void setPathConstraints(moveit_msgs::msg::Constraints path_constraints) {
		setProperty("path_constraints", std::move(path_constraints));
	}

protected:
	rclcpp::Node::SharedPtr node_;
	rclcpp::CallbackGroup::SharedPtr callback_group_;
	rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
	std::string service_name_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
