// A node that subscribes to /map data and publishes a ed nav goal for the robot to explore the map.
// The robot will explore the map by moving to the nearest frontier point.
// The robot will stop exploring when the map is fully explored or with a user service call.
// Increase the footprint of the robot in the costmap to avoid collision with the map.: this is done by setting the radius of the robot to 0.3m in the costmap_common_params.yaml file. -----> see

//Services:
// 1. /explore_explore : Service to start exploring the map.
// 2. /explore_stop : Service to stop the robot from exploring the map.

// Subscribers:
// 1. /map : Subscribes to the map data.
// 2. /goal_reached : Subscribes to the goal pose for the robot to move to.

// Publishers:
// frontier_markers : Publishes the frontier points on the map.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_exploration_interfaces/srv/empty.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_go2_nav_interfaces/msg/nav_to_pose.hpp"
#include "go2_exploration_interfaces/srv/empty.hpp"

// State machine
enum class State
{
    IDLE,
    // MANUAL_NAV_MODE,
    // AUTO_NAV_MODE,
    START,
    MOVING,
    WAIT_FOR_MOVEMENT_COMPLETE,
    REACHED_GOAL,
    E_STOP,
};


class Explore : public rclcpp::Node
{
    public:
     Explore():Node("explore")
     { 
        // Services
        nav_start_trigger = create_service<go2_exploration_interfaces::srv::Empty>("nav_start_trigger", std::bind(&Explore::nav_start_trigger_callback, this, std::placeholders::_1, std::placeholders::_2));
        nav_stop_trigger = create_service<go2_exploration_interfaces::srv::Empty>("nav_stop_trigger", std::bind(&Explore::nav_stop_trigger_callback, this, std::placeholders::_1, std::placeholders::_2));        
     }

    private:
      rclcpp::TimerBase::SharedPtr mTimer;
      State mRobotState = State::IDLE;
      rclcpp::Service<go2_exploration_interfaces::srv::Empty>::SharedPtr nav_start_trigger;
      rclcpp::Service<go2_exploration_interfaces::srv::Empty>::SharedPtr nav_stop_trigger;

      void nav_start_trigger_callback(const std::shared_ptr<go2_exploration_interfaces::srv::Empty::Request> request, std::shared_ptr<go2_exploration_interfaces::srv::Empty::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "Inside add service.");
        mRobotState = State::START;
        response->r = true;
      } 

      void nav_stop_trigger_callback(const std::shared_ptr<go2_exploration_interfaces::srv::Empty::Request> request, std::shared_ptr<go2_exploration_interfaces::srv::Empty::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "Inside add service.");
        mRobotState = State::E_STOP;
        response->r = true;
      } 

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Explore>());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside main.");

  rclcpp::shutdown();
  return 0;
}