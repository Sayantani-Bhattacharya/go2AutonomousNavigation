// A node that subscribes to /map data and publishes a ed nav goal for the robot to explore the map.
// The robot will explore the map by moving to the nearest frontier point.
// The robot will stop exploring when the map is fully explored or with a user service call.
// Increase the footprint of the robot in the costmap to avoid collision with the map.: this is done by setting the radius of the robot to 0.3m in the costmap_common_params.yaml file. -----> see

//Services:
// 1. /explore_explore : Service to start exploring the map.
// 2. /explore_stop : Service to stop the robot from exploring the map.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_go2_nav_interfaces/msg/nav_to_pose.hpp"

// State machine
enum class State
{
    IDLE,
    MANUAL_NAV_MODE,
    AUTO_NAV_MODE,
    START,
    MOVING,
    WAIT_FOR_MOVEMENT_COMPLETE,
    REACHED_GOAL,
};


class Explore : public rclcpp::Node
{
    public:

     Explore():Node("nav_to_pose")
     { 
     }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Explore>());
  rclcpp::shutdown();
  return 0;
}