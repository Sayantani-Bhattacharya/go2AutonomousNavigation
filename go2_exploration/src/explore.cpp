// A node that subscribes to /map data and publishes a ed nav goal for the robot to explore the map.
// The robot will explore the map by moving to the nearest frontier point.
// The robot will stop exploring when the map is fully explored or with a user service call.
// Increase the footprint of the robot in the costmap to avoid collision with the map.: this is done by setting the radius of the robot to 0.3m in the costmap_common_params.yaml file. -----> see

//Services:
// 1. /explore_start : Service to start exploring the map.
// 2. /explore_stop : Service to stop the robot from exploring the map.

// Subscribers:
// 1. /map : Subscribes to the map data.
// 2. /goal_reached : Subscribes to the goal pose for the robot to move to.
// 3. lookup transform

// Algorithm:
// seach frontiers, find the nearest frontier, publish the goal pose, move to the goal pose, repeat.

// Publishers:
// frontier_markers : Publishes the frontier points on the map.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// from nav_msgs.msg import OccupancyGrid
#include "go2_exploration_interfaces/srv/empty.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_go2_nav_interfaces/msg/nav_to_pose.hpp"
#include "go2_exploration_interfaces/srv/empty.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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

using namespace std::chrono_literals;

class Explore : public rclcpp::Node
{
    public:
     Explore():Node("explore")
     { 
        // Services
        nav_start_trigger = create_service<go2_exploration_interfaces::srv::Empty>("explore_start", std::bind(&Explore::nav_start_trigger_callback, this, std::placeholders::_1, std::placeholders::_2));
        nav_stop_trigger = create_service<go2_exploration_interfaces::srv::Empty>("explore_stop", std::bind(&Explore::nav_stop_trigger_callback, this, std::placeholders::_1, std::placeholders::_2));    

        // Subscribers
        // /goal_reached -- if needed.
        map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&Explore::map_sub_callback, this, std::placeholders::_1));

        
        // Transform listener
        // TODO: change odom to base_footprint later !!. 
        target_frame_ = this->declare_parameter<std::string>("odom", "base_link");
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        // Considering base footprint frame is projection on map plane(same z axis).

        // Publishers
        goal_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);  


        // frontier_markers : Publishes the frontier points on the map.

        // Timer
        mTimer = this->create_wall_timer(100ms, std::bind(&Explore::timer_callback, this));    
     }     

    private:
      rclcpp::TimerBase::SharedPtr mTimer;
      State mRobotState = State::IDLE;
      geometry_msgs::msg::PoseStamped mCurrPose;

      // Map data
      nav_msgs::msg::OccupancyGrid mMap;
      float mMapWidth;
      float mMapHeight;
      float mMapResolution;
      geometry_msgs::msg::Pose mMapOrigin;
      rclcpp::Time mMapLoadTime;
      std::vector<std::vector<int>> mMapGrid;

      // TF
      std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      std::string target_frame_;    

    
      rclcpp::Service<go2_exploration_interfaces::srv::Empty>::SharedPtr nav_start_trigger;
      rclcpp::Service<go2_exploration_interfaces::srv::Empty>::SharedPtr nav_stop_trigger;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_sub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;


      void nav_start_trigger_callback(const std::shared_ptr<go2_exploration_interfaces::srv::Empty::Request> request, std::shared_ptr<go2_exploration_interfaces::srv::Empty::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "Starting Exploration.");
        mRobotState = State::START;
        response->r = true;
      } 

      void nav_stop_trigger_callback(const std::shared_ptr<go2_exploration_interfaces::srv::Empty::Request> request, std::shared_ptr<go2_exploration_interfaces::srv::Empty::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "Stopping Exploration.");
        mRobotState = State::E_STOP;
        response->r = true;
      } 

      void map_sub_callback(const nav_msgs::msg::OccupancyGrid msg)
      {
        /*
        Get the map explored by GO2 and convert it to a grid format 
        that can be used by the exploration algo.
        */

        mMap = msg;

        // Map information
        mMapWidth = msg.info.width;
        mMapHeight = msg.info.height;
        mMapResolution = msg.info.resolution;
        mMapOrigin = msg.info.origin;
        mMapLoadTime = msg.info.map_load_time;
        mMapGrid.clear();
        mMapGrid.resize(mMapHeight, std::vector<int>(mMapWidth, 0));
        for (int i = 0; i < mMapHeight; i++)
        {
            for (int j = 0; j < mMapWidth; j++)
            {
                int index = j + i * mMapWidth;  
                if (index >= 0 && index < msg.data.size()) {
                    mMapGrid[i][j] = msg.data[index];
                } 
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "Index out of bounds: %d", index);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Map received: width= %f", mMapWidth);
        RCLCPP_INFO(this->get_logger(), "Map received: height= %f", mMapHeight);
        RCLCPP_INFO(this->get_logger(), "Map received: resolution= %f", mMapResolution);
        RCLCPP_INFO(this->get_logger(), "Map received: origin x= %f", mMapOrigin.position.x);
        RCLCPP_INFO(this->get_logger(), "Map received: origin y= %f", mMapOrigin.position.y);
        RCLCPP_INFO(this->get_logger(), "Map received: origin z= %f", mMapOrigin.position.z);      
        RCLCPP_INFO(this->get_logger(), "Map received: shape= %i", mMapGrid.size());

        // Count occurrences
        int count_1 = 0, count_0 = 0, count_neg1 = 0;
        for (const auto& row : mMapGrid) {
            for (int val : row) {
                if (val == 1) count_1++;
                else if (val == 0) count_0++;
                else if (val == -1) count_neg1++;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Occupied cells: %i, Free cells: %i, Unknown cells: %i", count_1, count_0, count_neg1);

      }

      void curr_pose_sub_callback(const geometry_msgs::msg::PoseStamped msg)
      {
        mCurrPose = msg;
        RCLCPP_INFO(this->get_logger(), "Current pose received: x= %f, y= %f", msg.pose.position.x, msg.pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "Current pose received: x=, y=", msg.pose.position.x, msg.pose.position.y);
      }

      void timer_callback()
      {

        // Store frame names in variables that will be used to compute transformations.

        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "map";
        // Lookup transform
        geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          t = tf_buffer->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        float x = t.transform.translation.x;
        float y = t.transform.translation.y;
        float z = t.transform.translation.z;
        RCLCPP_INFO(this->get_logger(), "Transform btw odom and map: x= %f, y= %f, z= %f", x, y, z);



        if (mRobotState == State::START)
        {
          // RCLCPP_INFO(this->get_logger(), "Inside timer, robotsate is {0}.", static_cast<int>(mRobotState)); 

          geometry_msgs::msg::PoseStamped goalMsg;          
          goalMsg.pose.position.x = 1.0;
          goalMsg.pose.position.y = 1.0;
          goalMsg.pose.position.z = 0.0;
          goalMsg.pose.orientation.x = 0.0;
          goalMsg.pose.orientation.y = 0.0;
          goalMsg.pose.orientation.z = 0.0;
          goalMsg.pose.orientation.w = 1.0;
          goalMsg.header.frame_id = "map";
          goalMsg.header.stamp = this->now();        
          goal_pose_pub->publish(goalMsg);
        }

        // can check here if the robot has reached the goal pose. can use lookup bet goalpose and current pose. but odnt ahve goalpose tf?
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