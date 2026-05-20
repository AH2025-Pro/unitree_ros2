/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <chrono>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unitree_go/msg/detail/sport_mode_state__struct.hpp>

#include "common/ros2_b2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

enum TestMode {
  STAND = 0,
  CLIMB_STAIRS = 1,
  DESCEND_STAIRS = 2,
  NORMAL_WALK = 3,
};

class Go2StairClimbNode : public rclcpp::Node {
 public:
  explicit Go2StairClimbNode(int test_mode)
      : Node("go2_stair_climb_node"),
        sport_client_(this),
        test_mode_(test_mode) {
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          HighStateHandler(data);
        });
    t1_ = std::thread([this] {
      // Wait for ROS2 spin
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      RobotControl();
    });
  }

  void RobotControl() {
    ct_ += dt_;
    switch (test_mode_) {
      case STAND: {
        RCLCPP_INFO(this->get_logger(), "Normal Stand");
        sport_client_.StandUp(req_);
        break;
      }
      case CLIMB_STAIRS: {
        RCLCPP_INFO(this->get_logger(), "Climb Stairs Mode (gait_type = 3)");
        // 1. First stand up
        sport_client_.StandUp(req_);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // 2. Switch to climb stairs gait (3)
        sport_client_.SwitchGait(req_, 3);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 3. Move forward
        sport_client_.Move(req_, 0.3, 0.0, 0.0);
        break;
      }
      case DESCEND_STAIRS: {
        RCLCPP_INFO(this->get_logger(), "Descend Stairs Mode (gait_type = 4)");
        // 1. First stand up
        sport_client_.StandUp(req_);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // 2. Switch to descend stairs gait (4)
        sport_client_.SwitchGait(req_, 4);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 3. Move forward
        sport_client_.Move(req_, 0.3, 0.0, 0.0);
        break;
      }
      case NORMAL_WALK: {
        RCLCPP_INFO(this->get_logger(), "Normal Walk Mode (gait_type = 1)");
        // 1. First stand up
        sport_client_.StandUp(req_);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // 2. Switch to normal trot gait (1)
        sport_client_.SwitchGait(req_, 1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 3. Move forward
        sport_client_.Move(req_, 0.3, 0.0, 0.0);
        break;
      }
      default: {
        RCLCPP_INFO(this->get_logger(), "Stop Move");
        sport_client_.StopMove(req_);
        break;
      }
    }
  }

  void HighStateHandler(
      const unitree_go::msg::SportModeState::SharedPtr data) {
    state_ = *data;
    RCLCPP_INFO(this->get_logger(),
                "Position: x=%.3f y=%.3f z=%.3f | gait_type=%d",
                state_.position[0], state_.position[1], state_.position[2],
                state_.gait_type);
  }

 private:
  unitree_go::msg::SportModeState state_;
  SportClient sport_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;

  unitree_api::msg::Request req_;
  double ct_ = 0.0;
  float dt_ = 0.1;
  int test_mode_;
  std::thread t1_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <test_mode>" << std::endl;
    std::cerr << "Available test modes:" << std::endl;
    std::cerr << "  0: STAND (Normal Stand)" << std::endl;
    std::cerr << "  1: CLIMB_STAIRS (Upstairs, gait_type=3)" << std::endl;
    std::cerr << "  2: DESCEND_STAIRS (Downstairs, gait_type=4)" << std::endl;
    std::cerr << "  3: NORMAL_WALK (Trot, gait_type=1)" << std::endl;
    return 1;
  }

  int test_mode = std::atoi(argv[1]);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2StairClimbNode>(test_mode);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
