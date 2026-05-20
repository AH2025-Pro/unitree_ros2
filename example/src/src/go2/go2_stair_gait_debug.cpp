/**
 * Go2 Stair Gait Debug Version
 * 调试版本：帮助查看为什么 SwitchGait 不起作用
 **/
#include <chrono>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <iostream>

#include "common/ros2_b2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

class StairGaitDebugNode : public rclcpp::Node {
 public:
  StairGaitDebugNode()
      : Node("stair_gait_debug_node"), sport_client_(this) {
    RCLCPP_INFO(this->get_logger(), "=== Go2 Stair Gait Debug Started ===");
    
    // 订阅运动状态，监控当前 gait_type
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          StateCallback(data);
        });
    
    // 延迟启动控制逻辑，等待订阅建立
    t1_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      StartSequence();
    });
  }

 private:
  void StateCallback(const unitree_go::msg::SportModeState::SharedPtr &data) {
    state_ = *data;
    
    // 每秒打印一次状态
    static int counter = 0;
    counter++;
    if (counter % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  ">>> Current State: gait_type=%d, mode=%d, "
                  "pos=(%.2f, %.2f, %.2f)",
                  data->gait_type, data->mode,
                  data->position[0], data->position[1], data->position[2]);
    }
  }

  void StartSequence() {
    RCLCPP_INFO(this->get_logger(), "=== Step 1: Standing up... ===");
    
    // 1. 首先站立
    sport_client_.StandUp(req_);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    RCLCPP_INFO(this->get_logger(), 
                ">>> After StandUp: gait_type=%d, mode=%d",
                state_.gait_type, state_.mode);
    
    RCLCPP_INFO(this->get_logger(), "=== Step 2: Switching to Stairs Gait (3)... ===");
    
    // 2. 切换到爬楼梯步态 (参数 3 = 上楼梯)
    sport_client_.SwitchGait(req_, 3);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    RCLCPP_INFO(this->get_logger(), 
                ">>> After SwitchGait(3): gait_type=%d, mode=%d",
                state_.gait_type, state_.mode);
    
    // 3. 开始移动
    RCLCPP_INFO(this->get_logger(), "=== Step 3: Starting to move forward... ===");
    sport_client_.Move(req_, 0.5, 0.0, 0.0);
    
    // 持续监控
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    RCLCPP_INFO(this->get_logger(), 
                ">>> After moving: gait_type=%d, mode=%d",
                state_.gait_type, state_.mode);
    
    RCLCPP_INFO(this->get_logger(), 
                "=== Debug Complete ===");
    RCLCPP_INFO(this->get_logger(), 
                "If gait_type is still 1 (not 3), the API might be deprecated in your firmware version.");
  }

 private:
  unitree_go::msg::SportModeState state_;
  SportClient sport_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  unitree_api::msg::Request req_;
  std::thread t1_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<StairGaitDebugNode>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
