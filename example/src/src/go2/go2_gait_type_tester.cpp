/**
 * Go2 Gait Type Tester
 * 测试不同的 gait_type 值，观察哪个有效
 **/
#include <chrono>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "common/ros2_b2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

class GaitTypeTester : public rclcpp::Node {
 public:
  explicit GaitTypeTester(int test_gait)
      : Node("gait_type_tester"),
        sport_client_(this),
        test_gait_(test_gait) {
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Testing Gait Type: %d", test_gait_);
    RCLCPP_INFO(this->get_logger(), "Gait Type meanings:");
    RCLCPP_INFO(this->get_logger(), "  0: idle");
    RCLCPP_INFO(this->get_logger(), "  1: trot (normal walk)");
    RCLCPP_INFO(this->get_logger(), "  2: run");
    RCLCPP_INFO(this->get_logger(), "  3: climb stair (上楼梯)");
    RCLCPP_INFO(this->get_logger(), "  4: forwardDownStair (下楼梯)");
    RCLCPP_INFO(this->get_logger(), "  9: adjust");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          StateCallback(data);
        });
    
    t1_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      TestSequence();
    });
  }

 private:
  void StateCallback(const unitree_go::msg::SportModeState::SharedPtr &data) {
    state_ = *data;
    
    // 持续打印当前状态
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Current gait_type: %d, mode: %d",
                        data->gait_type, data->mode);
  }

  void TestSequence() {
    // 1. 站立
    RCLCPP_INFO(this->get_logger(), "Step 1: Standing up...");
    sport_client_.StandUp(req_);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    RCLCPP_INFO(this->get_logger(), "Current gait_type after StandUp: %d", 
                state_.gait_type);
    
    // 2. 切换步态
    RCLCPP_INFO(this->get_logger(), "Step 2: Switching to gait_type %d...", 
                test_gait_);
    sport_client_.SwitchGait(req_, test_gait_);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    RCLCPP_INFO(this->get_logger(), "Current gait_type after SwitchGait: %d",
                state_.gait_type);
    
    // 3. 尝试移动
    RCLCPP_INFO(this->get_logger(), "Step 3: Starting to move...");
    sport_client_.Move(req_, 0.3, 0.0, 0.0);
    
    // 等待 5 秒观察效果
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 最终状态
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Final Result:");
    RCLCPP_INFO(this->get_logger(), "  Expected gait_type: %d", test_gait_);
    RCLCPP_INFO(this->get_logger(), "  Actual gait_type: %d", state_.gait_type);
    
    if (state_.gait_type == test_gait_) {
      RCLCPP_INFO(this->get_logger(), "  Status: SUCCESS ✓");
    } else {
      RCLCPP_INFO(this->get_logger(), "  Status: FAILED ✗");
      RCLCPP_INFO(this->get_logger(), "  Possible reasons:");
      RCLCPP_INFO(this->get_logger(), "  1. Firmware version too new (>= 1.1.6)");
      RCLCPP_INFO(this->get_logger(), "  2. API is deprecated in your version");
      RCLCPP_INFO(this->get_logger(), "  3. Need to update ROS2 SDK");
    }
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 继续运行一段时间观察
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }

 private:
  unitree_go::msg::SportModeState state_;
  SportClient sport_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  unitree_api::msg::Request req_;
  int test_gait_;
  std::thread t1_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <gait_type>" << std::endl;
    std::cerr << "Test different gait types:" << std::endl;
    std::cerr << "  1: trot (normal walk)" << std::endl;
    std::cerr << "  3: climb stair (上楼梯)" << std::endl;
    std::cerr << "  4: forwardDownStair (下楼梯)" << std::endl;
    return 1;
  }
  
  int test_gait = std::atoi(argv[1]);
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitTypeTester>(test_gait);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
