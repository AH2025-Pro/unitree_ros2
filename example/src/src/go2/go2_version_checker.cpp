/**
 * Go2 Robot Version Checker
 * This program displays the robot's firmware/software version information
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"

class VersionChecker : public rclcpp::Node {
 public:
  VersionChecker() : Node("go2_version_checker") {
    RCLCPP_INFO(this->get_logger(), "Starting Go2 Version Checker...");
    
    // Subscribe to lowstate topic to get version information
    suber_ = this->create_subscription<unitree_go::msg::LowState>(
        "lowstate", 10,
        [this](const unitree_go::msg::LowState::SharedPtr data) {
          CheckVersion(data);
        });
  }

 private:
  void CheckVersion(const unitree_go::msg::LowState::SharedPtr &data) {
    // Version information is in data->version[]
    // Usually version[0] contains the main version
    uint32_t version_main = data->version[0];
    uint32_t version_sub = data->version[1];
    
    // Display version information
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Robot Version Information:");
    RCLCPP_INFO(this->get_logger(), "Version[0]: %u", version_main);
    RCLCPP_INFO(this->get_logger(), "Version[1]: %u", version_sub);
    
    // Additional info from SN (serial number)
    RCLCPP_INFO(this->get_logger(), "SN[0]: %u", data->sn[0]);
    RCLCPP_INFO(this->get_logger(), "SN[1]: %u", data->sn[1]);
    
    // Battery status
    RCLCPP_INFO(this->get_logger(), "Battery Voltage: %.2f V", data->power_v);
    RCLCPP_INFO(this->get_logger(), "Battery Current: %.2f A", data->power_a);
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // Stop after displaying version once
    rclcpp::shutdown();
  }

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr suber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("main"), "Go2 Robot Version Checker Starting...");
  
  auto node = std::make_shared<VersionChecker>();
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
