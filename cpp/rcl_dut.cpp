// rclcpp node under test: subscribe /joint_states, cheap sanity check, publish a Header echoing the input stamp.
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>

class Dut : public rclcpp::Node {
public:
  Dut() : Node("ctrlbench_dut_cpp") {
    pub_ = create_publisher<std_msgs::msg::Header>("/ctrl_check", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        bool ok = m->position.size() == 6;
        for (double p : m->position) ok = ok && std::isfinite(p) && std::fabs(p) < 6.3;
        std_msgs::msg::Header out;
        out.stamp = m->header.stamp;
        out.frame_id = ok ? "ok" : "bad";
        pub_->publish(out);
      });
  }
private:
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dut>());
  rclcpp::shutdown();
}
