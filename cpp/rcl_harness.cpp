// Latency harness: publishes /joint_states at a fixed rate stamped with the wall clock, receives the node's
// /ctrl_check echo, and records stamp-to-receive latency. usage: rcl_harness rate_hz seconds out.csv
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>

static int64_t wall_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

class Harness : public rclcpp::Node {
public:
  Harness(double rate, double secs) : Node("ctrlbench_harness"), secs_(secs) {
    pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    sub_ = create_subscription<std_msgs::msg::Header>(
      "/ctrl_check", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Header::ConstSharedPtr m) {
        int64_t sent = (int64_t)m->stamp.sec * 1000000000LL + m->stamp.nanosec;
        if (started_) lat_us_.push_back((wall_ns() - sent) / 1e3);
      });
    msg_.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
    msg_.position = {0.1, -1.2, 1.5, -1.0, -1.4, 0.3};
    msg_.velocity.assign(6, 0.0);
    msg_.effort.assign(6, 0.0);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
                               [this]() { tick(); });
  }
  void begin() { started_ = true; t0_ = wall_ns(); }
  bool done() const { return started_ && (wall_ns() - t0_) * 1e-9 > secs_; }
  void write(const char * path) {
    std::ofstream f(path);
    f << "lat_us\n";
    for (double v : lat_us_) f << v << "\n";
    double dur = (wall_ns() - t0_) * 1e-9;
    std::printf("published=%ld received=%zu duration=%.2f achieved_rate=%.1f\n", (long)sent_, lat_us_.size(), dur, sent_ / dur);
  }
  long sent_ = 0;
private:
  void tick() {
    int64_t n = wall_ns();
    msg_.header.stamp.sec = (int32_t)(n / 1000000000LL);
    msg_.header.stamp.nanosec = (uint32_t)(n % 1000000000LL);
    pub_->publish(msg_);
    if (started_) ++sent_;
  }
  double secs_;
  bool started_ = false;
  int64_t t0_ = 0;
  std::vector<double> lat_us_;
  sensor_msgs::msg::JointState msg_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  if (argc < 4) { std::fprintf(stderr, "usage: rcl_harness rate_hz seconds out.csv\n"); return 2; }
  auto node = std::make_shared<Harness>(std::stod(argv[1]), std::stod(argv[2]));
  // wait for the node under test to be discovered before the timed window starts
  auto t_wait = wall_ns();
  while (rclcpp::ok() && node->count_subscribers("/joint_states") == 0 && (wall_ns() - t_wait) * 1e-9 < 20)
    rclcpp::spin_some(node);
  for (int i = 0; i < 200 && rclcpp::ok(); ++i) { rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  node->begin();
  while (rclcpp::ok() && !node->done()) rclcpp::spin_some(node);
  node->write(argv[3]);
  rclcpp::shutdown();
}
