# Changelog

## 0.1.0
- Measured C++ vs Python comparison of arm-control building blocks on a UR3: damped least squares IK (DH model with numpy and Eigen, MuJoCo model through the Python and C APIs), the Cartesian impedance law (numpy and Eigen, and the full Pinocchio path in both languages), and a 500 Hz ROS 2 round trip (rclpy and rclcpp).
- Every implementation pair is checked to produce the same results (same IK iteration counts, torque checksums within 1e-14) before timing.
- Runs are repeated, pinned to the least busy core, interleaved, and report median, p95, p99, jitter and the spread between repeats.
- `make build`, `make test`, `make bench`.
