| Benchmark | Python median | C++ median | Python p99 | C++ p99 | Python / C++ (median) | Spread of repeat medians (py, C++) |
|---|---|---|---|---|---|---|
| IK, UR3 DH model | 1,361 us | 3.80 us | 40,173 us | 108 us | 358.4x | 1,115 to 1,702 us, 2.83 to 4.02 us |
| IK, MuJoCo model | 600 us | 4.77 us | 16,308 us | 133 us | 125.9x | 482 to 606 us, 4.41 to 7.37 us |
| Impedance law only | 4.00 us | 0.04 us | 7.58 us | 0.30 us | 114.1x | 3.97 to 6.20 us, 0.03 to 0.07 us |
| Impedance with Pinocchio | 11.34 us | 1.68 us | 26.93 us | 2.56 us | 6.8x | 10.85 to 18.91 us, 1.26 to 2.23 us |
| ROS 2 round trip at 500 Hz | 288 us | 106 us | 2,654 us | 2,215 us | 2.7x | 265 to 339 us, 97.34 to 160 us |

| IK cost per iteration | Python | C++ | Python / C++ |
|---|---|---|---|
| IK, UR3 DH model | 174 us | 0.53 us | 326.9x |
| IK, MuJoCo model | 71.44 us | 0.67 us | 106.4x |

| ROS 2 node | CPU (one core) | Memory (RSS) | Achieved publish rate | Messages lost |
|---|---|---|---|---|
| rclpy | 23.5% | 59 MB | 496 Hz | 0.00% |
| rclcpp | 4.0% | 22 MB | 496 Hz | 0.00% |

