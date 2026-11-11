// Shared helpers for the ctrlbench C++ programs: CSV input, timing, the settings every implementation uses.
#pragma once
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace cb {
using Clock = std::chrono::steady_clock;
inline double now_ns() {
  return std::chrono::duration<double, std::nano>(Clock::now().time_since_epoch()).count();
}

inline std::vector<std::vector<double>> read_csv(const std::string & path) {
  std::ifstream f(path);
  if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); std::exit(2); }
  std::vector<std::vector<double>> rows;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::vector<double> r;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) r.push_back(std::stod(tok));
    rows.push_back(std::move(r));
  }
  return rows;
}

struct Result { double ns, iters, chk; };

inline void write_results(const std::string & path, const std::vector<Result> & res) {
  std::ofstream f(path);
  f << "time_ns,iters,checksum\n";
  f.precision(17);
  for (auto & r : res) f << r.ns << "," << r.iters << "," << r.chk << "\n";
}

// identical to ctrlbench/common.py
constexpr double IK_LAMBDA = 0.05, IK_TOL = 1e-4;
constexpr int IK_MAX_ITER = 200;
inline Eigen::Matrix<double, 6, 1> q_home() {
  Eigen::Matrix<double, 6, 1> q;
  q << 0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0;
  return q;
}
inline Eigen::Matrix<double, 6, 1> k_gain() { return (Eigen::Matrix<double, 6, 1>() << 800, 800, 800, 30, 30, 30).finished(); }
inline Eigen::Matrix<double, 6, 1> d_gain() { return (Eigen::Matrix<double, 6, 1>() << 40, 40, 40, 2, 2, 2).finished(); }

inline Eigen::Vector3d rot_err(const Eigen::Matrix3d & Rc, const Eigen::Matrix3d & Rt) {
  return 0.5 * (Rc.col(0).cross(Rt.col(0)) + Rc.col(1).cross(Rt.col(1)) + Rc.col(2).cross(Rt.col(2)));
}
// row-major 3x3 from a CSV slice
inline Eigen::Matrix3d mat3(const std::vector<double> & v, size_t off) {
  Eigen::Matrix3d m;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) m(i, j) = v[off + 3 * i + j];
  return m;
}
}  // namespace cb
