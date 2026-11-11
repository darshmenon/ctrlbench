// Damped least squares IK on the UR3 DH model, Eigen. Mirrors ctrlbench/bench_py.py run_ik_dh.
#include "common.hpp"
using namespace cb;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

static const double A[6] = {0, -0.24365, -0.21325, 0, 0, 0};
static const double D[6] = {0.1519, 0, 0, 0.11235, 0.08535, 0.0819};
static const double AL[6] = {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};

static Eigen::Matrix4d dh(double th, double d, double a, double al) {
  double ct = std::cos(th), st = std::sin(th), ca = std::cos(al), sa = std::sin(al);
  Eigen::Matrix4d T;
  T << ct, -st * ca, st * sa, a * ct,
       st, ct * ca, -ct * sa, a * st,
       0, sa, ca, d,
       0, 0, 0, 1;
  return T;
}

static void fk_frames(const Vec6 & q, Eigen::Matrix4d T[7]) {
  T[0].setIdentity();
  for (int i = 0; i < 6; ++i) T[i + 1] = T[i] * dh(q[i], D[i], A[i], AL[i]);
}

static Mat6 jac(const Eigen::Matrix4d T[7]) {
  Mat6 J;
  Eigen::Vector3d pe = T[6].block<3, 1>(0, 3);
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector3d z = T[i].block<3, 1>(0, 2), p = T[i].block<3, 1>(0, 3);
    J.block<3, 1>(0, i) = z.cross(pe - p);
    J.block<3, 1>(3, i) = z;
  }
  return J;
}

static void solve(const Eigen::Vector3d & pt, const Eigen::Matrix3d & Rt, int & iters, double & chk) {
  Vec6 q = q_home();
  Eigen::Matrix4d T[7];
  iters = IK_MAX_ITER;
  for (int it = 0; it < IK_MAX_ITER; ++it) {
    fk_frames(q, T);
    Vec6 e;
    e << pt - T[6].block<3, 1>(0, 3), rot_err(T[6].block<3, 3>(0, 0), Rt);
    if (e.norm() < IK_TOL) { iters = it; break; }
    Mat6 J = jac(T);
    Mat6 M = J * J.transpose() + IK_LAMBDA * IK_LAMBDA * Mat6::Identity();
    q += J.transpose() * M.partialPivLu().solve(e);
  }
  chk = q.sum();
}

int main(int argc, char ** argv) {
  if (argc != 3) { std::fprintf(stderr, "usage: ik_dh targets.csv out.csv\n"); return 2; }
  auto rows = read_csv(argv[1]);
  std::vector<Result> res(rows.size());
  volatile double sink = 0;
  for (auto & r : rows) {   // warm-up pass, untimed
    int it; double c;
    solve(Eigen::Vector3d(r[0], r[1], r[2]), mat3(r, 3), it, c);
    sink = sink + c;
  }
  for (size_t i = 0; i < rows.size(); ++i) {
    auto & r = rows[i];
    Eigen::Vector3d pt(r[0], r[1], r[2]);
    Eigen::Matrix3d Rt = mat3(r, 3);
    int it; double c;
    double t0 = now_ns();
    solve(pt, Rt, it, c);
    res[i] = {now_ns() - t0, (double)it, c};
    sink = sink + c;
  }
  write_results(argv[2], res);
  return 0;
}
