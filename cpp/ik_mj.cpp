// Damped least squares IK through MuJoCo's C API. Mirrors ctrlbench/bench_py.py run_ik_mj.
#include <mujoco/mujoco.h>
#include <cstdlib>
#include "common.hpp"
using namespace cb;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

int main(int argc, char ** argv) {
  if (argc != 4) { std::fprintf(stderr, "usage: ik_mj model.xml targets.csv out.csv\n"); return 2; }
  char err[1000] = "";
  mjModel * m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (!m) { std::fprintf(stderr, "load failed: %s\n", err); return 2; }
  mjData * d = mj_makeData(m);
  int sid = mj_name2id(m, mjOBJ_SITE, "attachment_site");
  auto rows = read_csv(argv[2]);
  std::vector<Result> res(rows.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> jp(3, m->nv), jr(3, m->nv);
  volatile double sink = 0;

  auto solve = [&](const std::vector<double> & r, int & iters, double & chk) {
    Eigen::Vector3d pt(r[0], r[1], r[2]);
    Eigen::Matrix3d Rt = mat3(r, 3);
    Vec6 q0 = q_home();
    for (int i = 0; i < 6; ++i) d->qpos[i] = q0[i];
    iters = IK_MAX_ITER;
    for (int it = 0; it < IK_MAX_ITER; ++it) {
      mj_kinematics(m, d);
      mj_comPos(m, d);
      Eigen::Map<const Eigen::Vector3d> p(d->site_xpos + 3 * sid);
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(d->site_xmat + 9 * sid);
      Vec6 e;
      e << pt - p, rot_err(R, Rt);
      if (e.norm() < IK_TOL) { iters = it; break; }
      mj_jacSite(m, d, jp.data(), jr.data(), sid);
      Mat6 J;
      J << jp, jr;
      Mat6 M = J * J.transpose() + IK_LAMBDA * IK_LAMBDA * Mat6::Identity();
      Vec6 dq = J.transpose() * M.partialPivLu().solve(e);
      for (int i = 0; i < 6; ++i) d->qpos[i] += dq[i];
    }
    chk = 0;
    for (int i = 0; i < 6; ++i) chk += d->qpos[i];
  };

  for (auto & r : rows) { int it; double c; solve(r, it, c); sink = sink + c; }
  for (size_t i = 0; i < rows.size(); ++i) {
    int it; double c;
    double t0 = now_ns();
    solve(rows[i], it, c);
    res[i] = {now_ns() - t0, (double)it, c};
    sink = sink + c;
  }
  write_results(argv[3], res);
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
