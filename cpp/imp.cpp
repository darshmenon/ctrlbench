// Cartesian impedance law tau = J^T (K e - D J v) + g. Two modes:
//   law  precomputed J, g, e  (pure linear algebra, Eigen)
//   pin  full Pinocchio path: FK, frame Jacobian, gravity, rotation error, then the law
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/math/rotation.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include "common.hpp"
using namespace cb;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

int main(int argc, char ** argv) {
  if (argc != 5) { std::fprintf(stderr, "usage: imp law|pin model.xml states.csv out.csv\n"); return 2; }
  std::string mode = argv[1];
  auto rows = read_csv(argv[3]);
  std::vector<Result> res(rows.size());
  const Vec6 K = k_gain(), D = d_gain();
  volatile double sink = 0;

  if (mode == "law") {
    auto one = [&](const std::vector<double> & r) {
      Vec6 v = Eigen::Map<const Vec6>(&r[6]);
      Mat6 J = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(&r[12]);
      Vec6 g = Eigen::Map<const Vec6>(&r[48]);
      Vec6 e = Eigen::Map<const Vec6>(&r[54]);
      Vec6 tau = J.transpose() * (K.cwiseProduct(e) - D.cwiseProduct(J * v)) + g;
      return tau.sum();
    };
    for (auto & r : rows) sink = sink + one(r);
    for (size_t i = 0; i < rows.size(); ++i) {
      double t0 = now_ns();
      double c = one(rows[i]);
      res[i] = {now_ns() - t0, 0, c};
      sink = sink + c;
    }
  } else {
    pinocchio::Model model;
    pinocchio::mjcf::buildModel(argv[2], model);
    pinocchio::Data data(model);
    auto fid = model.getFrameId("attachment_site");
    auto one = [&](const std::vector<double> & r) {
      Eigen::VectorXd q = Eigen::Map<const Vec6>(&r[0]);
      Vec6 v = Eigen::Map<const Vec6>(&r[6]);
      Eigen::Vector3d xd(r[60], r[61], r[62]);
      Eigen::Matrix3d Rd = mat3(r, 63);
      pinocchio::framesForwardKinematics(model, data, q);
      const auto & oMf = data.oMf[fid];
      Vec6 e;
      e << xd - oMf.translation(), pinocchio::log3(Eigen::Matrix3d(Rd * oMf.rotation().transpose()));
      pinocchio::Data::Matrix6x J(6, model.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(model, data, q, fid, pinocchio::LOCAL_WORLD_ALIGNED, J);
      const Eigen::VectorXd & g = pinocchio::computeGeneralizedGravity(model, data, q);
      Vec6 tau = J.transpose() * (K.cwiseProduct(e) - D.cwiseProduct(J * v)) + g;
      return tau.sum();
    };
    for (auto & r : rows) sink = sink + one(r);
    for (size_t i = 0; i < rows.size(); ++i) {
      double t0 = now_ns();
      double c = one(rows[i]);
      res[i] = {now_ns() - t0, 0, c};
      sink = sink + c;
    }
  }
  write_results(argv[4], res);
  return 0;
}
