#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/geometry.hpp"

#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "hpp/fcl/shape/geometric_shapes.h"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/fcl.hpp"

#include "pinocchio/collision/collision.hpp"
#include <string>

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>

void print_vector(const std::vector<double> &vec) {
  for (auto &v : vec) {
    std::cout << v << " ";
  }
  std::cout << std::endl;
}

int main() {

  Eigen::VectorXd q(7);
  q << 0.000, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000;

  std::string urdf =
      "/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.urdf";
  std::string srdf =
      "/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.srdf";

  pinocchio::Model model;
  pinocchio::Data data;

  pinocchio::urdf::buildModel(urdf, model);
  data = pinocchio::Data(model);

  pinocchio::computeGeneralizedGravity(model, data, q);

  std::cout << "Pinocchio" << data.g.transpose() << std::endl;

  std::string robot_sn = "Rizon10s-062147";

  flexiv::rdk::Robot robot(robot_sn);

  flexiv::rdk::Model f_model(robot);

  std::vector<double> qv(q.data(), q.data() + q.size());
  std::vector<double> vel(7, 0.);

  f_model.Update(qv, vel);

  // Compute gravity vector
  auto g = f_model.g();

  std::cout << "g from them " << g.transpose() << std::endl;

  std::cout << "Diference" << std::endl;
  std::cout << g - data.g << std::endl;

  Eigen::VectorXd relative_difference = (g - data.g)
                                            .cwiseAbs()
                                            .cwiseQuotient(
                                                g.cwiseAbs());
  std::cout << relative_difference << std::endl;
}
