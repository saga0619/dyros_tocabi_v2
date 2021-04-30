
#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
  rbdl_check_api_version (RBDL_API_VERSION);

  Model model;

  if (argc != 2) {
    std::cerr << "Error: not right number of arguments." << std::endl;
    std::cerr << "usage: " << argv[0] << " <model.urdf>" << std::endl;
    exit(-1);
  }

  if (!Addons::URDFReadFromFile ("/home/saga/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/test_robot.urdf", &model, false)) {
    std::cerr << "Error loading model " << argv[1] << std::endl;
    abort();
  }

  std::cout << "Degree of freedom overview:" << std::endl;
  std::cout << Utils::GetModelDOFOverview(model);

  std::cout << "Model Hierarchy:" << std::endl;
  std::cout << Utils::GetModelHierarchy(model);

  VectorNd Q = VectorNd::Zero (model.q_size);
  VectorNd QDot = VectorNd::Zero (model.qdot_size);
  VectorNd Tau = VectorNd::Zero (model.qdot_size);
  VectorNd QDDot = VectorNd::Zero (model.qdot_size);

  std::cout << "Forward Dynamics with q, qdot, tau set to zero:" << std::endl;
  ForwardDynamics (model, Q, QDot, Tau, QDDot);

  std::cout << QDDot.transpose() << std::endl;


  return 0;
}

