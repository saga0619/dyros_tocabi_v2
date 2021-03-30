#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <ros/ros.h>

#include "tocabi_controller/link.h"
#include "tocabi_ecat/shm_msgs.h"

using namespace std;


struct RobotData
{

    float control_time_;
    RigidBodyDynamics::Model model_;
    LinkData link_[LINK_NUMBER + 1];

    Eigen::MatrixVVf A_;
    Eigen::MatrixVVf A_inv_;
    Eigen::MatrixVVf Motor_inertia;
    Eigen::MatrixVVf Motor_inertia_inverse;

    Eigen::VectorQd q_;
    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorQd q_dot_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_dot_virtual_lpf;
    Eigen::VectorVQd q_ddot_virtual_;
    Eigen::VectorQd q_ext_;
    Eigen::VectorQd q_dot_est_;
    Eigen::VectorQd q_dot_est_1;
    Eigen::VectorQd q_hold_lower_;
    Eigen::VectorQd torque_elmo_;

    Eigen::VectorQd torque_desired;
};

struct DataContainer
{
    ros::NodeHandle nh;
    RobotData rd_;
};