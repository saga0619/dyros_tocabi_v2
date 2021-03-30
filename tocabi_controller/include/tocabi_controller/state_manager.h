#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "tocabi_controller/robot_data.h"
#include <ros/ros.h>
#include <ros/package.h>

void init_shm();

class StateManager
{
public:
    StateManager(DataContainer &dc_global);
    ~StateManager();

    void *stateThread(void);
    static void *thread_helper(void *context)
    {
        return ((StateManager *)context)->stateThread();
    }
    void getJointData();
    void getSensorData();
    void storeState();

    void stateEstimate();
    //private functions

    void updateKinematics_local(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);
    //update kinematic information with RBDL
    void updateKinematics(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);

    LinkData link_[LINK_NUMBER + 1];
    LinkData link_local_[LINK_NUMBER + 1];
    int link_id_[LINK_NUMBER + 1];

    Eigen::MatrixVVd A_;
    Eigen::MatrixVVd A_inv;
    Eigen::MatrixXd A_temp_;

    RigidBodyDynamics::Model model_, model_2;

    DataContainer &dc_;
    RobotData rd_;

    float q_a_[MODEL_DOF];
    float q_dot_a_[MODEL_DOF];

    Eigen::VectorQVQd q_virtual_local_;
    Eigen::VectorVQd q_dot_virtual_local_;
    Eigen::VectorVQd q_ddot_virtual_local_;

    //Eigen::VectorXf<MODEL_DOF_VIRTUAL> q_virtual_;
    Eigen::VectorQd q_;
    Eigen::VectorQd q_dot_;
    Eigen::VectorQd q_ddot_;

    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;
};