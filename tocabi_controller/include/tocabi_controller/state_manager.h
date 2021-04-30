#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <thread>
#include <future>

#include "tocabi_controller/robot_data.h"
#include "mujoco_ros_msgs/SimStatus.h"
#include "mujoco_ros_msgs/JointSet.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

class StateManager
{
public:
    StateManager(DataContainer &dc_global);
    ~StateManager();

    void *stateThread();
    static void *thread_starter(void *context) { return ((StateManager *)context)->stateThread(); }
    void getJointData();
    void getSensorData();
    void storeState(RobotData &robotd_);
    void calcNonlinear();

    void stateEstimate();
    //private functions

    void updateKinematics_local(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);
    //update kinematic information with RBDL
    void updateKinematics(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);

    void publishData();

    LinkData link_[LINK_NUMBER + 1];
    LinkData link_local_[LINK_NUMBER + 1];
    int link_id_[LINK_NUMBER + 1];

    Eigen::MatrixVVd A_;
    Eigen::MatrixVVd A_inv_;
    Eigen::MatrixXd A_temp_;
    Eigen::MatrixVVd Motor_inertia;
    Eigen::MatrixVVd Motor_inertia_inverse;

    RigidBodyDynamics::Model model_, model_2;

    DataContainer &dc_;
    RobotData rd_;

    float q_a_[MODEL_DOF] = {};
    float q_dot_a_[MODEL_DOF] = {};

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

    float control_time_ = 0;

    double total_mass_ = 0;
    tf2_ros::TransformBroadcaster br;

    //Simmode values..

    void ConnectSim();
    void GetSimData();
    void SendCommand(VectorQf command, std::vector<bool> command_mode);

    ros::Subscriber mujoco_sim_command_sub_;
    ros::Publisher mujoco_sim_command_pub_;

    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_msg_;

    ros::Subscriber task_command_que_sub_;
    tocabi_msgs::TaskCommandQue tc_que_msg_;
    ros::Subscriber task_command_sub_;
    tocabi_msgs::TaskCommand tc_msg_;

    void simCommandCallback(const std_msgs::StringConstPtr &msg);
    //void simStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg);

    void TaskCommandCallback(const tocabi_msgs::TaskCommandConstPtr &msg);
    void TaskQueCommandCallback(const tocabi_msgs::TaskCommandQueConstPtr &msg);
    void GuiCommandCallback(const std_msgs::StringConstPtr &msg);

    float sim_time_ = 0;
    float sim_time_before_ = 0;

    bool mujoco_ready = false;
    bool mujoco_init_receive = false;
    bool mujoco_reset = false;
};

#endif