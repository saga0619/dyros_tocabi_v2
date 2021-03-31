#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "tocabi_controller/tocabi.h"
#include "math_type_define.h"

using namespace std;
using namespace Eigen;

class LinkData
{
public:
  // Update link i of rbdl link id. name : link name, mass : link mass, xipos : local center of mass position
  void initialize(RigidBodyDynamics::Model &model_, int id_, std::string name_, double mass, Eigen::Vector3d &local_com_position);

  // Update COM jacobian
  void COM_Jac_Update(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_);

  // Update xpos, xipos, rotm.
  void pos_Update(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_);

  // Set Contact point, Contact jacobian
  //void Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);

  // Set Contact point, Contact jacobian
  //void Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::VectorVQd &q_dot_virtual, Eigen::Vector3d &Contact_position);

  // Set Sensor Position
  //void Set_Sensor_Position(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Sensor_position);

  // update Jacobian matrix of local position at link.
  void Set_Jacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position);

  // update Point Jacobian matrix of local position at link.
  void Set_Jacobian_custom(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position);

  // update link velocity(6D, translation and rotation) from jacobian matrix Jac.
  void vw_Update(const Eigen::VectorVQd &q_dot_virtual);

  // set link Trajectory of id i.
  void Set_Trajectory(Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d acc_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired, Eigen::Vector3d acc_des);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

  // set realtime trajectory of rotation of link
  void Set_Trajectory_rotation(double current_time, double start_time, double end_time, bool local_);

  // set realtime trajectory of rotation of link
  void Set_Trajectory_rotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired, bool local_);

  // set link initial position and rotation. initial position for task control.
  void Set_initpos();

  void Set_initpos_local();

  void Set_initTask();

  bool Check_name(RigidBodyDynamics::Model &model_);

  //void Get_PointPos(Eigen::VectorQVQd &q_virtual_, Eigen::VectorVQd &q_dot_virtual, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos, Eigen::Vector6d &global_velocity6D);

  //constant variables
  int id;
  double Mass;
  std::string name;

  //local COM position of body
  Eigen::Vector3d COM_position;

  //inertial matrix
  Eigen::Matrix3d inertia;

  //local sensor point
  Eigen::Vector3d sensor_point;

  //changing variables
  //rotation matrix
  Eigen::Matrix3d Rotm;

  //global position of body
  Eigen::Vector3d xpos;

  //global COM position of body
  Eigen::Vector3d xipos;

  //global position of sensor at body
  Eigen::Vector3d xpos_sensor;

  //cartesian velocity of body
  Eigen::Vector3d v;

  //rotational velocity of body
  Eigen::Vector3d w;

  //fstar of current link
  Eigen::Vector6d fstar;

  Eigen::Matrix6Vf Jac;
  Eigen::Matrix6Vf Jac_COM;


  //realtime traj of cartesian & orientation.
  //)) traj is outcome of cubic or quintic function, which will be used to make fstar!
  // x : cartesian coordinate traj (3x1)
  // v : cartesian velocity (3x1)
  // r : rotational matrix of current orientation (3x3)
  // w : rotational speed of current orientation (3x1)

  Eigen::Vector3d x_traj;
  Eigen::Vector3d v_traj;
  Eigen::Vector3d a_traj;

  Eigen::Matrix3d r_traj;
  Eigen::Vector3d w_traj;
  Eigen::Vector3d ra_traj;

  Eigen::Vector3d x_traj_local;
  Eigen::Vector3d v_traj_local;
  Eigen::Matrix3d r_traj_local;
  Eigen::Vector3d w_traj_local;

  Eigen::Vector3d x_init;
  Eigen::Vector3d v_init;
  Eigen::Matrix3d rot_init;
  Eigen::Vector3d w_init;

  Eigen::Vector3d x_init_local;
  Eigen::Vector3d v_init_local;
  Eigen::Matrix3d rot_init_local;
  Eigen::Vector3d w_init_local;

  Eigen::Vector3d x_task_init;
  Eigen::Vector3d v_task_init;
  Eigen::Vector3d a_task_init;
  Eigen::Matrix3d r_task_init;
  Eigen::Vector3d w_task_init;

  Eigen::Vector3d x_desired;
  Eigen::Matrix3d rot_desired;

  Eigen::Vector3d pos_p_gain;
  Eigen::Vector3d pos_d_gain;
  Eigen::Vector3d rot_p_gain;
  Eigen::Vector3d rot_d_gain;
  Eigen::Vector3d acc_p_gain;

  Eigen::Vector3d max_p_acc_;
  Eigen::Vector3d max_p_vel_;

  //RigidBodyDynamics::Model *model;

  Eigen::MatrixXd j_temp;
};

class EndEffector : public LinkData
{
public:
  // Set Contact point, Contact jacobian
  void Set_Contact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);
  //cartesian velocity of contact point at body
  Eigen::Vector3d v_contact;
  //cartesian velocity of contact point at body
  Eigen::Vector3d w_contact;
  Eigen::Matrix6Vf Jac_Contact;
  //global position of contact point at body
  Eigen::Vector3d xpos_contact;
  //local contact point
  Eigen::Vector3d contact_point;
};
