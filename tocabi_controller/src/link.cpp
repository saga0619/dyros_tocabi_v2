#include <tocabi_controller/link.h>

void LinkData::initialize(RigidBodyDynamics::Model &model_, int id_, std::string name_, double mass, Eigen::Vector3d &local_com_position)
{
    id = id_;
    Mass = mass;
    COM_position = local_com_position;
    name = name_;
    Rotm.setZero();
    inertia.setZero();
    //contact_point.setZero();
    inertia = model_.mBodies[id_].mInertia;
    Jac.setZero();
    Jac_COM.setZero();
    //Jac_Contact.setZero();

    j_temp.setZero(6, MODEL_DOF_VIRTUAL);
}

void LinkData::pos_Update(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_)
{
    //mtx_rbdl.lock();
    xpos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, Eigen::Vector3d::Zero(), false);
    xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, COM_position, false);
    Rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, id, false)).transpose();
    //mtx_rbdl.unlock();
    // COM_position =
    // RigidBodyDynamics::CalcBaseToBodyCoordinates(model_,q_virtual_,link_[i])
}

bool LinkData::Check_name(RigidBodyDynamics::Model &model_)
{
    return (model_.GetBodyName(id) == name);
}

void LinkData::COM_Jac_Update(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_)
{
    //mtx_rbdl.lock();
    Jac_COM.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, model_.mBodies[id].mCenterOfMass, j_temp, false);

    Jac_COM.block(0, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL).cast<float>(); //*E_T_;
    Jac_COM.block(3, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL).cast<float>();
}

void LinkData::Set_Jacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position)
{
    j_temp.setZero();
    //0 1  A
    //1 0  B
    //mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Jacobian_position, j_temp, false);

    //mtx_rbdl.unlock();
    Jac.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0).cast<float>();
    Jac.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0).cast<float>();
}

/*
void LinkData::Set_Jacobian_custom(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position)
{
    j_temp.setZero();

    //mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Jacobian_position, j_temp, false);

    //mtx_rbdl.unlock();
    Jac_point.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0);
    Jac_point.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0);
}*/
/*
void LinkData::Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position)
{
    j_temp.setZero();
    //mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(*model, q_virtual_, id, Contact_position, j_temp, false);
    xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model, q_virtual_, id, Contact_position, false);

    //mtx_rbdl.unlock();
    // Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
    // Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
    Jac_Contact.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0).cast<float>();
    Jac_Contact.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0).cast<float>();

    // Jac_Contact.block<3,3>(0,3)= -
    // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,id,Contact_position,false)
    // - link_[0].xpos);
}

void LinkData::Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::VectorVQd &q_dot_virtual, Eigen::Vector3d &Contact_position)
{
    j_temp.setZero();
    //mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(*model, q_virtual_, id, Contact_position, j_temp, false);
    xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model, q_virtual_, id, Contact_position, false);

    //mtx_rbdl.unlock();
    // Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
    // Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
    Jac_Contact.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0).cast<float>();
    Jac_Contact.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0).cast<float>();

    v_contact = Jac_Contact.block(0, 0, 3, MODEL_DOF_VIRTUAL).cast<double>() * q_dot_virtual;

    w_contact = Jac_Contact.block(3, 0, 3, MODEL_DOF_VIRTUAL).cast<double>() * q_dot_virtual;

    // Jac_Contact.block<3,3>(0,3)= -
    // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,id,Contact_position,false)
    // - link_[0].xpos);
}

void LinkData::Get_PointPos(Eigen::VectorQVQd &q_virtual_, Eigen::VectorVQd &q_dot_virtual, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos, Eigen::Vector6d &global_velocity6D)
{
    //mtx_rbdl.lock();
    global_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model, q_virtual_, id, local_pos, false);
    global_velocity6D = RigidBodyDynamics::CalcPointVelocity6D(*model, q_virtual_, q_dot_virtual, id, local_pos, false);

    //mtx_rbdl.unlock();
}

*/

void LinkData::vw_Update(const Eigen::VectorVQd &q_dot_virtual)
{
    Eigen::Vector6d vw;
    vw = Jac.cast<double>() * q_dot_virtual;
    v = vw.segment(0, 3);
    w = vw.segment(3, 3);
}

void LinkData::Set_Trajectory(Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired)
{
    x_traj = position_desired;
    v_traj = velocity_desired;
    r_traj = rotation_desired;
    w_traj = rotational_velocity_desired;
}

void LinkData::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, x_init(j), v_init(j), 0, x_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, x_init(j), v_init(j), 0, pos_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, pos_init(j), 0, 0, pos_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, pos_init(j), vel_init(j), 0, pos_desired(j), vel_desired(j), 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d acc_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired, Eigen::Vector3d acc_des)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, pos_init(j), vel_init(j), acc_init(j), pos_desired(j), vel_desired(j), acc_des(j));
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::Set_Trajectory_rotation(double current_time, double start_time, double end_time, bool local_)
{
    //if local_ is true, local based rotation control
    Eigen::Vector3d axis;
    double angle;
    if (local_)
    {
        Eigen::AngleAxisd aa(rot_desired);
        axis = aa.axis();
        angle = aa.angle();
    }
    else
    {
        Eigen::AngleAxisd aa(rot_init.transpose() * rot_desired);
        axis = aa.axis();
        angle = aa.angle();
    }
    // double c_a = DyrosMath::cubic(current_time, start_time, end_time, 0.0, angle, 0.0, 0.0);
    // Eigen::Matrix3d rmat;
    // rmat = Eigen::AngleAxisd(c_a, axis);

    // r_traj = rot_init * rmat;

    // double dtime = 0.001;
    // double c_a_dtime = DyrosMath::cubic(current_time + dtime, start_time, end_time, 0.0, angle, 0.0, 0.0);

    // Eigen::Vector3d ea = r_traj.eulerAngles(0, 1, 2);

    // Eigen::Vector3d ea_dtime = (rot_init * Eigen::AngleAxisd(c_a_dtime, axis)).eulerAngles(0, 1, 2);

    // w_traj = (ea_dtime - ea) / dtime;
    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, 0.0, 0.0, 0.0, angle, 0.0, 0.0);
    double c_a = quintic(0);
    Eigen::Matrix3d rmat;

    rmat = Eigen::AngleAxisd(c_a, axis);
    r_traj = rot_init * rmat;
    w_traj = quintic(1) * axis;
    ra_traj = quintic(2) * axis;
}

void LinkData::Set_Trajectory_rotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired_, bool local_)
{
    Eigen::Vector3d axis;
    double angle;
    if (local_)
    {
        Eigen::AngleAxisd aa(rot_desired_);
        axis = aa.axis();
        angle = aa.angle();
    }
    else
    {
        Eigen::AngleAxisd aa(rot_init.transpose() * rot_desired_);
        axis = aa.axis();
        angle = aa.angle();
    }
    double c_a = DyrosMath::cubic(current_time, start_time, end_time, 0.0, angle, 0.0, 0.0);
    Eigen::Matrix3d rmat;
    rmat = Eigen::AngleAxisd(c_a, axis);

    r_traj = rot_init * rmat;

    double dtime = 0.0001;
    double c_a_dtime = DyrosMath::cubic(current_time + dtime, start_time, end_time, 0.0, angle, 0.0, 0.0);

    Eigen::Vector3d ea = r_traj.eulerAngles(0, 1, 2);

    Eigen::Vector3d ea_dtime = (rot_init * Eigen::AngleAxisd(c_a_dtime, axis)).eulerAngles(0, 1, 2);

    w_traj = (ea_dtime - ea) / dtime;
}

void LinkData::Set_initpos()
{
    x_init = xpos;
    v_init = v;
    rot_init = Rotm;
    w_init = w;
}

void LinkData::Set_initTask()
{
    x_init = x_traj;
    v_init = v_traj;
    rot_init = r_traj;
    w_init = w_traj;
}

void EndEffector::Set_Contact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position)
{
    j_temp.setZero();

    //mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Contact_position, j_temp, false);

    xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, Contact_position, false);

    //mtx_rbdl.unlock();
    // Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
    // Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
    Jac_Contact.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0).cast<float>();
    Jac_Contact.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0).cast<float>();

    // Jac_Contact.block<3,3>(0,3)= -
    // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,id,Contact_position,false)
    // - link_[0].xpos);
}