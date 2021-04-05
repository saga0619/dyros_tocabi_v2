#include <tocabi_controller/link.h>

void LinkData::Initialize(RigidBodyDynamics::Model &model_, int id_, std::string name_, double mass_, Eigen::Vector3d &local_com_position)
{
    id = id_;
    mass = mass_;
    com_position = local_com_position;
    name = name_;
    rotm.setZero();
    inertia.setZero();

    inertia = model_.mBodies[id_].mInertia;
    jac.setZero();
    jac_com.setZero();

    j_temp.setZero(6, MODEL_DOF_VIRTUAL);
}

bool LinkData::CheckName(RigidBodyDynamics::Model &model_)
{
    return (model_.GetBodyName(id) == name);
}

void LinkData::UpdatePosition(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_)
{
    xpos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, Eigen::Vector3d::Zero(), false);
    xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, com_position, false);
    rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, id, false)).transpose();
}

void LinkData::UpdateVW(const Eigen::VectorVQd &q_dot_virtual)
{
    Eigen::Vector6d vw;
    vw = jac.cast<double>() * q_dot_virtual;
    v = vw.segment(0, 3);
    w = vw.segment(3, 3);
}

void LinkData::UpdateJacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_)
{
    j_temp.setZero();

    jac_com.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, model_.mBodies[id].mCenterOfMass, j_temp, false);

    jac_com.block(0, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL).cast<float>(); //*E_T_;
    jac_com.block(3, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL).cast<float>();

    j_temp.setZero();

    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Eigen::Vector3d::Zero(), j_temp, false);

    jac.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0).cast<float>();
    jac.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0).cast<float>();
}

void LinkData::SetTrajectory(Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired)
{
    x_traj = position_desired;
    v_traj = velocity_desired;
    r_traj = rotation_desired;
    w_traj = rotational_velocity_desired;
}

void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time)
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

void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired)
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

void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired)
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

void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired)
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

void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d acc_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired, Eigen::Vector3d acc_des)
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

void LinkData::SetTrajectoryRotation(double current_time, double start_time, double end_time, bool local_)
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

void LinkData::SetTrajectoryRotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired_, bool local_)
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

void LinkData::SetInitialWithPosition()
{
    x_init = xpos;
    v_init = v;
    rot_init = rotm;
    w_init = w;
}

void LinkData::SetInitialWithTrajectory()
{
    x_init = x_traj;
    v_init = v_traj;
    rot_init = r_traj;
    w_init = w_traj;
}

void EndEffector::SetContact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_)
{
    j_temp.setZero();

    //mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, contact_point, j_temp, false);

    xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, contact_point, false);

    //mtx_rbdl.unlock();
    // jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
    // jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
    jac_contact.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0).cast<float>();
    jac_contact.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0).cast<float>();

    // jac_Contact.block<3,3>(0,3)= -
    // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,id,Contact_position,false)
    // - link_[0].xpos);
}

void EndEffector::Initialize(LinkData &lk_, float x_length, float y_length, float min_force, float friction_ratio_, float friction_ratio_z_)
{
    id = lk_.id;
    mass = lk_.mass;
    com_position = lk_.com_position;
    name = lk_.name;
    rotm.setZero();
    inertia.setZero();

    inertia = lk_.inertia;
    jac.setZero();
    jac_com.setZero();

    cs_x_length = x_length;
    cs_y_length = y_length;
    contact_force_minimum = min_force;
    friction_ratio = friction_ratio_;
    friction_ratio_z = friction_ratio_z_;

    j_temp.setZero(6, MODEL_DOF_VIRTUAL);
}

void EndEffector::UpdateLinkData(LinkData &lk_)
{
    xpos = lk_.xpos;
    xipos = lk_.xipos;

    v = lk_.v;
    w = lk_.w;

    rotm = lk_.rotm;

    memcpy(&jac, &lk_.jac, sizeof(Matrix6Vf));
    memcpy(&jac_com, &lk_.jac_com, sizeof(Matrix6Vf));
}