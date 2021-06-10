#include <tocabi_data/link.h>

void LinkData::Initialize(RigidBodyDynamics::Model &model_, int id_)
{
    id = id_;
    mass = model_.mBodies[id_].mMass;

    com_position = model_.mBodies[id_].mCenterOfMass;
    inertia = model_.mBodies[id_].mInertia;

    pos_p_gain << 400, 400, 400;
    pos_d_gain << 40, 40, 40;
    pos_a_gain << 1, 1, 1;

    rot_p_gain << 400, 400, 400;
    rot_d_gain << 40, 40, 40;
    rot_a_gain << 1, 1, 1;
}

void LinkData::UpdatePosition(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_)
{
    xpos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, Eigen::Vector3d::Zero(), false);
    xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, com_position, false);
    rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, id, false)).transpose();
}

void LinkData::UpdateVW(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, const Eigen::VectorVQd &q_dot_virtual_)
{
    Eigen::Vector6d vw = RigidBodyDynamics::CalcPointVelocity6D(model_, q_virtual_, q_dot_virtual_, id, Eigen::Vector3d::Zero(), false);

    v = vw.segment(3, 3);
    w = vw.segment(0, 3);
}

void LinkData::GetPointPos(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, const Eigen::VectorVQd &q_dot_virtual_, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos, Eigen::Vector6d &global_velocity6D)
{
    global_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, local_pos, false);
    global_velocity6D = RigidBodyDynamics::CalcPointVelocity6D(model_, q_virtual_, q_dot_virtual_, id, local_pos, false);
}

Eigen::Matrix6Vd LinkData::Jac()
{

    return jac.cast<Eigen::rScalar>();
}
Eigen::Matrix6Vd LinkData::JacCOM()
{
    return jac_com.cast<Eigen::rScalar>();
}

void LinkData::UpdateJacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_)
{
    j_temp.setZero(6, MODEL_DOF_VIRTUAL);

    jac_com.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, model_.mBodies[id].mCenterOfMass, j_temp, false);

    jac_com.block(0, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>(); //*E_T_;
    jac_com.block(3, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>();

    j_temp.setZero();

    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Eigen::Vector3d::Zero(), j_temp, false);

    jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>();
    jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>();
}

void LinkData::UpdateJacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, const Eigen::VectorVQd &q_dot_virtual_)
{
    j_temp.setZero(6, MODEL_DOF_VIRTUAL);

    jac_com.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, model_.mBodies[id].mCenterOfMass, j_temp, false);

    jac_com.block(0, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>(); //*E_T_;
    jac_com.block(3, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>();

    j_temp.setZero();

    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Eigen::Vector3d::Zero(), j_temp, false);

    jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>();
    jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = j_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL).cast<Eigen::lScalar>();

    Eigen::Vector6d vw = RigidBodyDynamics::CalcPointVelocity6D(model_, q_virtual_, q_dot_virtual_, id, Eigen::Vector3d::Zero(), false);

    v = vw.segment(3, 3);
    w = vw.segment(0, 3);
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

void LinkData::SetTrajectoryRotation(double current_time, double start_time, double end_time)
{
    Eigen::Quaterniond q0(rot_init);
    Eigen::Quaterniond q1(rot_desired);

    Eigen::Vector3d qs_ = DyrosMath::QuinticSpline(current_time, start_time, end_time, 0, 0, 0, 1, 0, 0);

    Eigen::Quaterniond q_traj = q0.slerp(qs_(0), q1);

    r_traj = q_traj.toRotationMatrix();

    Eigen::AngleAxisd axd(q1 * q0.inverse());

    w_traj = axd.angle() * qs_(1) * axd.axis();
    ra_traj = axd.angle() * qs_(2) * axd.axis();
}

void LinkData::SetGain(double pos_p, double pos_d, double pos_a, double rot_p, double rot_d, double rot_a)
{
    pos_p_gain << pos_p, pos_p, pos_p;
    pos_d_gain << pos_d, pos_d, pos_d;
    pos_a_gain << pos_a, pos_a, pos_a;

    rot_p_gain << rot_p, rot_p, rot_p;
    rot_d_gain << rot_d, rot_d, rot_d;
    rot_a_gain << rot_a, rot_a, rot_a;
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
    j_temp.setZero(6, MODEL_DOF_VIRTUAL);

    //mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, contact_point, j_temp, false);

    xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, contact_point, false);

    //mtx_rbdl.unlock();
    // jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
    // jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
    jac_contact.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0).cast<Eigen::lScalar>();
    jac_contact.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0).cast<Eigen::lScalar>();

    // jac_Contact.block<3,3>(0,3)= -
    // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,id,Contact_position,false)
    // - link_[0].xpos);
}

void EndEffector::InitializeEE(LinkData &lk_, float x_length, float y_length, float min_force, float friction_ratio_, float friction_ratio_z_)
{
    id = lk_.id;
    mass = lk_.mass;
    com_position = lk_.com_position;
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