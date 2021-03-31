#include "tocabi_controller/state_manager.h"
#include <thread>

using namespace std;

StateManager::StateManager(DataContainer &dc_global) : dc_(dc_global)
{
    cout << "Init StateManager" << endl;
    init_shm();

    string t_path_ = ros::package::getPath("tocabi_description");
    string urdf_path = t_path_ + "/robots/dyros_tocabi.urdf";

    bool verbose = false;

    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true, verbose);
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_2, true, verbose);

    if (model_.dof_count == MODEL_DOF_VIRTUAL)
    {
        A_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_id_[i] = model_.GetBodyId(TOCABI::LINK_NAME[i]);

            if (!model_.IsBodyId(link_id_[i]))
            {
                ROS_INFO_COND(verbose, "Failed to get body id at link %d : %s", i, TOCABI::LINK_NAME[i]);
            }
        }

        double total_mass = 0;

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_[i].initialize(model_2, link_id_[i], TOCABI::LINK_NAME[i], model_2.mBodies[link_id_[i]].mMass, model_2.mBodies[link_id_[i]].mCenterOfMass);
            total_mass += link_[i].Mass;
        }

        
        /*
        link_[Right_Foot].contact_point << 0.03, 0, -0.1585;
        link_[Right_Foot].sensor_point << 0.0, 0.0, -0.09;
        link_[Left_Foot].contact_point << 0.03, 0, -0.1585;
        link_[Left_Foot].sensor_point << 0.0, 0.0, -0.09;

        link_[Right_Hand].contact_point << 0, 0.0, -0.035;
        link_[Right_Hand].sensor_point << 0.0, 0.0, 0.0;
        link_[Left_Hand].contact_point << 0, 0.0, -0.035;
        link_[Left_Hand].sensor_point << 0.0, 0.0, 0.0;
        */      


        memcpy(link_local_, link_, sizeof(LinkData) * LINK_NUMBER);
    }
}

StateManager::~StateManager()
{
    cout << "StateManager Terminate" << endl;
}

void *StateManager::stateThread(void)
{
    cout << "StateManager Thread Entered" << endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    volatile int rcv_tcnt = 0;
    rcv_tcnt = shm_msgs_->t_cnt;
    cout << "first packet " << rcv_tcnt << endl;
    volatile int cycle_count_ = rcv_tcnt;
    volatile int cc_zero_ = 0;

    RobotData_origin test_, test2_;

    while (!shm_msgs_->shutdown)
    {
        cycle_count_++;
        cc_zero_++;
        while (rcv_tcnt >= shm_msgs_->t_cnt)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        if (rcv_tcnt + 1 != shm_msgs_->t_cnt)
        {
            //std::cout << "missed packet : " << shm_msgs_->t_cnt - rcv_tcnt << std::endl;
        }
        rcv_tcnt = shm_msgs_->t_cnt;

        if (true) //dc.imu_ignore == true)
        {
            for (int i = 0; i < 6; i++)
            {
                q_virtual_local_(i) = 0.0;
                q_dot_virtual_local_(i) = 0.0;
                q_ddot_virtual_local_(i) = 0.0;
            }
            q_virtual_local_(MODEL_DOF + 6) = 1.0;
        }

        //cout<<"?"<<endl;

        getJointData();

        //cout<<"?"<<endl;
        updateKinematics_local(model_, link_local_, q_virtual_local_, q_dot_virtual_local_, q_ddot_virtual_local_);

        stateEstimate();

        updateKinematics(model_2, link_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);


        auto t1 = chrono::steady_clock::now();

        storeState(dc_.rd_);

        auto d1 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t1);

        static int64_t total = 0;
        static int max = 0;
        static int min = 10000000;
        static float avg = 0;
        static int cnt = 0;
        static int cnt2 = 0;
        static int cnt3 = 0;
        total += d1.count();

        if (d1.count() > 15)
        {
            cnt3 = d1.count();
            cnt++;
            cnt2 = rcv_tcnt;
        }

        if (max < d1.count())
            max = d1.count();
        if (min > d1.count())
            min = d1.count();

        avg = (float)total / cc_zero_;

        //printf("%d\n", rcv_tcnt);
        //printf("\x1b[A\x1b[A\33[2K\r");
        if (rcv_tcnt % 33 == 0)
        {
            printf("\33[2K\r");
            printf("%8d %8d avg : %7.3f max : %4d min : %4d, cnt : %4d, cnt2 : %4d, cnt3 : %4d ", rcv_tcnt, cycle_count_, avg, max, min, cnt, cnt2, cnt3);
            fflush(stdout);
        }
    }
}

void StateManager::getJointData()
{
    memcpy(q_a_, shm_msgs_->pos, sizeof(float) * MODEL_DOF);
    memcpy(q_dot_a_, shm_msgs_->vel, sizeof(float) * MODEL_DOF);

    q_ = Map<VectorQf>(q_a_, MODEL_DOF).cast<double>();
    q_dot_ = Map<VectorQf>(q_dot_a_, MODEL_DOF).cast<double>();

    q_virtual_local_.segment(6, MODEL_DOF) = q_;
    q_dot_virtual_local_.segment(6, MODEL_DOF) = q_dot_;

    //shm_msgs_->pos
}

void StateManager::getSensorData()
{
}

void StateManager::storeState(RobotData &rd_dst)
{
    memcpy(&rd_dst.model_, &model_, sizeof(RigidBodyDynamics::Model));

    for (int i = 0; i < (LINK_NUMBER + 1); i++)
    {
        memcpy(&rd_dst.link_[i].Jac, &link_[i].Jac, sizeof(Matrix6Vf));
        memcpy(&rd_dst.link_[i].Jac_COM, &link_[i].Jac_COM, sizeof(Matrix6Vf));

        memcpy(&rd_dst.link_[i].xpos, &link_[i].xpos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].xipos, &link_[i].xipos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].Rotm, &link_[i].Rotm, sizeof(Matrix3d));
        memcpy(&rd_dst.link_[i].v, &link_[i].v, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].w, &link_[i].w, sizeof(Vector3d));

        //xpos xipos rotm v w
    }

    memcpy(&rd_dst.A_, &A_, sizeof(MatrixVVf));
    memcpy(&rd_dst.A_inv_, &A_inv_, sizeof(MatrixVVf));
    memcpy(&rd_dst.Motor_inertia, &Motor_inertia, sizeof(MatrixVVf));
    memcpy(&rd_dst.Motor_inertia_inverse, &Motor_inertia_inverse, sizeof(MatrixVVf));
    memcpy(&rd_dst.q_, &q_, sizeof(VectorQd));
    memcpy(&rd_dst.q_dot_, &q_dot_, sizeof(VectorQd));
    memcpy(&rd_dst.q_virtual_, &q_virtual_, sizeof(VectorQVQd));
    memcpy(&rd_dst.q_dot_virtual_, &q_dot_virtual_, sizeof(VectorVQd));
    memcpy(&rd_dst.q_ddot_virtual_, &q_ddot_virtual_, sizeof(VectorVQd));
}

void StateManager::updateKinematics_local(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */
    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);

    link_p[Pelvis].pos_Update(model_l, q_virtual_f);
    link_p[Right_Foot].pos_Update(model_l, q_virtual_f);
    link_p[Left_Foot].pos_Update(model_l, q_virtual_f);
    link_p[Right_Hand].pos_Update(model_l, q_virtual_f);
    link_p[Left_Hand].pos_Update(model_l, q_virtual_f);

    Eigen::Vector3d zero;
    zero.setZero();

    link_p[Pelvis].Set_Jacobian(model_l, q_virtual_f, zero);
    link_p[Right_Foot].Set_Jacobian(model_l, q_virtual_f, zero);
    link_p[Left_Foot].Set_Jacobian(model_l, q_virtual_f, zero);
    link_p[Right_Hand].Set_Jacobian(model_l, q_virtual_f, zero);
    link_p[Left_Hand].Set_Jacobian(model_l, q_virtual_f, zero);

    link_p[Pelvis].vw_Update(q_dot_virtual_f);
    link_p[Right_Foot].vw_Update(q_dot_virtual_f);
    link_p[Left_Foot].vw_Update(q_dot_virtual_f);
    link_p[Right_Hand].vw_Update(q_dot_virtual_f);
    link_p[Left_Hand].vw_Update(q_dot_virtual_f);
}

void StateManager::updateKinematics(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

    A_temp_.setZero();

    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_l, q_virtual_f, A_temp_, false);

    A_ = A_temp_;
    A_inv_ = A_.inverse();
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_p[i].pos_Update(model_l, q_virtual_f);
    }
    Eigen::Vector3d zero;
    zero.setZero();
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_p[i].Set_Jacobian(model_l, q_virtual_f, zero);
    }

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {

        link_p[i].COM_Jac_Update(model_l, q_virtual_f);
    }
    //COM link information update ::
    double com_mass;
    RigidBodyDynamics::Math::Vector3d com_pos;
    RigidBodyDynamics::Math::Vector3d com_vel, com_accel, com_ang_momentum, com_ang_moment;

    RigidBodyDynamics::Utils::CalcCenterOfMass(model_l, q_virtual_f, q_dot_virtual_f, &q_ddot_virtual_f, com_mass, com_pos, &com_vel, &com_accel, &com_ang_momentum, &com_ang_moment, false);

    //com_.accel = com_accel;
    //com_.angular_momentum = com_ang_momentum;
    //com_.angular_moment = com_ang_moment;

    // double w_ = sqrt(9.81 / com_.pos(2));

    // com_.ZMP(0) = com_.pos(0) - com_.accel(0) / pow(w_, 2);
    // com_.ZMP(1) = com_.pos(1) - com_.accel(1) / pow(w_, 2);

    // com_.CP(0) = com_.pos(0) + com_.vel(0) / w_;
    // com_.CP(1) = com_.pos(1) + com_.vel(1) / w_;

    Eigen::Matrix3Vf jacobian_com;

    jacobian_com.setZero();

    for (int i = 0; i < LINK_NUMBER; i++)
    {
        jacobian_com += link_p[i].Jac_COM.topRows(3) * link_p[i].Mass;
    }

    link_p[COM_id].Mass = com_mass;
    link_p[COM_id].xpos = com_pos;
    link_p[COM_id].v = com_vel;

    link_p[COM_id].Jac.setZero(6, MODEL_DOF_VIRTUAL);

    //link_p[COM_id].Jac.block(0, 0, 2, MODEL_DOF + 6) = jacobian_com.block(0, 0, 2, MODEL_DOF + 6) / com_.mass;
    //link_p[COM_id].Jac.block(2, 0, 4, MODEL_DOF + 6) = link_p[Pelvis].Jac.block(2, 0, 4, MODEL_DOF + 6);

    link_p[COM_id].Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com.block(0, 0, 3, MODEL_DOF_VIRTUAL) / com_mass;
    link_p[COM_id].Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = link_p[Pelvis].Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);

    link_p[COM_id].Jac_COM.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com / com_mass;

    link_p[COM_id].xpos = com_pos;
    //link_p[COM_id].xpos(2) = link_p[Pelvis].xpos(2);
    link_p[COM_id].Rotm = link_p[Pelvis].Rotm;

    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        link_p[i].vw_Update(q_dot_virtual_f);
    }
}

void StateManager::stateEstimate()
{
    q_virtual_ = q_virtual_local_;
    q_dot_virtual_ = q_dot_virtual_local_;
    q_ddot_virtual_ = q_ddot_virtual_local_;
}