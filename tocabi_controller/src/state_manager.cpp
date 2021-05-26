#include "tocabi_controller/state_manager.h"

using namespace std;
using namespace TOCABI;

StateManager::StateManager(DataContainer &dc_global) : dc_(dc_global)
{
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

        total_mass_ = 0;

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_[i].Initialize(model_2, link_id_[i]);
            total_mass_ += link_[i].mass;
        }
        dc_.rd_.total_mass_ = total_mass_;

        memcpy(link_local_, link_, sizeof(LinkData) * LINK_NUMBER);
    }

    if (dc_.simMode)
    {
        mujoco_sim_command_pub_ = dc_.nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100);
        mujoco_sim_command_sub_ = dc_.nh.subscribe("/mujoco_ros_interface/sim_command_sim2con", 100, &StateManager::SimCommandCallback, this);
    }

    joint_state_pub_ = dc_.nh.advertise<sensor_msgs::JointState>("/tocabi/jointstates", 100);

    joint_state_msg_.name.resize(MODEL_DOF);
    joint_state_msg_.position.resize(MODEL_DOF);

    for (int i = 0; i < MODEL_DOF; i++)
        joint_state_msg_.name[i] = JOINT_NAME[i];

    task_command_sub_ = dc_.nh.subscribe("/tocabi/taskcommand", 100, &StateManager::TaskCommandCallback, this);
    task_command_que_sub_ = dc_.nh.subscribe("/tocabi/taskquecommand", 100, &StateManager::TaskQueCommandCallback, this);
    gui_command_sub_ = dc_.nh.subscribe("/tocabi/command", 100, &StateManager::GuiCommandCallback, this);
    gui_state_pub_ = dc_.nh.advertise<std_msgs::Int32MultiArray>("/tocabi/systemstate", 100);
    point_pub_ = dc_.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/point", 100);
    status_pub_ = dc_.nh.advertise<std_msgs::String>("/tocabi/guilog", 100);
    timer_pub_ = dc_.nh.advertise<std_msgs::Float32>("/tocabi/time", 100);

    elmo_status_pub_ = dc_.nh.advertise<std_msgs::Int8MultiArray>("/tocabi/ecatstates", 100);

    point_pub_msg_.polygon.points.resize(10);
    elmo_status_msg_.data.resize(MODEL_DOF * 3);
}

StateManager::~StateManager()
{
    cout << "StateManager Terminate" << endl;
}

void *StateManager::StateThread()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int rcv_tcnt = -1;

    //Checking Connect//

    //Check Coonnect Complete//
    rcv_tcnt = dc_.tc_shm_->statusCount;
    //cout << "first packet " << rcv_tcnt << endl;
    int cycle_count_ = rcv_tcnt;
    int stm_count_ = 0;

    int cnt = 0;
    int cnt2 = 0;
    int cnt3 = 0;
    auto time_start = std::chrono::steady_clock::now();
    while (!dc_.tc_shm_->shutdown)
    {
        ros::spinOnce();
        if (rcv_tcnt >= dc_.tc_shm_->statusCount)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        else
        {
            cycle_count_++;
            stm_count_++;

            if (rcv_tcnt + 1 != dc_.tc_shm_->statusCount)
            {
                std::cout << "missed packet : " << dc_.tc_shm_->statusCount - rcv_tcnt << std::endl;
            }
            rcv_tcnt = dc_.tc_shm_->statusCount;

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

            auto t1 = chrono::steady_clock::now();
            GetJointData(); //0.246 us //w/o march native 0.226
            auto dur_start_ = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_start).count();
            control_time_ = rcv_tcnt / 2000.0;
            //local kinematics update : 33.7 us // w/o march native 20 us
            UpdateKinematics_local(model_, link_local_, q_virtual_local_, q_dot_virtual_local_, q_ddot_virtual_local_);

            auto d1 = chrono::duration_cast<chrono::nanoseconds>(chrono::steady_clock::now() - t1).count();
            auto t2 = chrono::steady_clock::now();
            StateEstimate();

            //global kinematics update : 127 us //w/o march native 125 us
            UpdateKinematics(model_2, link_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);

            StoreState(dc_.rd_); //6.2 us //w/o march native 8us

            auto d2 = chrono::duration_cast<chrono::nanoseconds>(chrono::steady_clock::now() - t2).count();
            MeasureTime(stm_count_, d1, d2);

            dc_.rd_.us_from_start_ = dur_start_;

            if (stm_count_ % 33 == 0)
            {
                cnt3++;
                PublishData();
            }
            //dc_.tc_shm_->t_cnt2 = stm_count_;
            dc_.tc_shm_->t_cnt2 = cnt3;

            //printf("%d\n", rcv_tcnt);
            //printf("\x1b[A\x1b[A\33[2K\r");
            // if (rcv_tcnt % 33 == 0)
            // {
            //     printf("\33[2K\r");
            //     printf("%8d %8d avg : %7.3f max : %4d min : %4d, cnt : %4d, cnt2 : %4d, cnt3 : %4d ", rcv_tcnt, cycle_count_, avg, max, min, cnt, cnt2, cnt3);
            //     fflush(stdout);
            // }
        }
    }
    cout << "StateManager Thread END" << endl;
}

void StateManager::PublishData()
{
    timer_msg_.data = control_time_;

    timer_pub_.publish(timer_msg_);

    geometry_msgs::TransformStamped ts;

    ts.header.stamp = ros::Time::now();

    ts.header.frame_id = "world";
    ts.child_frame_id = "Pelvis_Link";

    ts.transform.rotation.x = q_virtual_local_[3];
    ts.transform.rotation.y = q_virtual_local_[4];
    ts.transform.rotation.z = q_virtual_local_[5];
    ts.transform.rotation.w = q_virtual_local_[MODEL_DOF_VIRTUAL];

    ts.transform.translation.x = q_virtual_(0);
    ts.transform.translation.y = q_virtual_(1);
    ts.transform.translation.z = q_virtual_(2);

    br.sendTransform(ts);

    joint_state_msg_.header.stamp = ros::Time::now();
    for (int i = 0; i < MODEL_DOF; i++)
        joint_state_msg_.position[i] = q_virtual_local_[i + 6];

    joint_state_pub_.publish(joint_state_msg_);

    point_pub_msg_.polygon.points[0].x = link_[COM_id].xpos(0);
    point_pub_msg_.polygon.points[0].y = link_[COM_id].xpos(1);
    point_pub_msg_.polygon.points[0].z = link_[COM_id].xpos(2);

    point_pub_msg_.polygon.points[1].x = link_[Right_Foot].xpos(0);
    point_pub_msg_.polygon.points[1].y = link_[Right_Foot].xpos(1);
    point_pub_msg_.polygon.points[1].z = link_[Right_Foot].xpos(2);

    point_pub_msg_.polygon.points[2].x = link_[Left_Foot].xpos(0);
    point_pub_msg_.polygon.points[2].y = link_[Left_Foot].xpos(1);
    point_pub_msg_.polygon.points[2].z = link_[Left_Foot].xpos(2);

    point_pub_msg_.polygon.points[3].x = link_[Pelvis].xpos(0);
    point_pub_msg_.polygon.points[3].y = link_[Pelvis].xpos(1);
    point_pub_msg_.polygon.points[3].z = link_[Pelvis].xpos(2);

    double tr_, tp_, ty_;

    DyrosMath::rot2Euler_tf2(link_[Pelvis].rotm, tr_, tp_, ty_);

    point_pub_msg_.polygon.points[4].x = tr_; //rpy
    point_pub_msg_.polygon.points[4].y = tp_; //rpy
    point_pub_msg_.polygon.points[4].z = ty_; //rpy

    point_pub_msg_.polygon.points[5].x = link_[Right_Hand].xpos(0);
    point_pub_msg_.polygon.points[5].y = link_[Right_Hand].xpos(1);
    point_pub_msg_.polygon.points[5].z = link_[Right_Hand].xpos(2);

    point_pub_msg_.polygon.points[6].x = link_[Left_Hand].xpos(0);
    point_pub_msg_.polygon.points[6].y = link_[Left_Hand].xpos(1);
    point_pub_msg_.polygon.points[6].z = link_[Left_Hand].xpos(2);

    point_pub_msg_.polygon.points[7].x = 0.0; //zmp
    point_pub_msg_.polygon.points[7].y = 0.0;
    point_pub_msg_.polygon.points[7].z = 0.0;

    DyrosMath::rot2Euler_tf2(link_[Left_Foot].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[8].x = tr_;
    point_pub_msg_.polygon.points[8].y = tp_;
    point_pub_msg_.polygon.points[8].z = ty_;

    DyrosMath::rot2Euler_tf2(link_[Right_Foot].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[9].x = tr_;
    point_pub_msg_.polygon.points[9].y = tp_;
    point_pub_msg_.polygon.points[9].z = ty_;

    point_pub_.publish(point_pub_msg_);

    //

    bool query_elmo_pub_ = false;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        elmo_status_msg_.data[i] = state_elmo_[i];
        elmo_status_msg_.data[i + 33] = state_zp_[i];
        elmo_status_msg_.data[i + 66] = state_safety_[i];

        if (state_elmo_[i] != state_elmo_before_[i])
        {
            query_elmo_pub_ = true;
        }

        if (state_safety_[i] != state_safety_before_[i])
        {
            query_elmo_pub_ = true;
        }

        if (state_zp_[i] != state_zp_before_[i])
        {
            query_elmo_pub_ = true;
        }

        state_elmo_before_[i] = state_elmo_[i];
        state_safety_before_[i] = state_safety_[i];
        state_zp_before_[i] = state_zp_[i];
    }

    if (query_elmo_pub_)
    {
        elmo_status_pub_.publish(elmo_status_msg_);
    }

    //memcpy(joint_state_before_, joint_state_, sizeof(int) * MODEL_DOF);
}

void StateManager::GetJointData()
{
    while (dc_.tc_shm_->statusWriting)
    {
        if (dc_.tc_shm_->shutdown)
            break;
    }

    memcpy(q_a_, dc_.tc_shm_->pos, sizeof(float) * MODEL_DOF);
    memcpy(q_dot_a_, dc_.tc_shm_->vel, sizeof(float) * MODEL_DOF);

    q_ = Map<VectorQf>(q_a_, MODEL_DOF).cast<double>();
    q_dot_ = Map<VectorQf>(q_dot_a_, MODEL_DOF).cast<double>();

    q_virtual_local_.segment(6, MODEL_DOF) = q_;
    q_dot_virtual_local_.segment(6, MODEL_DOF) = q_dot_;

    q_virtual_local_(0) = dc_.tc_shm_->pos_virtual[0];
    q_virtual_local_(1) = dc_.tc_shm_->pos_virtual[1];
    q_virtual_local_(2) = dc_.tc_shm_->pos_virtual[2];

    q_virtual_local_(3) = dc_.tc_shm_->pos_virtual[3];
    q_virtual_local_(4) = dc_.tc_shm_->pos_virtual[4];
    q_virtual_local_(5) = dc_.tc_shm_->pos_virtual[5];
    q_virtual_local_(MODEL_DOF_VIRTUAL) = dc_.tc_shm_->pos_virtual[6];

    //memcpy(joint_state_, dc_.tc_shm_->status, sizeof(int) * MODEL_DOF);

    memcpy(state_elmo_, dc_.tc_shm_->ecat_status, sizeof(int8_t) * MODEL_DOF);
    memcpy(state_safety_, dc_.tc_shm_->safety_status, sizeof(int8_t) * MODEL_DOF);
    memcpy(state_zp_, dc_.tc_shm_->zp_status, sizeof(int8_t) * MODEL_DOF);

    //dc_.tc_shm_->pos
}

void StateManager::GetSensorData()
{
}

void StateManager::ConnectSim()
{
}

void StateManager::GetSimData()
{
    ros::spinOnce();
}

void StateManager::MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2)
{
    dc_.tc_shm_->t_cnt = currentCount;

    lat = nanoseconds1;
    total1 += lat;
    lavg = total1 / currentCount;
    if (lmax < lat)
    {
        lmax = lat;
    }
    if (lmin > lat)
    {
        lmin = lat;
    }
    // int sdev = (sat - savg)
    total_dev1 += sqrt(((lat - lavg) * (lat - lavg)));
    ldev = total_dev1 / currentCount;

    dc_.tc_shm_->lat_avg = lavg;
    dc_.tc_shm_->lat_max = lmax;
    dc_.tc_shm_->lat_min = lmin;
    dc_.tc_shm_->lat_dev = ldev;

    sat = nanoseconds2;
    total2 += sat;
    savg = total2 / currentCount;
    if (smax < sat)
    {
        smax = sat;
    }
    if (smin > sat)
    {
        smin = sat;
    }
    // int sdev = (sat - savg)
    total_dev2 += sqrt(((sat - savg) * (sat - savg)));
    sdev = total_dev2 / currentCount;

    dc_.tc_shm_->send_avg = savg;
    dc_.tc_shm_->send_max = smax;
    dc_.tc_shm_->send_min = smin;
    dc_.tc_shm_->send_dev = sdev;
}

void StateManager::StoreState(RobotData &rd_dst)
{

    memcpy(&rd_dst.model_, &model_2, sizeof(RigidBodyDynamics::Model));

    for (int i = 0; i < (LINK_NUMBER + 1); i++)
    {
        memcpy(&rd_dst.link_[i].jac, &link_[i].jac, sizeof(Matrix6Vf));
        memcpy(&rd_dst.link_[i].jac_com, &link_[i].jac_com, sizeof(Matrix6Vf));

        memcpy(&rd_dst.link_[i].xpos, &link_[i].xpos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].xipos, &link_[i].xipos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].rotm, &link_[i].rotm, sizeof(Matrix3d));
        memcpy(&rd_dst.link_[i].v, &link_[i].v, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].w, &link_[i].w, sizeof(Vector3d));

        //xpos xipos rotm v w
    }

    memcpy(&rd_dst.A_, &A_, sizeof(MatrixVVd));
    memcpy(&rd_dst.A_inv_, &A_inv_, sizeof(MatrixVVd));
    memcpy(&rd_dst.Motor_inertia, &Motor_inertia, sizeof(MatrixVVd));
    memcpy(&rd_dst.Motor_inertia_inverse, &Motor_inertia_inverse, sizeof(MatrixVVd));
    memcpy(&rd_dst.q_, &q_, sizeof(VectorQd));
    memcpy(&rd_dst.q_dot_, &q_dot_, sizeof(VectorQd));
    memcpy(&rd_dst.q_virtual_, &q_virtual_, sizeof(VectorQVQd));
    memcpy(&rd_dst.q_dot_virtual_, &q_dot_virtual_, sizeof(VectorVQd));
    memcpy(&rd_dst.q_ddot_virtual_, &q_ddot_virtual_, sizeof(VectorVQd));

    if (!rd_dst.firstCalc)
    {

        memcpy(&rd_dst.link_, link_, (LINK_NUMBER + 1) * sizeof(LinkData));

        rd_dst.firstCalc = true;
    }

    if (rd_.task_signal_)
    {
        rd_.task_signal_ = false;
        memcpy(&rd_dst.tc_, &rd_.tc_, sizeof(tocabi_msgs::TaskCommand));
        rd_dst.task_signal_ = true;
    }
    if (rd_.task_que_signal_)
    {
        rd_.task_que_signal_ = false;
        memcpy(&rd_dst.tc_q_, &rd_.tc_q_, sizeof(tocabi_msgs::TaskCommandQue));
        rd_dst.task_que_signal_ = true;
    }

    rd_dst.control_time_ = control_time_;
}

void StateManager::UpdateKinematics_local(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

    //local kinematics update sequence

    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);

    link_p[Pelvis].UpdatePosition(model_l, q_virtual_f);
    link_p[Right_Foot].UpdatePosition(model_l, q_virtual_f);
    link_p[Left_Foot].UpdatePosition(model_l, q_virtual_f);
    link_p[Right_Hand].UpdatePosition(model_l, q_virtual_f);
    link_p[Left_Hand].UpdatePosition(model_l, q_virtual_f);

    link_p[Pelvis].UpdateJacobian(model_l, q_virtual_f);
    link_p[Right_Foot].UpdateJacobian(model_l, q_virtual_f);
    link_p[Left_Foot].UpdateJacobian(model_l, q_virtual_f);
    link_p[Right_Hand].UpdateJacobian(model_l, q_virtual_f);
    link_p[Left_Hand].UpdateJacobian(model_l, q_virtual_f);

    link_p[Pelvis].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Right_Foot].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Left_Foot].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Right_Hand].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Left_Hand].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
}

void StateManager::UpdateKinematics(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

    // Update Kinematics : Total 127 us

    // sector 1 Start : rbdl update
    A_temp_.setZero();
    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_l, q_virtual_f, A_temp_, false);
    A_ = A_temp_;
    // sector 1 End : 30 us

    // sector 2 start : link pos/vel/jac update
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_p[i].UpdatePosition(model_l, q_virtual_f);
        link_p[i].UpdateJacobian(model_l, q_virtual_f, q_dot_virtual_f);
    }
    // sector 2 end : 57 us

    // sector 3 start : mass matrix inverse
    //A_inv_ = A_.inverse();
    A_inv_ = A_.llt().solve(Eigen::MatrixVVd::Identity());
    // sector 3 end : 39 us

    // sector 4 start : com calculation
    Eigen::Matrix3Vf jacobian_com;
    jacobian_com.setZero();
    for (int i = 0; i < LINK_NUMBER; i++)
    {
        jacobian_com += link_p[i].jac_com.topRows(3) * link_p[i].mass / total_mass_;
    }
    link_p[COM_id].mass = total_mass_;
    link_p[COM_id].xpos.setZero();
    for (int i = 0; i < LINK_NUMBER; i++)
        link_p[COM_id].xpos += link_p[i].xipos * link_p[i].mass / total_mass_;
    link_p[COM_id].v = jacobian_com.cast<double>() * q_dot_virtual_f;
    link_p[COM_id].rotm = link_p[Pelvis].rotm;
    link_p[COM_id].jac.setZero(6, MODEL_DOF_VIRTUAL);
    //link_p[COM_id].jac.block(0, 0, 2, MODEL_DOF + 6) = jacobian_com.block(0, 0, 2, MODEL_DOF + 6) / com_.mass;
    //link_p[COM_id].jac.block(2, 0, 4, MODEL_DOF + 6) = link_p[Pelvis].jac.block(2, 0, 4, MODEL_DOF + 6);
    link_p[COM_id].jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com.block(0, 0, 3, MODEL_DOF_VIRTUAL);
    link_p[COM_id].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = link_p[Pelvis].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);

    link_p[COM_id].jac_com.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com;
    //link_p[COM_id].xpos(2) = link_p[Pelvis].xpos(2);
    // sector 4 end : 3 us
}

void StateManager::StateEstimate()
{
    q_virtual_ = q_virtual_local_;
    q_dot_virtual_ = q_dot_virtual_local_;
    q_ddot_virtual_ = q_ddot_virtual_local_;
}

void StateManager::CalcNonlinear()
{
    //RigidBodyDynamics::NonlinearEffects(model_,)
}

void StateManager::SimCommandCallback(const std_msgs::StringConstPtr &msg)
{

    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        //parameterInitialize();
        sim_time_before_ = 0.0;

        mujoco_ready = true;

        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        mujoco_init_receive = true;
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_.publish(rst_msg_);
        sim_time_ = 0.0;
        control_time_ = 0.0;
        //dc.semode_init = true;
        mujoco_reset = true;
    }

    if (buf == "terminate")
    {
        dc_.tc_shm_->shutdown = true;
    }
}

void StateManager::TaskCommandCallback(const tocabi_msgs::TaskCommandConstPtr &msg)
{
    rd_.tc_ = *msg;
    rd_.task_signal_ = true;
    rd_.tc_init = true;
    rd_.tc_time_ = control_time_;

    std::cout << "tc received" << std::endl;
}

void StateManager::TaskQueCommandCallback(const tocabi_msgs::TaskCommandQueConstPtr &msg)
{
    rd_.tc_q_ = *msg;
    rd_.task_que_signal_ = true;

    std::cout << "tc_que received" << std::endl;
}

void StateManager::GuiCommandCallback(const std_msgs::StringConstPtr &msg)
{
    std::cout << "Received msg from GUI : " << msg->data << std::endl;
    //Receiving Command from GUI!

    if (msg->data == "torqueon")
    {
        dc_.torqueOnSwitch = true;
    }
    else if (msg->data == "torqueoff")
    {
        dc_.torqueOffSwitch = true;
    }
    else if (msg->data == "emergencyoff")
    {
        dc_.emergencySwitch = true;
    }
    else if (msg->data == "inityaw")
    {
        dc_.inityawSwitch = true;
    }
    else if (msg->data == "ftcalib")
    {
        dc_.ftcalibSwtich = true;
    }
    else if (msg->data == "imureset")
    {
        dc_.imuResetSwtich = true;
    }
    else if (msg->data == "stateestimation")
    {
        dc_.stateEstimateSwitch = true;
    }
    else if (msg->data == "safetyreset")
    {
        dc_.safetyResetSwitch = true;
    }
    else if (msg->data == "ecatinit")
    {
        dc_.tc_shm_->upper_init_signal = true;  
    }
    else if (msg->data == "ecatinitlower")
    {
        dc_.tc_shm_->low_init_signal = true;
    }
    else if(msg->data == "ecatinitwaist")
    {
        dc_.tc_shm_->waist_init_signal = true;
    }

    //Controlling GUI
}

void StateManager::StatusPub(const char *str, ...)
{
    va_list lst;
    va_start(lst, str);

    char text_[256];

    vsnprintf(text_, 255, str, lst);

    std::string str_(text_);

    std_msgs::String str_msg_;
    str_msg_.data = str_;

    status_pub_.publish(str_msg_);

    //std::cout<<str_;

    va_end(lst);
}