#include "tocabi_ecat/ecat_settings.h"

const char ifname_lower[] = "enp1s0f0";
char ifname_lower2[] = "enp1s0f1";
const char ifname_upper[] = "enp1s0f2";
char ifname_upper2[] = "enp1s0f3";

const int starting_point = ELMO_DOF_UPPER;

const char ELMO_NAME[ELMO_DOF][20] = {
    "Head_Joint", "Neck_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint", "L_Wrist2_Joint", "L_Wrist1_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
    "R_Armlink_Joint", "R_Shoulder3_Joint", "R_Elbow_Joint", "R_Forearm_Joint", "L_Forearm_Joint", "L_Elbow_Joint", "L_Shoulder1_Joint", "L_Shoulder2_Joint",
    "R_Shoulder2_Joint", "R_Shoulder1_Joint", "Upperbody_Joint", "Waist2_Joint", "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint",
    "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint", "Waist1_Joint", "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint",
    "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint"};

//pos[i] = pos_elmo[JointMap[i]]
const int JointMap[ELMO_DOF] = {
    ELMO::L_HipYaw_Joint,
    ELMO::L_HipRoll_Joint,
    ELMO::L_HipPitch_Joint,
    ELMO::L_Knee_Joint,
    ELMO::L_AnklePitch_Joint,
    ELMO::L_AnkleRoll_Joint,
    ELMO::R_HipYaw_Joint,
    ELMO::R_HipRoll_Joint,
    ELMO::R_HipPitch_Joint,
    ELMO::R_Knee_Joint,
    ELMO::R_AnklePitch_Joint,
    ELMO::R_AnkleRoll_Joint,
    ELMO::Waist1_Joint,
    ELMO::Waist2_Joint,
    ELMO::Upperbody_Joint,
    ELMO::L_Shoulder1_Joint,
    ELMO::L_Shoulder2_Joint,
    ELMO::L_Shoulder3_Joint,
    ELMO::L_Armlink_Joint,
    ELMO::L_Elbow_Joint,
    ELMO::L_Forearm_Joint,
    ELMO::L_Wrist1_Joint,
    ELMO::L_Wrist2_Joint,
    ELMO::Neck_Joint,
    ELMO::Head_Joint,
    ELMO::R_Shoulder1_Joint,
    ELMO::R_Shoulder2_Joint,
    ELMO::R_Shoulder3_Joint,
    ELMO::R_Armlink_Joint,
    ELMO::R_Elbow_Joint,
    ELMO::R_Forearm_Joint,
    ELMO::R_Wrist1_Joint,
    ELMO::R_Wrist2_Joint};

//pos_elmo[i] = pos[JointMap2[i]]
const int JointMap2[ELMO_DOF] = {
    MODEL::Head_Joint,
    MODEL::Neck_Joint,
    MODEL::R_Wrist1_Joint,
    MODEL::R_Wrist2_Joint,
    MODEL::L_Wrist2_Joint,
    MODEL::L_Wrist1_Joint,
    MODEL::L_Shoulder3_Joint,
    MODEL::L_Armlink_Joint,
    MODEL::R_Armlink_Joint,
    MODEL::R_Shoulder3_Joint,
    MODEL::R_Elbow_Joint,
    MODEL::R_Forearm_Joint,
    MODEL::L_Forearm_Joint,
    MODEL::L_Elbow_Joint,
    MODEL::L_Shoulder1_Joint,
    MODEL::L_Shoulder2_Joint,
    MODEL::R_Shoulder2_Joint,
    MODEL::R_Shoulder1_Joint,
    MODEL::Upperbody_Joint,
    MODEL::Waist2_Joint,
    MODEL::R_HipYaw_Joint,
    MODEL::R_HipRoll_Joint,
    MODEL::R_HipPitch_Joint,
    MODEL::R_Knee_Joint,
    MODEL::R_AnklePitch_Joint,
    MODEL::R_AnkleRoll_Joint,
    MODEL::Waist1_Joint,
    MODEL::L_HipYaw_Joint,
    MODEL::L_HipRoll_Joint,
    MODEL::L_HipPitch_Joint,
    MODEL::L_Knee_Joint,
    MODEL::L_AnklePitch_Joint,
    MODEL::L_AnkleRoll_Joint};

const double CNT2RAD[ELMO_DOF] =
    {
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_80, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_80, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46};

const double EXTCNT2RAD[ELMO_DOF] =
    {
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46};

const double RAD2CNT[ELMO_DOF] =
    {
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_80, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_80, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46};

const double EXTRAD2CNT[ELMO_DOF] =
    {
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46};

const double NM2CNT[ELMO_DOF] =
    {         //Elmo 순서
        95.0, //head
        95.0,
        95.0, //wrist
        95.0,
        95.0,
        95.0,
        15.5, //shoulder3
        15.5, //arm
        15.5, //arm
        15.5, //shoulder3
        42.0, //Elbow
        42.0, //Forearm
        42.0, //Forearm
        42.0, //Elbow
        15.5, //shoulder1
        15.5, //shoulder2
        15.5, //shoulder2
        15.5, //shoulder1
        3.3,  //Waist
        3.3,
        3.0, //rightLeg
        4.3,
        3.8,
        3.46,
        4.5,
        6.0,
        3.3, //upperbody
        3.0, //leftLeg
        4.3,
        3.8,
        3.46,
        4.5,
        6.0};

const int q_ext_mod_elmo_[ELMO_DOF] =
    {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0,
        5370, 3942, 5148 - 6, 3234, 7499 - 5, 4288,
        0,
        3799, 2522, 735, 8132 + 37, 2127 + 0, 7155 + 0};

//right -> left
//right hippitch front -> modval +
//left knee pitch front -> modval -
//right ankle pitch up -> modval

const double joint_velocity_limit[ELMO_DOF] =
    {20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
     20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
     10.0, 10.0, 2.0,
     5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
     5.0, 5.0,
     5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

const double joint_upper_limit[ELMO_DOF] =
    {1.0, 1.6,
     1.57, 2.094, 2.094, 1.57, 1.92, 3.15, 3.15, 1.92,
     2.8, 3.15, 3.15, 0.5, 1.54, 3.15, 3.15, 2.09,
     3, 3,
     0.6, 3, 3, 3, 1, 0.664,
     3,
     0.6, 3, 3, 3, 1, 0.664};

const double joint_lower_limit[ELMO_DOF] =
    {-1.0, -1.6,
     -1.57, -2.094, -2.094, -1.57, -1.92, -3.15, -3.15, -1.92,
     -0.5, -3.15, -3.15, -2.8, -2.09, -3.15, -3.15, -1.54,
     -3, -3,
     -0.6, -3, -1.5, -0.5, -1.4, -0.664,
     -3,
     -0.6, -3, -1.5, -0.5, -1.4, -0.664};

const double elmo_axis_direction[ELMO_DOF] =
    {1, -1, 1, 1, -1, 1,
     1, 1, 1, 1, -1, 1,
     1, -1, 1, 1, 1, 1,
     1, 1, -1, -1, -1, -1,
     1, 1, 1, 1, -1, 1,
     1, -1, 1};

const double elmo_ext_axis_direction[ELMO_DOF] =
    {1, -1, 1, -1, -1, 1,
     1, 1, 1, -1, -1, 1,
     1, -1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1,
     -1, -1, 1, 1, 1, -1,
     -1, 1, -1};

const double upper_homming_minimum_required_length[ELMO_DOF] =
    {

};

const double pos_p_gain[ELMO_DOF] =
    {2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
     2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
     6000.0, 10000.0, 10000.0,
     400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 10.0, 10.0,
     1000.0, 1000.0,
     400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 10.0, 10.0

};
const double pos_d_gain[ELMO_DOF] =
    {15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
     15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
     200.0, 100.0, 100.0,
     10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 1.0, 1.0,
     10.0, 10.0,
     10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 1.0, 1.0

};
