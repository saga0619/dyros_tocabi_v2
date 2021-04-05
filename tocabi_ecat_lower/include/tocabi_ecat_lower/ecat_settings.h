#include <iostream>

#define CNT_TO_RAD_46 (3.141592 * 2 / (8192 * 100)) //819200
#define CNT_TO_RAD_80 (3.141592 * 2 / (8000 * 100)) //819200

#define DEG2RAD (3.141592f/180.0f)

#define EXT_CNT_TO_RAD_46 (3.141592 * 2 / 8192) //819200
#define EXT_CNT_TO_RAD_80 (3.141592 * 2 / 8192) //819200

#define RAD_TO_CNT_46 (1 / (CNT_TO_RAD_46))
#define RAD_TO_CNT_80 (1 / (CNT_TO_RAD_80))

#define EXT_RAD_TO_CNT_46 (1 / (EXT_CNT_TO_RAD_46))
#define EXT_RAD_TO_CNT_80 (1 / (EXT_CNT_TO_RAD_80))

#define EC_TIMEOUTMON 500

#define ELMO_DOF 33

#define ELMO_DOF_LOWER 15

#define ELMO_DOF_UPPER 18

#define LEG_DOF 12

#define UPPERBODY_DOF 21

#define CYCLETIME 500

#define PERIOD_NS 500000
#define SEC_IN_NSEC 1000000000


std::string ifname_str = "enp4s0";


enum
{
    Head_Joint,
    Neck_Joint,
    R_Wrist1_Joint,
    R_Wrist2_Joint,
    L_Wrist2_Joint,
    L_Wrist1_Joint,
    L_Shoulder3_Joint,
    L_Armlink_Joint,
    R_Armlink_Joint,
    R_Shoulder3_Joint,
    R_Elbow_Joint,
    R_Forearm_Joint,
    L_Forearm_Joint,
    L_Elbow_Joint,
    L_Shoulder1_Joint,
    L_Shoulder2_Joint,
    R_Shoulder2_Joint,
    R_Shoulder1_Joint,
    Upperbody_Joint,
    Waist2_Joint,
    R_HipYaw_Joint,
    R_HipRoll_Joint,
    R_HipPitch_Joint,
    R_Knee_Joint,
    R_AnklePitch_Joint,
    R_AnkleRoll_Joint,
    Waist1_Joint,
    L_HipYaw_Joint,
    L_HipRoll_Joint,
    L_HipPitch_Joint,
    L_Knee_Joint,
    L_AnklePitch_Joint,
    L_AnkleRoll_Joint
};



const int starting_joint = Upperbody_Joint;

const std::string ELMO_NAME[ELMO_DOF] = {
    "Head_Joint", "Neck_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint", "L_Wrist2_Joint", "L_Wrist1_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
    "R_Armlink_Joint", "R_Shoulder3_Joint", "R_Elbow_Joint", "R_Forearm_Joint", "L_Forearm_Joint", "L_Elbow_Joint", "L_Shoulder1_Joint", "L_Shoulder2_Joint",
    "R_Shoulder2_Joint", "R_Shoulder1_Joint", "Upperbody_Joint", "Waist2_Joint", "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint",
    "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint", "Waist1_Joint", "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint",
    "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint"};

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
        5370, 3942, 5148 -6 , 3234, 7499-5, 4288,
        0,
        3799, 2522, 735, 8132+37, 2127+0, 7155+0};

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
    {1.0, 1.0,
     1.57, 2.094, 2.094, 1.57, 1.92, 3.15, 3.15, 1.92,
     2.8, 3.15, 3.15, -0.1, 1.54, 3.15, 3.15, 2.09,
     3, 3,
     0.6, 3, 3, 3, 1, 0.664,
     3,
     0.6, 3, 3, 3, 1, 0.664};

const double joint_lower_limit[ELMO_DOF] =
    {-1.0, -1.0,
     -1.57, -2.094, -2.094, -1.57, -1.92, -3.15, -3.15, -1.92,
     0.1, -3.15, -3.15, -2.8, -2.09, -3.15, -3.15, -1.54,
     -3, -3,
     -0.6, -3, -1.5, -0.5, -1.0, -0.664,
     -3,
     -0.6, -3, -1.5, -0.5, -1.0, -0.664};

const double elmo_axis_direction[ELMO_DOF] =
    {1, -1, 1, -1, 1, 1,
     1, 1, 1, -1, -1, 1,
     1, -1, 1, 1, 1, 1,
     1, 1, -1, -1, -1, -1,
     1, 1, 1, 1, -1, 1,
     1, -1, 1};

const double elmo_ext_axis_direction[ELMO_DOF] =
    {1, -1, 1, 1, 1, 1,
     1, 1, 1, -1, -1, 1,
     1, -1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1,
     -1, -1, 1, 1, 1, -1,
     -1, 1, -1};

const double upper_homming_minimum_required_length[ELMO_DOF] =
    {

};