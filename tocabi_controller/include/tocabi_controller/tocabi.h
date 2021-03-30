#ifndef TOCABI_H
#define TOCABI_H

#include <iostream>

#define MODEL_DOF 33

namespace TOCABI
{
    const std::string JOINT_NAME[MODEL_DOF] = {
        "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint",
        "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint",
        "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint",
        "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint",
        "Waist1_Joint", "Waist2_Joint", "Upperbody_Joint",
        "L_Shoulder1_Joint", "L_Shoulder2_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
        "L_Elbow_Joint", "L_Forearm_Joint", "L_Wrist1_Joint", "L_Wrist2_Joint",
        "Neck_Joint", "Head_Joint",
        "R_Shoulder1_Joint", "R_Shoulder2_Joint", "R_Shoulder3_Joint", "R_Armlink_Joint",
        "R_Elbow_Joint", "R_Forearm_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint"};

    const std::string ELMO_NAME[MODEL_DOF] = {
        "Head_Joint", "Neck_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint", "L_Wrist2_Joint", "L_Wrist1_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
        "R_Armlink_Joint", "R_Shoulder3_Joint", "R_Elbow_Joint", "R_Forearm_Joint", "L_Forearm_Joint", "L_Elbow_Joint", "L_Shoulder1_Joint", "L_Shoulder2_Joint",
        "R_Shoulder2_Joint", "R_Shoulder1_Joint", "Upperbody_Joint", "Waist2_Joint", "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint",
        "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint", "Waist1_Joint", "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint",
        "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint"};

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

    //Usage : ELMO[JointMap[i]] = MODEL[i]

    const int JointMap[MODEL_DOF] = {
        L_HipYaw_Joint,
        L_HipRoll_Joint,
        L_HipPitch_Joint,
        L_Knee_Joint,
        L_AnklePitch_Joint,
        L_AnkleRoll_Joint,
        R_HipYaw_Joint,
        R_HipRoll_Joint,
        R_HipPitch_Joint,
        R_Knee_Joint,
        R_AnklePitch_Joint,
        R_AnkleRoll_Joint,
        Waist1_Joint,
        Waist2_Joint,
        Upperbody_Joint,
        L_Shoulder1_Joint,
        L_Shoulder2_Joint,
        L_Shoulder3_Joint,
        L_Armlink_Joint,
        L_Elbow_Joint,
        L_Forearm_Joint,
        L_Wrist1_Joint,
        L_Wrist2_Joint,
        Neck_Joint,
        Head_Joint,
        R_Shoulder1_Joint,
        R_Shoulder2_Joint,
        R_Shoulder3_Joint,
        R_Armlink_Joint,
        R_Elbow_Joint,
        R_Forearm_Joint,
        R_Wrist1_Joint,
        R_Wrist2_Joint};

    const std::string ACTUATOR_NAME[MODEL_DOF] = {"L_HipYaw_Motor", "L_HipRoll_Motor", "L_HipPitch_Motor", "L_Knee_Motor", "L_AnklePitch_Motor", "L_AnkleRoll_Motor", "R_HipYaw_Motor", "R_HipRoll_Motor", "R_HipPitch_Motor", "R_Knee_Motor", "R_AnklePitch_Motor", "R_AnkleRoll_Motor", "Waist1_Motor", "Waist2_Motor", "Upperbody_Motor", "L_Shoulder1_Motor", "L_Shoulder2_Motor", "L_Shoulder3_Motor", "L_Armlink_Motor", "L_Elbow_Motor", "L_Forearm_Motor", "L_Wrist1_Motor", "L_Wrist2_Motor", "Neck_Motor", "Head_Motor", "R_Shoulder1_Motor", "R_Shoulder2_Motor", "R_Shoulder3_Motor", "R_Armlink_Motor", "R_Elbow_Motor", "R_Forearm_Motor", "R_Wrist1_Motor", "R_Wrist2_Motor"};

    static constexpr const char *LINK_NAME[MODEL_DOF+1] = {
        "Pelvis_Link", "Waist1_Link", "Waist2_Link", "Upperbody_Link",
        "L_HipRoll_Link", "L_HipCenter_Link", "L_Thigh_Link", "L_Knee_Link", "L_AnkleCenter_Link", "L_AnkleRoll_Link",
        "R_HipRoll_Link", "R_HipCenter_Link", "R_Thigh_Link", "R_Knee_Link", "R_AnkleCenter_Link", "R_AnkleRoll_Link",
        "L_Shoulder1_Link", "L_Shoulder2_Link", "L_Shoulder3_Link", "L_Armlink_Link", "L_Elbow_Link", "L_Forearm_Link", "L_Wrist1_Link", "L_Wrist2_Link",
        "R_Shoulder1_Link", "R_Shoulder2_Link", "R_Shoulder3_Link", "R_Armlink_Link", "R_Elbow_Link", "R_Forearm_Link", "R_Wrist1_Link", "R_Wrist2_Link",
        "Neck_Link", "Head_Link"};

    const std::string POSITIONACTUATOR_NAME[MODEL_DOF] = {
        "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint",
        "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint",
        "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint",
        "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint",
        "Waist1_Joint", "Waist2_Joint", "Upperbody_Joint",
        "L_Shoulder1_Joint", "L_Shoulder2_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
        "L_Elbow_Joint", "L_Forearm_Joint", "L_Wrist1_Joint", "L_Wrist2_Joint",
        "Neck_Joint", "Head_Joint",
        "R_Shoulder1_Joint", "R_Shoulder2_Joint", "R_Shoulder3_Joint", "R_Armlink_Joint",
        "R_Elbow_Joint", "R_Forearm_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint"};

} // namespace TOCABI

const int Pelvis = 0;
const int Upper_Body = 3;
const int Left_Foot = 9;
const int Right_Foot = 15;
const int Left_Hand = 23;
const int Right_Hand = 31;
const int Head = 33;
const int COM_id = 34;

const int LEFT = 0;
const int RIGHT = 1;

//static atomic
const std::string cred("\033[0;31m");
const std::string creset("\033[0m");
const std::string cblue("\033[0;34m");
const std::string cgreen("\033[0;32m");
const std::string cyellow("\033[0;33m");

#endif