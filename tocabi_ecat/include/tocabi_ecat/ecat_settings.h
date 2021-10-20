#define CNT_TO_RAD_46 (3.141592 * 2 / (8192 * 100)) //819200
#define CNT_TO_RAD_80 (3.141592 * 2 / (8000 * 100)) //819200

#define DEG2RAD (3.141592f / 180.0f)

#define EXT_CNT_TO_RAD_46 (3.141592 * 2 / 8192) //819200
#define EXT_CNT_TO_RAD_80 (3.141592 * 2 / 8192) //819200

#define RAD_TO_CNT_46 (1 / (CNT_TO_RAD_46))
#define RAD_TO_CNT_80 (1 / (CNT_TO_RAD_80))

#define EXT_RAD_TO_CNT_46 (1 / (EXT_CNT_TO_RAD_46))
#define EXT_RAD_TO_CNT_80 (1 / (EXT_CNT_TO_RAD_80))

#define EC_TIMEOUTMON 500

#define ELMO_DOF 33

#define ELMO_DOF_UPPER 18

#define ELMO_DOF_LOWER ELMO_DOF - ELMO_DOF_UPPER

#define LEG_DOF 12

#define CL_LOCK 20

#define UPPERBODY_DOF 21

#define PERIOD_NS 500000
#define SEC_IN_NSEC 1000000000UL

#define FORCE_CONTROL_MODE false

extern const char ifname_lower[];
extern char ifname_lower2[];
extern const char ifname_upper[];
extern char ifname_upper2[];

extern const int starting_point;

enum ELMO
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

namespace MODEL
{

    enum MODEL
    {
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
        R_Wrist2_Joint
    };

}

extern const char ELMO_NAME[ELMO_DOF][20];

//pos[i] = pos_elmo[JointMap[i]]
extern const int JointMap[ELMO_DOF];

//pos_elmo[i] = pos[JointMap2[i]]
extern const int JointMap2[ELMO_DOF];

extern const double CNT2RAD[ELMO_DOF];
extern const double EXTCNT2RAD[ELMO_DOF] ;

extern const double RAD2CNT[ELMO_DOF] ;

extern const double EXTRAD2CNT[ELMO_DOF] ;

extern const double NM2CNT[ELMO_DOF] ;

extern const int q_ext_mod_elmo_[ELMO_DOF];
//right -> left
//right hippitch front -> modval +
//left knee pitch front -> modval -
//right ankle pitch up -> modval

extern const double joint_velocity_limit[ELMO_DOF] ;

extern const double joint_upper_limit[ELMO_DOF] ;

extern const double joint_lower_limit[ELMO_DOF];

extern const double elmo_axis_direction[ELMO_DOF];

extern const double elmo_ext_axis_direction[ELMO_DOF];

extern const double upper_homming_minimum_required_length[ELMO_DOF];

extern const double pos_p_gain[ELMO_DOF];

extern const double pos_d_gain[ELMO_DOF];
