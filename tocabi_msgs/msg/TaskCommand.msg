# CoM/Pelvis Control
float64 ratio # left foot 0 ----- 1 right foot
float64 height

float64 pelv_pitch
float64 roll
float64 pitch
float64 yaw

uint32 solver
uint32 contactredis

bool maintain_lc

# Contact 
bool left
bool right

# Contact 
bool left_foot
bool right_foot
bool left_hand
bool right_hand

# Control Duration And Mode
float64 time
int32 mode

# Left/Right Arm Control
float64 l_x
float64 l_y
float64 l_z
float64 l_roll
float64 l_pitch
float64 l_yaw

float64 r_x
float64 r_y
float64 r_z
float64 r_roll
float64 r_pitch
float64 r_yaw

# Walking
uint32 walking_enable
uint32 pattern
uint32 pattern2
uint32 comcontrol
uint32 ik_mode
bool dob
bool imu
bool mom
uint32 first_foot_step
float32 x
float32 y
float32 z
float32 walking_height
float32 theta
float32 step_length_x
float32 step_length_y

# TaskGain
bool customTaskGain
float32 acc_p
float32 pos_p
float32 pos_d
float32 ang_p
float32 ang_d
