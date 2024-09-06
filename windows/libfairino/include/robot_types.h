#ifndef    ROBOTTYPES_H_
#define    ROBOTTYPES_H_

#include <stdio.h>

typedef  int errno_t;
typedef  unsigned char uint8_t;
typedef  signed char int8_t;

typedef  unsigned short int  uint16_t;

typedef  unsigned int uint32_t;

/**
* @brief Joint position data type
*/
typedef  struct
{
	double jPos[6];   /* Six joint positions, unit: deg */
}JointPos;

/**
* @brief Cartesian spatial location data type
*/
typedef struct
{
	double x;    /* X-axis coordinate, unit: mm  */
	double y;    /* Y-axis coordinate, unit: mm  */
	double z;    /* Z-axis coordinate, unit: mm  */
} DescTran;

/**
* @brief Euler Angle attitude data type
*/
typedef struct
{
	double rx;   /* Rotation Angle about fixed axis X, unit: deg  */
	double ry;   /* Rotation Angle about fixed axis y, unit: deg  */
	double rz;   /* Rotation Angle about fixed axis Z, unit: deg  */
} Rpy;

/**
*@brief Cartesian space pose type
*/
typedef struct
{
	DescTran tran;      /* Cartesian position  */
	Rpy rpy;            /* Cartesian space attitude  */
} DescPose;

/**
* @brief Extension axis position data type
*/
typedef  struct
{
	double ePos[4];   /* Position of four expansion shafts, unit: mm */
}ExaxisPos;

/**
* @brief The force component and torque component of the force sensor
*/
typedef struct
{
	double fx;  /* Component of force along the x axis, unit: N  */
	double fy;  /* Component of force along the y axis, unit: N  */
	double fz;  /* Component of force along the z axis, unit: N  */
	double tx;  /* Component of torque about the X-axis, unit: Nm */
	double ty;  /* Component of torque about the Y-axis, unit: Nm */
	double tz;  /* Component of torque about the Z-axis, unit: Nm */
} ForceTorque;


/**
* @brief  Spiral parameter data type
*/
typedef  struct
{
	int    circle_num;           /* Coil number  */
	float  circle_angle;         /* Spiral Angle  */
	float  rad_init;             /* Initial radius of spiral, unit: mm  */
	float  rad_add;              /* Radius increment  */
	float  rotaxis_add;          /* Increment in the direction of the axis of rotation  */
	unsigned int rot_direction;  /* Rotation direction, 0- clockwise, 1- counterclockwise  */
}SpiralParam;

#pragma pack(push)
#pragma pack(1)
typedef struct ROBOT_AUX_STATE
{
	uint8_t servoId;
	int servoErrCode;
	int servoState;
	double servoPos;
	float servoVel;
	float servoTorque;
} robot_aux_state;
#pragma pack(pop)


#pragma pack(push)
#pragma pack(1)
typedef struct _EXT_AXIS_STATUS
{
	double pos;           //Expansion axis position
	double vel;           //Spread axis velocity
	int errorCode;        //Expansion axis error code
	uint8_t ready;        //Servo ready
	uint8_t inPos;        //Servo in place
	uint8_t alarm;        //Servo alarm
	uint8_t flerr;        //Following error
	uint8_t nlimit;       //Negative limit
	uint8_t pLimit;       //Positive limit
	uint8_t mdbsOffLine;  //The drive 485 bus is offline
	uint8_t mdbsTimeout;  //The communication between the controller card and the control box 485 times out
	uint8_t homingStatus; //The expansion axis homing state
}EXT_AXIS_STATUS;
#pragma pack(pop)



#pragma pack(push)
#pragma pack(1)

typedef struct _ROBOT_STATE_PKG
{
	uint16_t frame_head;
	uint8_t  frame_cnt;
	uint16_t data_len;
	uint8_t  program_state;  //Program running status, 1- stop;2- Run; 3- Pause
	uint8_t  robot_state;//Robot motion state, 1- stop; 2- Run; 3- Pause; 4- Drag  
	int      main_code;
	int      sub_code;
	uint8_t  robot_mode;//Robot mode, 0-automatic mode; 1- Manual mode
	double   jt_cur_pos[6];//Current joint position
	double   tl_cur_pos[6];//Current tool position
	double   flange_cur_pos[6];//Current pose of end flange
	double   actual_qd[6];//Robot current joint speed
	double   actual_qdd[6];//Current joint acceleration of robot
	double   target_TCP_CmpSpeed[2];//Robot TCP synthesis command speed
	double   target_TCP_Speed[6]; //Robot TCP command speed
	double   actual_TCP_CmpSpeed[2]; //Robot TCP synthesizes actual speed
	double   actual_TCP_Speed[6];//Robot TCP actual speed
	double   jt_cur_tor[6];//Current torque 
	int      tool;//tool number
	int      user; //workpiece number
	uint8_t  cl_dgt_output_h;//Digital output 15-8
	uint8_t  cl_dgt_output_l;//Digital output 7-0
	uint8_t  tl_dgt_output_l; //tool digital output7-0
	uint8_t  cl_dgt_input_h;//Digital input 15-8
	uint8_t  cl_dgt_input_l;//Digital input 7-0
	uint8_t  tl_dgt_input_l;//tool Digital input 7-0
	uint16_t cl_analog_input[2];//Control box analog input
	uint16_t tl_anglog_input; //Tool analog input
	double   ft_sensor_raw_data[6];//Force/torque sensor raw data
	double   ft_sensor_data[6];//Force/torque sensor data
	uint8_t  ft_sensor_active;//Force/torque sensor active status, 0-reset, 1-activated
	uint8_t  EmergencyStop;//Emergency stop sign
	int      motion_done;//Position signal
	uint8_t  gripper_motiondone;//gripper movement complete signal
	int      mc_queue_len; //Motion queue length
	uint8_t  collisionState;//Collision detection, 1- collision; 0- No collision
	int      trajectory_pnum; //Track point number
	uint8_t  safety_stop0_state;  /* Safety stop signal SI0 */
	uint8_t  safety_stop1_state;  /* Safety stop signal SI1 */
	uint8_t  gripper_fault_id;     /* gripper error number */
	uint16_t gripper_fault;       /* Gripper fault */
	uint16_t gripper_active;      /* Gripper active status */
	uint8_t  gripper_position;    /* Gripper position */
	int8_t   gripper_speed;        /* Gripper speed */
	int8_t   gripper_current;     /* Gripper current */
	int      gripper_temp;        /* Gripper temperature */
	int      gripper_voltage;     /* Gripper voltage */
	robot_aux_state aux_state;/* 485Extended axis state */
	EXT_AXIS_STATUS extAxisStatus[4];  /* UDPExtended axis state */
	uint16_t extDIState[8];        //Extended DI
	uint16_t extDOState[8];        //Extended DO
	uint16_t extAIState[4];        //Extended AI
	uint16_t extAOState[4];        //Extended AO
	int rbtEnableState;            //robot enable state
	uint16_t check_sum;            /* Sum check */
}ROBOT_STATE_PKG;

#pragma pack(pop)


#endif
