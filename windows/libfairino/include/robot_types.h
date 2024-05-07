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

typedef struct ROBOT_AUX_STATE
{
	uint8_t servoId;
	int servoErrCode;
	int servoState;
	double servoPos;
	float servoVel;
	float servoTorque;
} robot_aux_state;
#pragma pack(push)
#pragma pack(1)

typedef struct _ROBOT_STATE_PKG
{
	uint16_t frame_head;
	uint8_t  frame_cnt;
	uint16_t data_len;
	uint8_t  program_state;
	uint8_t  robot_state;
	int      main_code;
	int      sub_code;
	uint8_t  robot_mode;
	double   jt_cur_pos[6];
	double   tl_cur_pos[6];
	double   flange_cur_pos[6];
	double   actual_qd[6];
	double   actual_qdd[6];
	double   target_TCP_CmpSpeed[2];
	double   target_TCP_Speed[6];
	double   actual_TCP_CmpSpeed[2];
	double   actual_TCP_Speed[6];
	double   jt_cur_tor[6];
	int      tool;
	int      user;
	uint8_t  cl_dgt_output_h;
	uint8_t  cl_dgt_output_l;
	uint8_t  tl_dgt_output_l;
	uint8_t  cl_dgt_input_h;
	uint8_t  cl_dgt_input_l;
	uint8_t  tl_dgt_input_l;
	uint16_t cl_analog_input[2];
	uint16_t tl_anglog_input;
	double   ft_sensor_raw_data[6];
	double   ft_sensor_data[6];
	uint8_t  ft_sensor_active;
	uint8_t  EmergencyStop;
	int      motion_done;
	uint8_t  gripper_motiondone;
	int      mc_queue_len;
	uint8_t  collisionState;
	int      trajectory_pnum;
    uint8_t  safety_stop0_state;  /* safety stop signal SI0 */
    uint8_t  safety_stop1_state;  /* safety stop signal SI1 */
    uint8_t  gripper_fault_id;    /* Wrong gripper number */
    uint16_t gripper_fault;       /* Gripper failure */
    uint16_t gripper_active;      /* Gripper activation status */
    uint8_t  gripper_position;    /* Gripper position */
    int8_t   gripper_speed;       /* Gripper speed */
    int8_t   gripper_current;     /* Gripper current */
    int      gripper_temp;        /* Gripper temperature */
    int      gripper_voltage;     /* Gripper voltage */
	robot_aux_state aux_state;
}ROBOT_STATE_PKG;

#pragma pack(pop)


#endif
