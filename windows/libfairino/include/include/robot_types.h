#ifndef    ROBOTTYPES_H_
#define    ROBOTTYPES_H_

#include <stdio.h>

typedef  int errno_t;
typedef  unsigned char uint8_t;
typedef  signed char int8_t;

typedef  unsigned short int  uint16_t;

typedef  unsigned int uint32_t;

/**
 * @brief 关节位置数据类型
 */
typedef  struct
{
	double jPos[6];   /* 六个关节位置，单位deg */
}JointPos;

/**
* @brief 笛卡尔空间位置数据类型
*/
typedef struct
{
	double x;    /* x轴坐标，单位mm  */
	double y;    /* y轴坐标，单位mm  */
	double z;    /* z轴坐标，单位mm  */
} DescTran;

/**
* @brief 欧拉角姿态数据类型
*/
typedef struct
{
	double rx;   /* 绕固定轴X旋转角度，单位：deg  */
	double ry;   /* 绕固定轴Y旋转角度，单位：deg  */
	double rz;   /* 绕固定轴Z旋转角度，单位：deg  */
} Rpy;

/**
 *@brief 笛卡尔空间位姿类型
 */
typedef struct
{
	DescTran tran;      /* 笛卡尔空间位置  */
	Rpy rpy;			/* 笛卡尔空间姿态  */
} DescPose;

/**
 * @brief 扩展轴位置数据类型
 */
typedef  struct
{
	double ePos[4];   /* 四个扩展轴位置，单位mm */
}ExaxisPos;

/**
 * @brief 力传感器的受力分量和力矩分量
 */
typedef struct
{
	double fx;  /* 沿x轴受力分量，单位N  */
	double fy;  /* 沿y轴受力分量，单位N  */
	double fz;  /* 沿z轴受力分量，单位N  */
	double tx;  /* 绕x轴力矩分量，单位Nm */
	double ty;  /* 绕y轴力矩分量，单位Nm */
	double tz;  /* 绕z轴力矩分量，单位Nm */
} ForceTorque;


/**
 * @brief  螺旋参数数据类型
 */
typedef  struct
{
	int    circle_num;           /* 螺旋圈数  */
	float  circle_angle;         /* 螺旋倾角  */
	float  rad_init;             /* 螺旋初始半径，单位mm  */
	float  rad_add;              /* 半径增量  */
	float  rotaxis_add;          /* 转轴方向增量  */
	unsigned int rot_direction;  /* 旋转方向，0-顺时针，1-逆时针  */
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
    uint8_t  safety_stop0_state;  /* 安全停止信号SI0 */
    uint8_t  safety_stop1_state;  /* 安全停止信号SI1 */
    uint8_t  gripper_fault_id;    /* 错误夹爪号 */
    uint16_t gripper_fault;       /* 夹爪故障 */
    uint16_t gripper_active;      /* 夹爪激活状态 */
    uint8_t  gripper_position;    /* 夹爪位置 */
    int8_t   gripper_speed;       /* 夹爪速度 */
    int8_t   gripper_current;     /* 夹爪电流 */
    int      gripper_temp;        /* 夹爪温度 */
    int      gripper_voltage;     /* 夹爪电压 */
	robot_aux_state aux_state;
}ROBOT_STATE_PKG;

#pragma pack(pop)


#endif
