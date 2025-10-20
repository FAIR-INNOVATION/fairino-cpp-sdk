#ifndef    ROBOTTYPES_H_
#define    ROBOTTYPES_H_

#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <string.h>
#include <sstream>

typedef  int errno_t;
typedef  unsigned char uint8_t;
typedef  signed char int8_t;

typedef  unsigned short int  uint16_t;

typedef  unsigned int uint32_t;

/**
* @brief Joint position data type
*/
typedef struct JointPos
{
	double jPos[6];   /* Six joint positions, unit: deg */
	JointPos(double J1, double J2, double J3, double J4, double J5, double J6)
	{
		jPos[0] = J1;
		jPos[1] = J2;
		jPos[2] = J3;
		jPos[3] = J4;
		jPos[4] = J5;
		jPos[5] = J6;
	}

	JointPos()
	{

	}
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
typedef struct DescPose
{
	DescTran tran;      /* Cartesian position  */
	Rpy rpy;            /* Cartesian space attitude  */
	DescPose(double x, double y, double z, double rx, double ry, double rz)
	{
		tran.x = x;
		tran.y = y;
		tran.z = z;

		rpy.rx = rx;
		rpy.ry = ry;
		rpy.rz = rz;
	}

	DescPose()
	{

	}
} DescPose;

/**
* @brief Extension axis position data type
*/
typedef  struct ExaxisPos
{
	double ePos[4];   /* Position of four expansion shafts, unit: mm */
	ExaxisPos(double axis1, double axis2, double axis3, double axis4)
	{
		ePos[0] = axis1;
		ePos[1] = axis2;
		ePos[2] = axis3;
		ePos[3] = axis4;
	}

	ExaxisPos()
	{

	}

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
typedef struct
{
	int circle_num;              /* Coil number  */
	float circle_angle;          /* Spiral Angle  */
	float rad_init;              /* Initial radius of spiral, unit: mm  */
	float rad_add;               /* Radius increment  */
	float rotaxis_add;           /* Increment in the direction of the axis of rotation  */
	unsigned int rot_direction;  /* Rotation direction, 0- clockwise, 1- counterclockwise  */
	int velAccMode;              /* Velocity and acceleration parameter mode: 0-constant angular velocity; 1-constant linear velocity */
}SpiralParam;

typedef struct AxleComParam
{
	int baudRate;    // Baud rate: 1-9600, 2-14400, 3-19200, 4-38400, 5-56000, 6-67600, 7-115200, 8-128000.
	int dataBit;     // Data bit: Data bit support (8,9), currently commonly used 8
	int stopBit;     // Stop bits: 1-1, 2-0.5, 3-2, 4-1.5, usually 1
	int verify;      // Parity bits: 0-none, 1-Odd, 2-Even, usually 0.
	int timeout;     // Timeout period: 1 to 1000ms. Set a proper time parameter based on the peripherals
	int timeoutTimes;   // Number of timeout times: 1 to 10. The system mainly resends timeout to reduce occasional exceptions and improve user experience
	int period;      // Periodic command interval: 1 to 1000ms. This parameter is used to specify the interval for sending periodic commands

	AxleComParam()
	{

	}

	AxleComParam(int _baudRate, int _dataBit, int _stopBit, int _verify, int _timeout, int _timeoutTimes, int _period)
	{
		baudRate = _baudRate;
		dataBit = _dataBit;
		stopBit = _stopBit;
		verify = _verify;
		timeout = _timeout;
		timeoutTimes = _timeoutTimes;
		period = _period;
	}
}AxleComParam;

/**
 * @brief robot time
 */
#pragma pack(push)
#pragma pack(1)
typedef struct RobotTime
{
	uint16_t year = 0;
	uint8_t mouth = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint8_t second = 0;
	uint16_t millisecond = 0;

	RobotTime()
	{

	}

	std::string ToString()
	{
		std::string rtn = std::to_string(year) + "-" + std::to_string(mouth) + "-" + std::to_string(day) + " " + std::to_string(hour) + ":" + std::to_string(minute) + ":" + std::to_string(second) + "." + std::to_string(millisecond);
		
		return rtn;
	}

}RobotTime;
#pragma pack(pop)

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
typedef struct _WELDING_BREAKOFF_STATE
{
	uint8_t breakOffState;  //º¸½ÓÖÐ¶Ï×´Ì¬
	uint8_t weldArcState;   //º¸½Óµç»¡ÖÐ¶Ï×´Ì¬
}WELDING_BREAKOFF_STATE;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)

/**
 * @brief  feedback packet of robot controller state
 */
typedef struct _ROBOT_STATE_PKG
{
	uint16_t frame_head;      // Frame header, fixed as 0x5A5A 
	uint8_t  frame_cnt;       // Frame counter, cycles 0-255 
	uint16_t data_len;        // Length of data content 
	uint8_t  program_state;   // Program running state: 1-Stop; 2-Run; 3-Pause; 
	uint8_t  robot_state;     // Robot motion state: 1-Stop; 2-Run; 3-Pause; 4-Drag 
	int      main_code;       // Main error code 
	int      sub_code;        // Sub error code 
	uint8_t  robot_mode;      // Robot mode: 1-Manual mode; 0-Auto mode; 
	double   jt_cur_pos[6];   // Current joint positions of 6 axes, unit: deg 
	double   tl_cur_pos[6];   // Current tool position
			// tl_cur_pos[0]: Position along x-axis, unit: mm,
			// tl_cur_pos[1]: Position along y-axis, unit: mm,
			// tl_cur_pos[2]: Position along z-axis, unit: mm,
			// tl_cur_pos[3]: Rotation angle around fixed X-axis, unit: deg
			// tl_cur_pos[4]: Rotation angle around fixed y-axis, unit: deg
			// tl_cur_pos[5]: Rotation angle around fixed z-axis, unit: deg 
	double   flange_cur_pos[6]; // Current flange position
			// flange_cur_pos[0]: Position along x-axis, unit: mm,
			// flange_cur_pos[1]: Position along y-axis, unit: mm,
			// flange_cur_pos[2]: Position along z-axis, unit: mm,
			// flange_cur_pos[3]: Rotation angle around fixed X-axis, unit: deg
			// flange_cur_pos[4]: Rotation angle around fixed y-axis, unit: deg
			// flange_cur_pos[5]: Rotation angle around fixed z-axis, unit: deg 
	double   actual_qd[6];      // Current joint velocities of 6 axes, unit: deg/s 
	double   actual_qdd[6];     // Current joint accelerations of 6 axes, unit: deg/s^2 
	double   target_TCP_CmpSpeed[2]; 
		// target_TCP_CmpSpeed[0]: TCP composite command speed (position), unit: mm/s
		// target_TCP_CmpSpeed[1]: TCP composite command speed (orientation), unit: deg/s 
	double   target_TCP_Speed[6];   // TCP command speed
		// target_TCP_Speed[0]: Velocity along x-axis, unit: mm/s,
		// target_TCP_Speed[1]: Velocity along y-axis, unit: mm/s,
		// target_TCP_Speed[2]: Velocity along z-axis, unit: mm/s,
		// target_TCP_Speed[3]: Angular velocity around fixed X-axis, unit: deg/s
		// target_TCP_Speed[4]: Angular velocity around fixed y-axis, unit: deg/s
		// target_TCP_Speed[5]: Angular velocity around fixed z-axis, unit: deg/s 
	double   actual_TCP_CmpSpeed[2];
		// actual_TCP_CmpSpeed[0]: TCP composite actual speed (position), unit: mm/s
		// actual_TCP_CmpSpeed[1]: TCP composite actual speed (orientation), unit: deg/s 
	double   actual_TCP_Speed[6];   // TCP actual speed
		// actual_TCP_Speed[0]: Velocity along x-axis, unit: mm/s,
		// actual_TCP_Speed[1]: Velocity along y-axis, unit: mm/s,
		// actual_TCP_Speed[2]: Velocity along z-axis, unit: mm/s,
		// actual_TCP_Speed[3]: Angular velocity around fixed X-axis, unit: deg/s
		// actual_TCP_Speed[4]: Angular velocity around fixed y-axis, unit: deg/s
		// actual_TCP_Speed[5]: Angular velocity around fixed z-axis, unit: deg/s 
	double   jt_cur_tor[6];      // Current torque of 6 axes, unit: N¡¤m 
	int      tool;               // Applied tool coordinate system number 
	int      user;               // Applied workpiece coordinate system number 
	uint8_t  cl_dgt_output_h;    // Control box digital IO output 15-8 
	uint8_t  cl_dgt_output_l;    // Control box digital IO output 7-0 
	uint8_t  tl_dgt_output_l;    // Tool digital IO output 7-0, only bit0-bit1 valid 
	uint8_t  cl_dgt_input_h;     // Control box digital IO input 15-8 
	uint8_t  cl_dgt_input_l;     // Control box digital IO input 7-0 
	uint8_t  tl_dgt_input_l;     // Tool digital IO input 7-0, only bit0-bit1 valid 
	uint16_t cl_analog_input[2]; // cl_analog_input[0]: Control box analog input 0
								 // cl_analog_input[1]: Control box analog input 1 
	uint16_t tl_anglog_input;    // Tool analog input 
	double   ft_sensor_raw_data[6]; // Force-torque sensor raw data
		// ft_sensor_raw_data[0]: Force along x-axis, unit: N
		// ft_sensor_raw_data[1]: Force along y-axis, unit: N
		// ft_sensor_raw_data[2]: Force along z-axis, unit: N
		// ft_sensor_raw_data[3]: Torque around x-axis, unit: Nm
		// ft_sensor_raw_data[4]: Torque around y-axis, unit: Nm
		// ft_sensor_raw_data[5]: Torque around z-axis, unit: Nm 
	double   ft_sensor_data[6];     // Force-torque sensor data
	    // ft_sensor_data[0]: Force along x-axis, unit: N
		// ft_sensor_data[1]: Force along y-axis, unit: N
		// ft_sensor_data[2]: Force along z-axis, unit: N
		// ft_sensor_data[3]: Torque around x-axis, unit: Nm
		// ft_sensor_data[4]: Torque around y-axis, unit: Nm
		// ft_sensor_data[5]: Torque around z-axis, unit: Nm 
	uint8_t  ft_sensor_active;   // Force-torque sensor active state: 0-Reset, 1-Active 
	uint8_t  EmergencyStop;      // Emergency stop flag: 0-Not pressed, 1-Pressed 
	int      motion_done;        // Motion completion signal: 1-Completed, 0-Not completed 
	uint8_t  gripper_motiondone; // Gripper motion completion signal: 1-Completed, 0-Not completed 
	int      mc_queue_len;       // Motion command queue length 
	uint8_t  collisionState;     // Collision detection: 1-Collision, 0-No collision 
	int      trajectory_pnum;    // Trajectory point number 
	uint8_t  safety_stop0_state; // Safety stop signal SI0 
	uint8_t  safety_stop1_state; // Safety stop signal SI1 
	uint8_t  gripper_fault_id;   // Faulty gripper number 
	uint16_t gripper_fault;      // Gripper fault 
	uint16_t gripper_active;     // Gripper active state 
	uint8_t  gripper_position;   // Gripper position 
	int8_t   gripper_speed;      // Gripper speed 
	int8_t   gripper_current;    // Gripper current 
	int      gripper_temp;       // Gripper temperature 
	int      gripper_voltage;    // Gripper voltage 
	robot_aux_state aux_state;   // 485 axis state
	EXT_AXIS_STATUS extAxisStatus[4];  // UDP extended axis status 
	uint16_t extDIState[8];        //Extended DI input
	uint16_t extDOState[8];        //Extended DO output
	uint16_t extAIState[4];        //Extended AI input
	uint16_t extAOState[4];        //Extended AO output
	int rbtEnableState;            //Robot enable state
	double   jointDriverTorque[6];        //Robot joint driver torque
	double   jointDriverTemperature[6];   //Robot joint driver temperature
	RobotTime robotTime;           //Robot system time
	int softwareUpgradeState;      //Robot software upgrade state
	uint16_t endLuaErrCode;        //End LUA running state
	uint16_t cl_analog_output[2];  //Control box analog output
	uint16_t tl_analog_output;     //Tool analog output
	float gripperRotNum;           //Rotary gripper current rotation count
	uint8_t gripperRotSpeed;       //Rotary gripper current rotation speed percentage
	uint8_t gripperRotTorque;      //Rotary gripper current rotation torque percentage
	WELDING_BREAKOFF_STATE weldingBreakOffState;  //Welding breakoff state
	double jt_tgt_tor[6];          //Joint command torque
	int smartToolState;            //SmartTool handle button state
	float wideVoltageCtrlBoxTemp;         //Temperature of wide voltage control box
	uint16_t wideVoltageCtrlBoxFanCurrent;//Wide voltage control box fan current (Ma)
	double toolCoord[6];		//Tool coordinate system
	double wobjCoord[6];		//Workpiece coordinate system
	double extoolCoord[6];		//External tool coordinate system
	double exAxisCoord[6];		//Extended axis coordinate system
	double load;				//Load mass
	double loadCog[3];			//Load center of gravity
	double lastServoTarget[6];	//The last ServoJ target position in the queue
	int servoJCmdNum;			//servoJ command count
	uint16_t check_sum;			//check sum
}ROBOT_STATE_PKG;

#pragma pack(pop)


#endif
