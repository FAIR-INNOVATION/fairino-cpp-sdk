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
		jPos[0] = 0.0;
		jPos[1] = 0.0;
		jPos[2] = 0.0;
		jPos[3] = 0.0;
		jPos[4] = 0.0;
		jPos[5] = 0.0;
	}
}JointPos;

/**
* @brief Cartesian spatial location data type
*/
typedef struct DescTran
{
	double x;    /* X-axis coordinate, unit: mm  */
	double y;    /* Y-axis coordinate, unit: mm  */
	double z;    /* Z-axis coordinate, unit: mm  */
	DescTran(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	DescTran()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
} DescTran;

/**
* @brief Euler Angle attitude data type
*/
typedef struct Rpy
{
	double rx;   /* Rotation Angle about fixed axis X, unit: deg  */
	double ry;   /* Rotation Angle about fixed axis y, unit: deg  */
	double rz;   /* Rotation Angle about fixed axis Z, unit: deg  */

	Rpy(double _rx, double _ry, double _rz)
	{
		rx = _rx;
		ry = _ry;
		rz = _rz;
	}

	Rpy()
	{
		rx = 0.0;
		ry = 0.0;
		rz = 0.0;
	}
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
		tran = { 0.0, 0.0, 0.0 };
		rpy = { 0.0, 0.0, 0.0 };
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
	double targetJointPos[6];   // 6 joint command positions, unit: deg
	double targetJointVel[6];   // 6 joint command velocities, unit: deg/s
	double targetJointAcc[6];   // 6 joint command accelerations, unit: deg/s²
	double targetJointCurrent[6]; // 6 joint command currents, unit: A
	double actualJointCurrent[6]; // 6 joint actual currents, unit: A
	double actualTCPForce[6];   // Robot end torque: x,y,z,rx,ry,rz, unit: Nm
	double targetTCPPos[6];     // Robot TCP command position: x,y,z,rx,ry,rz, unit: mm
	uint8_t collisionLevel[6];  // Robot collision level
	double speedScaleManual;    // Manual mode global speed percentage
	double speedScaleAuto;      // Auto mode global speed percentage
	int luaLineNum;             // Current lua program line number
	uint8_t abnomalStop;        // 0-Normal; 1-Abnormal
	char currentLuaFileName[256]; // Current running lua program name
	uint8_t programTotalLine;   // Lua program total lines
	uint8_t safetyBoxSingal[6]; // Robot button box button state
	double weldVoltage;         // Welding voltage, unit: V
	double weldCurrent;         // Welding current, unit: A
	double weldTrackVel;        // Seam tracking speed, unit: mm/s
	uint8_t tpdException;       // TPD trajectory count exceeded: 0-Not exceeded, 1-Exceeded
	uint8_t alarmRebootRobot;   // Warning: 1-Release emergency stop and power cycle control box, 2-Joint communication error, power cycle control box
	uint8_t modbusMasterConnect;  // bits 0-7 correspond to ModbusTCP master 0-7 connection status: 0-Disconnected, 1-Connected
	uint8_t modbusSlaveConnect;   // ModbusTCP slave connection status: 0-Disconnected, 1-Connected
	uint8_t btnBoxStopSignal;     // Button box emergency stop signal: 0-Emergency stop released, 1-Emergency stop pressed
	uint8_t dragAlarm;            // Drag warning: 0-No alarm, 1-Alarm, 2-Position feedback abnormal, no switch
	uint8_t safetyDoorAlarm;      // Safety door warning: 0-Safety door closed, 1-Safety door open
	uint8_t safetyPlaneAlarm;     // Safety wall warning: 0-Not entered safety wall, 1-Entered safety wall
	uint8_t motonAlarm;           // Motion warning
	uint8_t interfaceAlarm;       // Interference zone warning
	int udpCmdState;              // UDP communication connection status on port 20007
	uint8_t weldReadyState;       // Welder ready state
	uint8_t alarmCheckEmergStopBtn;  // 0-Normal; 1-Communication error, check if emergency stop button is released
	uint8_t tsTmCmdComError;      // 0-Normal; 1-Torque command communication failure
	uint8_t tsTmStateComError;    // 0-Normal; 1-Torque state communication failure
	int ctrlBoxError;             // Control box error
	uint8_t safetyDataState;      // Safety data state flag: 0-Normal, 1-Abnormal
	uint8_t forceSensorErrState;  // Force sensor connection timeout error: bits 0-1 correspond to force sensor ID1-ID2
	uint8_t ctrlOpenLuaErrCode[4];  // 4 controller peripheral protocol error codes (500 error codes)
	uint8_t strangePosFlag;       // Singular pose flag: 0-Normal; 1-Singular pose
	uint8_t alarm;                // Warning
	uint8_t driverAlarm;          // Driver alarm axis number
	uint8_t aliveSlaveNumError;   // Active slave count error: 0-Normal; 1-Count error
	uint8_t slaveComError[8];     // Slave error: 0-Normal; 1-Slave offline; 2-Slave state inconsistent with configuration; 3-Slave not configured; 4-Slave configuration error; 5-Slave initialization error; 6-Slave mailbox communication initialization error
	uint8_t cmdPointError;        // Command point error
	uint8_t IOError;              // IO error
	uint8_t gripperError;         // Gripper error
	uint8_t fileError;            // File error
	uint8_t paraError;            // Parameter error
	uint8_t exaxisOutLimitError;  // External axis soft limit exceeded error
	uint8_t driverComError[6];    // Communication failure with driver
	uint8_t driverError;          // Driver communication failure axis number
	uint8_t outSoftLimitError;    // Soft limit exceeded error
	char axleGenComData[130];     // Robot end transparent transmission feedback data
	uint8_t socketConnTimeout;    // Socket connection timeout, bits 0-4: socket ID 1-4
	uint8_t socketReadTimeout;    // Socket read timeout, bits 0-4: socket ID 1-4
	uint8_t tsWebStateComErr;     // Web-torque communication failure: 0-Normal; 1-Failure
	uint16_t check_sum;			// Check sum
}ROBOT_STATE_PKG;

#pragma pack(pop)

enum class RobotState
{
	ProgramState = 3,           // Program running state: 1-Stop; 2-Run; 3-Pause
	RobotState = 4,             // Robot motion state: 1-Stop; 2-Run; 3-Pause; 4-Drag
	MainCode = 5,               // Main error code
	SubCode = 6,                // Sub error code
	RobotMode = 7,              // Robot mode: 1-Manual mode; 0-Auto mode
	JointCurPos = 8,            // Current joint positions of 6 axes, unit: deg
	ToolCurPos = 9,             // Current tool position: [0]x(mm), [1]y(mm), [2]z(mm), [3]RX(deg), [4]RY(deg), [5]RZ(deg)
	FlangeCurPos = 10,          // Current flange position: [0]x(mm), [1]y(mm), [2]z(mm), [3]RX(deg), [4]RY(deg), [5]RZ(deg)
	ActualJointVel = 11,        // Current joint velocities of 6 axes, unit: deg/s
	ActualJointAcc = 12,        // Current joint accelerations of 6 axes, unit: deg/s²
	TargetTCPCmpSpeed = 13,     // TCP composite command speed: [0]position(mm/s), [1]orientation(deg/s)
	TargetTCPSpeed = 14,        // TCP command speed: [0]Vx(mm/s), [1]Vy(mm/s), [2]Vz(mm/s), [3]ωx(deg/s), [4]ωy(deg/s), [5]ωz(deg/s)
	ActualTCPCmpSpeed = 15,     // TCP composite actual speed: [0]position(mm/s), [1]orientation(deg/s)
	ActualTCPSpeed = 16,        // TCP actual speed: [0]Vx(mm/s), [1]Vy(mm/s), [2]Vz(mm/s), [3]ωx(deg/s), [4]ωy(deg/s), [5]ωz(deg/s)
	ActualJointTorque = 17,     // Current torque of 6 axes, unit: N·m
	Tool = 18,                  // Applied tool coordinate system number
	User = 19,                  // Applied workpiece coordinate system number
	ClDgtOutputH = 20,          // Control box digital IO output 15-8
	ClDgtOutputL = 21,          // Control box digital IO output 7-0
	TlDgtOutputL = 22,          // Tool digital IO output 7-0, only bit0-bit1 valid
	ClDgtInputH = 23,           // Control box digital IO input 15-8
	ClDgtInputL = 24,           // Control box digital IO input 7-0
	TlDgtInputL = 25,           // Tool digital IO input 7-0, only bit0-bit1 valid
	ClAnalogInput = 26,         // Control box analog input: [0]channel 0, [1]channel 1
	TlAnalogInput = 27,         // Tool analog input
	FtSensorRawData = 28,       // Force-torque sensor raw data: [0]Fx(N), [1]Fy(N), [2]Fz(N), [3]Tx(Nm), [4]Ty(Nm), [5]Tz(Nm)
	FtSensorData = 29,          // Force-torque sensor data (processed): [0]Fx(N), [1]Fy(N), [2]Fz(N), [3]Tx(Nm), [4]Ty(Nm), [5]Tz(Nm)
	FtSensorActive = 30,        // Force-torque sensor active state: 0-Reset, 1-Active
	EmergencyStop = 31,         // Emergency stop flag: 0-Not pressed, 1-Pressed
	MotionDone = 32,            // Motion completion signal: 1-Completed, 0-Not completed
	GripperMotiondone = 33,     // Gripper motion completion signal: 1-Completed, 0-Not completed
	McQueueLen = 34,            // Motion command queue length
	CollisionState = 35,        // Collision detection: 1-Collision, 0-No collision
	TrajectoryPnum = 36,        // Trajectory point number
	SafetyStop0State = 37,      // Safety stop signal SI0
	SafetyStop1State = 38,      // Safety stop signal SI1
	GripperFaultId = 39,        // Faulty gripper number
	GripperFault = 40,          // Gripper fault
	GripperActive = 41,         // Gripper active state
	GripperPosition = 42,       // Gripper position
	GripperSpeed = 43,          // Gripper speed
	GripperCurrent = 44,        // Gripper current
	GripperTemp = 45,           // Gripper temperature
	GripperVoltage = 46,        // Gripper voltage
	AuxState = 47,              // 485 axis state
	ExtAxisStatus = 48,         // UDP extended axis status (4 axes)
	ExtDIState = 49,            // Extended DI input (8)
	ExtDOState = 50,            // Extended DO output (8)
	ExtAIState = 51,            // Extended AI input (4)
	ExtAOState = 52,            // Extended AO output (4)
	RbtEnableState = 53,        // Robot enable state
	JointDriverTorque = 54,     // Robot joint driver torque (6 joints)
	JointDriverTemperature = 55,// Robot joint driver temperature (6 joints)
	RobotTime = 56,             // Robot system time
	SoftwareUpgradeState = 57,  // Robot software upgrade state
	EndLuaErrCode = 58,         // End LUA running state
	ClAnalogOutput = 59,        // Control box analog output (2 channels)
	TlAnalogOutput = 60,        // Tool analog output
	GripperRotNum = 61,         // Rotary gripper current rotation count
	GripperRotSpeed = 62,       // Rotary gripper current rotation speed percentage
	GripperRotTorque = 63,      // Rotary gripper current rotation torque percentage
	WeldingBreakOffState = 64,  // Welding breakoff state
	TargetJointTorque = 65,     // Joint command torque (6 joints)
	SmartToolState = 66,        // SmartTool handle button state
	WideVoltageCtrlBoxTemp = 67,// Temperature of wide voltage control box
	WideVoltageCtrlBoxFanCurrent = 68, // Wide voltage control box fan current (mA)
	ToolCoord = 69,             // Tool coordinate system: x,y,z,rx,ry,rz
	WobjCoord = 70,             // Workpiece coordinate system: x,y,z,rx,ry,rz
	ExtoolCoord = 71,           // External tool coordinate system: x,y,z,rx,ry,rz
	ExAxisCoord = 72,           // Extended axis coordinate system: x,y,z,rx,ry,rz
	Load = 73,                  // Load mass
	LoadCog = 74,               // Load center of gravity: x,y,z
	LastServoTarget = 75,       // The last ServoJ target position in the queue (6 joints)
	ServoJCmdNum = 76,          // servoJ command count
	TargetJointPos = 77,        // 6 joint command positions, unit: deg
	TargetJointVel = 78,        // 6 joint command velocities, unit: deg/s
	TargetJointAcc = 79,        // 6 joint command accelerations, unit: deg/s²
	TargetJointCurrent = 80,    // 6 joint command currents, unit: A
	ActualJointCurrent = 81,    // 6 joint actual currents, unit: A
	ActualTCPForce = 82,        // Robot end torque: x,y,z,rx,ry,rz, unit: Nm
	TargetTCPPos = 83,          // Robot TCP command position: x,y,z,rx,ry,rz, unit: mm
	CollisionLevel = 84,        // Robot collision level (6)
	SpeedScaleManual = 85,      // Manual mode global speed percentage
	SpeedScaleAuto = 86,        // Auto mode global speed percentage
	LuaLineNum = 87,            // Current lua program line number
	AbnomalStop = 88,           // 0-Normal; 1-Abnormal
	CurrentLuaFileName = 89,    // Current running lua program name
	ProgramTotalLine = 90,      // Lua program total lines
	SafetyBoxSingal = 91,       // Robot button box button state (6)
	WeldVoltage = 92,           // Welding voltage, unit: V
	WeldCurrent = 93,           // Welding current, unit: A
	WeldTrackVel = 94,          // Seam tracking speed, unit: mm/s
	TpdException = 95,          // TPD trajectory count exceeded: 0-Not exceeded, 1-Exceeded
	AlarmRebootRobot = 96,      // Warning: 1-Release emergency stop and power cycle, 2-Joint communication error, power cycle
	ModbusMasterConnect = 97,   // bits 0-7: ModbusTCP master 0-7 connection status, 0-Disconnected, 1-Connected
	ModbusSlaveConnect = 98,    // ModbusTCP slave connection status: 0-Disconnected, 1-Connected
	BtnBoxStopSignal = 99,      // Button box emergency stop signal: 0-Released, 1-Pressed
	DragAlarm = 100,            // Drag warning: 0-No alarm, 1-Alarm, 2-Position feedback abnormal
	SafetyDoorAlarm = 101,      // Safety door warning: 0-Closed, 1-Open
	SafetyPlaneAlarm = 102,     // Safety wall warning: 0-Not entered, 1-Entered
	MotonAlarm = 103,           // Motion warning
	InterfaceAlarm = 104,       // Interference zone warning
	UdpCmdState = 105,          // UDP communication connection status on port 20007
	WeldReadyState = 106,       // Welder ready state
	AlarmCheckEmergStopBtn = 107, // 0-Normal; 1-Communication error, check emergency stop button
	TsTmCmdComError = 108,      // 0-Normal; 1-Torque command communication failure
	TsTmStateComError = 109,    // 0-Normal; 1-Torque state communication failure
	CtrlBoxError = 110,         // Control box error
	SafetyDataState = 111,      // Safety data state: 0-Normal, 1-Abnormal
	ForceSensorErrState = 112,  // Force sensor connection timeout: bits 0-1 for sensor ID1-ID2
	CtrlOpenLuaErrCode = 113,   // 4 controller peripheral protocol error codes (500 error codes)
	StrangePosFlag = 114,       // Singular pose flag: 0-Normal; 1-Singular pose
	Alarm = 115,                // Warning
	DriverAlarm = 116,          // Driver alarm axis number
	AliveSlaveNumError = 117,   // Active slave count error: 0-Normal; 1-Count error
	SlaveComError = 118,        // Slave error: 0-Normal; 1-Offline; 2-State mismatch; 3-Not configured; 4-Config error; 5-Init error; 6-Mailbox init error
	CmdPointError = 119,        // Command point error
	IOError = 120,              // IO error
	GripperError = 121,         // Gripper error
	FileError = 122,            // File error
	ParaError = 123,            // Parameter error
	ExaxisOutLimitError = 124,  // External axis soft limit exceeded error
	DriverComError = 125,       // Communication failure with driver (6 axes)
	DriverError = 126,          // Driver communication failure axis number
	OutSoftLimitError = 127,    // Soft limit exceeded error
	AxleGenComData = 128,       // Robot end transparent transmission feedback data
	SocketConnTimeout = 129,    // Socket connection timeout, bits 0-4: socket ID 1-4
	SocketReadTimeout = 130,    // Socket read timeout, bits 0-4: socket ID 1-4
	TsWebStateComErr = 131      // Web-torque communication failure: 0-Normal; 1-Failure
};

#endif
