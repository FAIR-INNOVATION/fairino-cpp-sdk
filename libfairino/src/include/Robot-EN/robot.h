#ifndef FRROBOT_H_
#define FRROBOT_H_

#ifdef WINDOWS_OPTION
	#define FR_LIB_EXPORT __declspec(dllexport)
#else
	#define FR_LIB_EXPORT 
#endif

#ifdef WIN32
#include <winsock2.h>
#endif // WIN32

#include "robot_types.h"
#include <iostream>
#include <list>
#include <vector>
#include <memory>

class FRTcpClient;

class FR_LIB_EXPORT FRRobot
{
public:
    /**
	 *@brief  Robot interface class constructor
	 */
    FRRobot();
    
	/**
    *@brief  Establish communication with the robot controller
    *@param  [in] ip  Controller IP address. The default value is 192.168.58.2
    *@return Error code
	 */
    errno_t  RPC(const char *ip);

	/**
	 * @brief  Close communication with robot controller
	 * @return Error code
	 */
    errno_t  CloseRPC();

	/**
    *@brief  Query the SDK version number
    *@param  [out] version  SDK version
    *@return  Error code
     */	 
	errno_t  GetSDKVersion(char *version);
	
	/**
    *@brief  Obtain Controller IP address
    *@param  [out] ip  Controller IP
    *@return  Error code
	 */
	errno_t  GetControllerIP(char *ip);
	
	/**
    *@brief  Control the robot to enter or exit the drag teaching mode
    *@param  [in] state 0-exit drag mode，1-enter the drag mode
    *@return  Error code
	 */
	errno_t  DragTeachSwitch(uint8_t state);
	
	/**
    *@brief  Check whether the robot is in drag mode
    *@param  [out] state 0-non-drag teaching mode，1-drag the teaching mode
    *@return  Error code
	 */
	errno_t  IsInDragTeach(uint8_t *state);
	
	/**
    *@brief  Enable or disable the function on or off the robot. By default, the function is enabled automatically after the robot is powered on
    *@param  [in] state  0-down-enable，1-upper enable
    *@return  Error code
	 */
	errno_t  RobotEnable(uint8_t state);
    
	/**
    *@brief Control robot hand/automatic mode
    *@param [in] mode 0-automatic mode，1-manual mode
    *@return Error code
	 */
    errno_t  Mode(int mode);
	
	/**
    *@brief  Jog point movement
    *@param  [in]  ref 0- node movement, 2- base coordinate system, 4- tool coordinate system, 8- workpiece coordinate system
    *@param  [in]  nb 1-joint 1(or axis x), 2-joint 2(or axis y), 3-joint 3(or axis z), 4-joint 4(or rotation about axis x), 5-joint 5(or rotation about axis y), 6-joint 6(or rotation about axis z)
    *@param  [in]  dir 0-negative correlation, 1-positive correlation
    *@param  [in]  vel The percentage of velocity,[0~100]
    *@param  [in]  acc The percentage of acceleration, [0~100]
    *@param  [in]  max_dis Maximum Angle of single click, unit: [°] or distance, unit: [mm]
    *@return  Error code
	 */
	errno_t  StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis);
	
	/**
    *@brief  Jog point dynamic deceleration stop
    *@param  [in]  ref  1- point stop, 3- point stop in base coordinate system, 5- point stop in tool coordinate system, 9- point stop in workpiece coordinate system
    *@return  Error code
	 */
	errno_t  StopJOG(uint8_t ref);
	
	/**
    *@brief The jog stops immediately
    *@return  Error code
	 */
	errno_t  ImmStopJOG(); 
	
	/**
    *@brief  Joint space motion
    *@param  [in] joint_pos  Target joint location, unit: deg
    *@param  [in] desc_pos   Target Cartesian position
    *@param  [in] tool  Tool coordinate number, range [0~14]
    *@param  [in] user  Workpiece coordinate number, range [0~14]
    *@param  [in] vel  Percentage of speed, range [0~100]
    *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] ovl  Velocity scaling factor, range[0~100]
    *@param  [in] epos  Position of expansion shaft, unit: mm
    *@param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos  The pose offset
    *@return  Error code
	 */
	errno_t  MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos);
	
	/**
	 *@brief  Joint space motion(Overloaded functions do not require the input of Cartesian positions)
	 *@param  [in] joint_pos  Target joint location, unit: deg
	 *@param  [in] tool  Tool coordinate number, range [0~14]
	 *@param  [in] user  Workpiece coordinate number, range [0~14]
	 *@param  [in] vel  Percentage of speed, range [0~100]
	 *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
	 *@param  [in] ovl  Velocity scaling factor, range[0~100]
	 *@param  [in] epos  Position of expansion shaft, unit: mm
	 *@param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
	 *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	 *@param  [in] offset_pos  The pose offset
	 *@return Error code
	 */
	errno_t  MoveJ(JointPos* joint_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos* epos, float blendT, uint8_t offset_flag, DescPose* offset_pos);

	/**
     * @brief Rectilinear motion in Cartesian space
     * @param [in] joint_pos  Target joint location, unit: deg
     * @param [in] desc_pos   Target Cartesian position
     * @param [in] tool  Tool coordinate number, range [0~14]
     * @param [in] user  Workpiece coordinate number, range [0~14]
     * @param [in] vel  Percentage of speed, range [0~100]
     * @param [in] acc  Acceleration percentage, range [0~100], not open for now
     * @param [in] ovl  Velocity scaling factor, range[0~100]
     * @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
     * @param [in] blendMode transition mode; 0- Internal cutting transition 1- Corner transition
	 * @param [in] epos  Position of expansion shaft, unit: mm
     * @param [in] search  0- no wire seeking, 1- wire seeking
     * @param [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
     * @param [in] offset_pos  The pose offset
	 * @param [in] velAccParamMode Speed-acceleration parameter mode 0- percentage 1- Physical velocity (mm/s) Acceleration (mm/s ²)
	 * @param [in] overSpeedStrategy  Overspeed handling strategy, 1-standard; 2- Reporting a wrong stop when speeding; 3- Adaptive deceleration, default is 0
	 * @param [in] speedPercent  The allowable deceleration threshold percentage [0-100], default 10%
	 * @return Error code
	 */
	errno_t MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos, int velAccParamMode = 0, int overSpeedStrategy = 0, int speedPercent = 10);

	/**
	 * @brief Rectilinear motion in Cartesian space(Overloaded function 2 does not require the input of joint positions)
	 * @param [in] desc_pos Target Cartesian position
	 * @param [in] tool Tool coordinate number, range [0~14]
	 * @param [in] user Workpiece coordinate number, range [0~14]
	 * @param [in] vel Percentage of speed, range [0~100]
	 * @param [in] acc Acceleration percentage, range [0~100], not open for now
	 * @param [in] ovl Velocity scaling factor, range[0~100]
	 * @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
	 * @param [in] blendMode transition mode; 0- Internal cutting transition 1- Corner transition
	 * @param [in] epos Position of expansion shaft, unit: mm
	 * @param [in] search 0- no wire seeking, 1- wire seeking
	 * @param [in] offset_flag 0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	 * @param [in] offset_pos The pose offset
	 * @param [in] config Robot inverse kinematics calculation joint configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration
	 * @param [in] velAccParamMode Speed-acceleration parameter mode 0- percentage 1- Physical velocity (mm/s) Acceleration (mm/s ²)
	 * @param [in] overSpeedStrategy Overspeed handling strategy, 1-standard; 2- Reporting a wrong stop when speeding; 3- Adaptive deceleration, default is 0
	 * @param [in] speedPercent The allowable deceleration threshold percentage [0-100], default 10%
	 * @return Error code
	 */
	errno_t MoveL(DescPose* desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode, ExaxisPos* epos, uint8_t search, uint8_t offset_flag, DescPose* offset_pos, int config = -1, int velAccParamMode = 0, int overSpeedStrategy = 0, int speedPercent = 10);


	/**
	*@brief  Rectilinear motion in Cartesian space
	*@param  [in] joint_pos  Target joint location, unit: deg
	*@param  [in] desc_pos   Target Cartesian position
	*@param  [in] tool  Tool coordinate number, range [0~14]
	*@param  [in] user  Workpiece coordinate number, range [0~14]
	*@param  [in] vel  Percentage of speed, range [0~100]
	*@param  [in] acc  Acceleration percentage, range [0~100], not open for now
	*@param  [in] ovl  Velocity scaling factor, range[0~100]
	*@param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
	*@param  [in] epos  Position of expansion shaft, unit: mm
	*@param  [in] search  0- no wire seeking, 1- wire seeking
	*@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	*@param  [in] offset_pos  The pose offset
	 *@param  [in] overSpeedStrategy  Overspeed handling strategy, 1-standard; 2- Reporting a wrong stop when speeding; 3- Adaptive deceleration, default is 0
	 *@param  [in] speedPercent  The allowable deceleration threshold percentage [0-100], default 10%
	*@return  Error code
	*/
	errno_t MoveL(JointPos* joint_pos, DescPose* desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos* epos, uint8_t search, uint8_t offset_flag, DescPose* offset_pos, int overSpeedStrategy = 0, int speedPercent = 10);


	/**
     *@brief  Circular arc motion in Cartesian space
     *@param  [in] joint_pos_p  Waypoint joint position, unit: deg
     *@param  [in] desc_pos_p   Waypoint Cartesian position
     *@param  [in] ptool  Tool coordinate number, range [0~14]
     *@param  [in] puser  Workpiece coordinate number, range [0~14]
     *@param  [in] pvel  Percentage of speed, range [0~100]
     *@param  [in] pacc  Acceleration percentage, range [0~100], not open for now
     *@param  [in] epos_p  Position of expansion shaft, unit: mm
     *@param  [in] poffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
     *@param  [in] offset_pos_p  The pose offset
     *@param  [in] joint_pos_t  Target joint position, unit: deg
     *@param  [in] desc_pos_t   Target point Cartesian position
     *@param  [in] ttool  Tool coordinate number, range [0~14]
     *@param  [in] tuser  Workpiece coordinate number, range [0~14]
     *@param  [in] tvel  Percentage of speed, range [0~100]
     *@param  [in] tacc  Acceleration percentage, range [0~100], not open for now
     *@param  [in] epos_t  Position of expansion shaft, unit: mm
     *@param  [in] toffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
     *@param  [in] offset_pos_t  The pose offset   
     *@param  [in] ovl  Velocity scaling factor, range[0~100]    
     *@param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
	 *@param  [in] velAccParamMode Speed-acceleration parameter mode 0- percentage 1- Physical velocity (mm/s) Acceleration (mm/s ²)
	 *@return  Error code
	 */		
	errno_t MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t, float ovl, float blendR, int velAccParamMode = 0);

	/**
	 *@brief  Circular arc motion in Cartesian space (Overloaded function does not require input of joint positions.)
	 *@param  [in] desc_pos_p   Waypoint Cartesian position
	 *@param  [in] ptool  Tool coordinate number, range [0~14]
	 *@param  [in] puser  Workpiece coordinate number, range [0~14]
	 *@param  [in] pvel  Percentage of speed, range [0~100]
	 *@param  [in] pacc  Acceleration percentage, range [0~100], not open for now
	 *@param  [in] epos_p  Position of expansion shaft, unit: mm
	 *@param  [in] poffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	 *@param  [in] offset_pos_p  The pose offset
	 *@param  [in] desc_pos_t   Target point Cartesian position
	 *@param  [in] ttool  Tool coordinate number, range [0~14]
	 *@param  [in] tuser  Workpiece coordinate number, range [0~14]
	 *@param  [in] tvel  Percentage of speed, range [0~100]
	 *@param  [in] tacc  Acceleration percentage, range [0~100], not open for now
	 *@param  [in] epos_t  Position of expansion shaft, unit: mm
	 *@param  [in] toffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	 *@param  [in] offset_pos_t  The pose offset
	 *@param  [in] ovl  Velocity scaling factor, range[0~100]
	 *@param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
	 *@param  [in] config Robot inverse kinematics calculation joint configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration
	 * @param  [in] velAccParamMode Speed-acceleration parameter mode 0- percentage 1- Physical velocity (mm/s) Acceleration (mm/s ²)
	 * @return  Error code
	 */
	errno_t MoveC(DescPose* desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos* epos_p, uint8_t poffset_flag, DescPose* offset_pos_p, DescPose* desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos* epos_t, uint8_t toffset_flag, DescPose* offset_pos_t, float ovl, float blendR, int config = -1, int velAccParamMode = 0);

	/**
     *@brief  Circular motion in Cartesian space
     *@param  [in] joint_pos_p  Path point 1 joint position, unit: deg
     *@param  [in] desc_pos_p   Waypoint 1 Cartesian position
     *@param  [in] ptool  Tool coordinate number, range [0~14]
     *@param  [in] puser  Workpiece coordinate number, range [0~14]
     *@param  [in] pvel  Percentage of speed, range [0~100]
     *@param  [in] pacc  Acceleration percentage, range [0~100], not open for now
     *@param  [in] epos_p  Position of expansion shaft, unit: mm
     *@param  [in] joint_pos_t  Joint position at waypoint 2, unit: deg
     *@param  [in] desc_pos_t   Waypoint 2 Cartesian position
     *@param  [in] ttool  Tool coordinate number, range [0~14]
     *@param  [in] tuser  Workpiece coordinate number, range [0~14]
     *@param  [in] tvel  Percentage of speed, range [0~100]
     *@param  [in] tacc  Acceleration percentage, range [0~100], not open for now
     *@param  [in] epos_t  Position of expansion shaft, unit: mm
     *@param  [in] ovl  Velocity scaling factor, range[0~100]   
     *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
     *@param  [in] offset_pos  The pose offset  
     *@param  [in] oacc Percentage of acceleration
	 *@param  [in] blendR -1: Block; 0-1000: Smooth radius
	 * @param  [in] velAccParamMode Speed-acceleration parameter mode 0- percentage 1- Physical velocity (mm/s) Acceleration (mm/s ²)
	 * @return  Error code
	 */
	errno_t Circle(JointPos* joint_pos_p, DescPose* desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos* epos_p, JointPos* joint_pos_t, DescPose* desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos* epos_t, float ovl, uint8_t offset_flag, DescPose* offset_pos, double oacc = 100.0, double blendR = -1, int velAccParamMode = 0);

	/**
	 *@brief  Circular motion in Cartesian space (Overloaded function does not require input of joint positions.)
	 *@param  [in] desc_pos_p   Waypoint 1 Cartesian position
	 *@param  [in] ptool  Tool coordinate number, range [0~14]
	 *@param  [in] puser  Workpiece coordinate number, range [0~14]
	 *@param  [in] pvel  Percentage of speed, range [0~100]
	 *@param  [in] pacc  Acceleration percentage, range [0~100], not open for now
	 *@param  [in] epos_p  Position of expansion shaft, unit: mm
	 *@param  [in] desc_pos_t   Waypoint 2 Cartesian position
	 *@param  [in] ttool  Tool coordinate number, range [0~14]
	 *@param  [in] tuser  Workpiece coordinate number, range [0~14]
	 *@param  [in] tvel  Percentage of speed, range [0~100]
	 *@param  [in] tacc  Acceleration percentage, range [0~100], not open for now
	 *@param  [in] epos_t  Position of expansion shaft, unit: mm
	 *@param  [in] ovl  Velocity scaling factor, range[0~100]
	 *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	 *@param  [in] offset_pos  The pose offset
	 *@param  [in] oacc Percentage of acceleration
	 *@param  [in] blendR -1: Block; 0-1000: Smooth radius
	 *@param  [in] config Robot inverse kinematics calculation joint configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration
	 * @param  [in] velAccParamMode Speed-acceleration parameter mode 0- percentage 1- Physical velocity (mm/s) Acceleration (mm/s ²)
	 * @return Error code
	 */
	errno_t Circle(DescPose* desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos* epos_p, DescPose* desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos* epos_t, float ovl, uint8_t offset_flag, DescPose* offset_pos, double oacc = 100.0, double blendR = -1, int config = -1, int velAccParamMode = 0);


	/**
	 *@brief  Spiral motion in Cartesian space
	 *@param  [in] joint_pos  Target joint location, unit: deg
	 *@param  [in] desc_pos   Target Cartesian position
	 *@param  [in] tool  Tool coordinate number, range [0~14]
	 *@param  [in] user  Workpiece coordinate number, range [0~14]
	 *@param  [in] vel  Percentage of speed, range [0~100]
	 *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
	 *@param  [in] epos  Position of expansion shaft, unit: mm
	 *@param  [in] ovl  Velocity scaling factor, range[0~100]
	 *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	 *@param  [in] offset_pos  The pose offset
	 *@param  [in] spiral_param  Spiral parameter
	 *@return  Error code
	 */
	errno_t  NewSpiral(JointPos* joint_pos, DescPose* desc_pos, int tool, int user, float vel, float acc, ExaxisPos* epos, float ovl, uint8_t offset_flag, DescPose* offset_pos, SpiralParam spiral_param);

	/**
     *@brief  Spiral motion in Cartesian space (Overloaded function does not require input of joint positions.)
     *@param  [in] desc_pos   Target Cartesian position
     *@param  [in] tool  Tool coordinate number, range [0~14]
     *@param  [in] user  Workpiece coordinate number, range [0~14]
     *@param  [in] vel  Percentage of speed, range [0~100]
     *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
     *@param  [in] epos  Position of expansion shaft, unit: mm
     *@param  [in] ovl  Velocity scaling factor, range[0~100]    
     *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
     *@param  [in] offset_pos  The pose offset
     *@param  [in] spiral_param  Spiral parameter
	 *@param  [in] config Robot inverse kinematics calculation joint configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration
     *@return  Error code
	 */
	errno_t  NewSpiral(DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param, int config = -1);	

	/**
	 * @brief Servo movement starts, used with ServoJ and ServoCart instructions
	 * @return  Error code
	 */
    errno_t ServoMoveStart();

	/**
	 * @brief Servo movement end, used with ServoJ and ServoCart instructions
	 * @return  Error code
	 */
    errno_t ServoMoveEnd();

	/**
    *@brief  Joint space servo mode motion
    *@param  [in] joint_pos  Target joint location, unit: deg
	*@param  [in] axisPos  External axis position, unit: mm
    *@param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    *@param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    *@param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    *@param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    *@param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
	*@param  [in] id servoJ command ID,default:0
    *@return  Error code
	*/
	errno_t ServoJ(JointPos* joint_pos, ExaxisPos* axisPos, float acc, float vel, float cmdT, float filterT, float gain, int id = 0);

	/**
    *@brief  Cartesian space servo mode motion
    *@param  [in]  mode  0- absolute motion (base coordinates), 1- incremental motion (base coordinates), 2- incremental motion (tool coordinates)
    *@param  [in]  desc_pos  Target Cartesian pose or pose increment
    *@param  [in]  pos_gain  Proportional coefficient of pose increment, effective only for incremental motion, range [0~1]
    *@param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    *@param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    *@param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    *@param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    *@param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
    *@return  Error code
	 */
	errno_t  ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);
   
	/**
    *@brief  Point to point motion in Cartesian space
    *@param  [in]  desc_pos  Target Cartesian pose or pose increment
    *@param  [in] tool  Tool coordinate number, range [0~14]
    *@param  [in] user  Workpiece coordinate number, range [0~14]
    *@param  [in] vel  Percentage of speed, range [0~100]
    *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] ovl  Velocity scaling factor, range[0~100]
    *@param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    *@param  [in] config  Joint space configuration, [-1]- refer to the current joint position, [0~7]- refer to the specific joint space configuration, the default is -1 
    *@return  Error code
	 */
	errno_t  MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);
	
	/**
    *@brief  The spline motion begins
    *@return  Error code
	 */
	errno_t  SplineStart();

    /**
     *@brief  Joint space spline movement
     *@param  [in] joint_pos  Target joint location, unit: deg
     *@param  [in] desc_pos   Target Cartesian position
     *@param  [in] tool  Tool coordinate number, range [0~14]
     *@param  [in] user  Workpiece coordinate number, range [0~14]
     *@param  [in] vel  Percentage of speed, range [0~100]
     *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
     *@param  [in] ovl  Velocity scaling factor, range[0~100]   
     *@return  Error code
	 */
	errno_t  SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl);
	
	/**
	 *@brief  Joint space spline movement(Overloaded functions do not require the input of Cartesian positions)
	 *@param  [in] joint_pos  Target joint location, unit: deg
	 *@param  [in] tool  Tool coordinate number, range [0~14]
	 *@param  [in] user  Workpiece coordinate number, range [0~14]
	 *@param  [in] vel  Percentage of speed, range [0~100]
	 *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
	 *@param  [in] ovl  Velocity scaling factor, range[0~100]
	 *@return  Error code
	 */
	errno_t  SplinePTP(JointPos* joint_pos, int tool, int user, float vel, float acc, float ovl);


	/**
    *@brief  The spline movement is complete
    *@return  Error code
	 */
	errno_t  SplineEnd();
	
	/**
	 *@brief New spline motion starts
	 *@param  [in] type   0-arc transition, 1-the given point is the path point
	 *@param  [in] averageTime global average connection time (ms) (10 ~ ), default 2000
	 *@return  Error code
	 */
	errno_t NewSplineStart(int type, int averageTime = 2000);
	
	/**
	 *@brief New Spline Cue Points
	 *@param  [in] joint_pos  Target joint position, unit: deg
	 *@param  [in] desc_pos   Target Cartesian pose
	 *@param  [in] tool  Tool coordinate number, range [0~14]
	 *@param  [in] user  Workpiece coordinate number, range [0~14]
	 *@param  [in] vel  Speed percentage, range [0~100]
	 *@param  [in] acc  Acceleration percentage, range [0~100], not open yet
	 *@param  [in] ovl  Speed scaling factor, range [0~100]
	 *@param  [in] blendR [-1.0]-movement in place (blocking), [0~1000.0]-smooth radius (non-blocking), unit: mm
	 *@param  [in] lastFlag [0,1] 0-the middle point of the spline curve, continue to execute the next point after execution, 1-the end point of the spline curve, decelerate and stop after execution
	 *@return  Error code
	 */	 
	errno_t  NewSplinePoint(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag);

	/**
	 *@brief New Spline Cue Points (Overloaded function does not require input of joint positions.)
	 *@param  [in] desc_pos   Target Cartesian pose
	 *@param  [in] tool  Tool coordinate number, range [0~14]
	 *@param  [in] user  Workpiece coordinate number, range [0~14]
	 *@param  [in] vel  Speed percentage, range [0~100]
	 *@param  [in] acc  Acceleration percentage, range [0~100], not open yet
	 *@param  [in] ovl  Speed scaling factor, range [0~100]
	 *@param  [in] blendR [-1.0]-movement in place (blocking), [0~1000.0]-smooth radius (non-blocking), unit: mm
	 *@param  [in] lastFlag [0,1] 0-the middle point of the spline curve, continue to execute the next point after execution, 1-the end point of the spline curve, decelerate and stop after execution
	 *@param  [in] config Robot inverse kinematics calculation joint configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration
	 *@return  Error code
	 */
	errno_t NewSplinePoint(DescPose* desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag, int config = -1);


	/**
	 *@brief New spline motion ends
	 *@return  Error code
	 */
	errno_t  NewSplineEnd();
	
	/**
	 *@brief Stop motion
	 *@return  Error code
	 */
	errno_t  StopMotion();

	/**
	 * @brief Pause motion
	 * @return  Error code
	 */
	errno_t  PauseMotion();	

	/**
	 * @brief Resume motion
	 * @return  Error code
	 */
	errno_t  ResumeMotion();	

	/**
	 * @brief  The whole point shift begins
	 * @param  [in]  flag 0- offset in base coordinate system/workpiece coordinate system, 2- offset in tool coordinate system
	 * @param  [in] offset_pos  The pose offset
	 * @return  Error code
	 */
	errno_t  PointsOffsetEnable(int flag, DescPose *offset_pos);
	
	/**
    *@brief  The whole point shift ends
    *@return  Error code
	 */
	errno_t  PointsOffsetDisable();
	
	/**
    *@brief  Set the control box digital output
    *@param  [in] id  I/O number and range[0~15]
    *@param  [in] status 0- off, 1- on
    *@param  [in] smooth 0- Not smooth, 1- smooth
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
    *@brief  Set tool digital output
    *@param  [in] id  I/O number and range[0~1]
    *@param  [in] status 0- off, 1- on
    *@param  [in] smooth 0- not smooth, 1- smooth
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
    *@brief  Set control box analog output
    *@param  [in] id  I/O number and range[0~1]
    *@param  [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetAO(int id, float value, uint8_t block);

	/**
    *@brief  Set tool analog output
    *@param  [in] id  I/O number, range [0]
    *@param  [in] value Percentage of current or voltage value, range [0~100] corresponding to voltage [0~10V]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetToolAO(int id, float value, uint8_t block);

	/**
    *@brief  Get the control box digital input
    *@param  [in] id  I/O number range[0~15]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  0- low, 1- high
    *@return  Error code
	 */	
	errno_t  GetDI(int id, uint8_t block, uint8_t *result);

	/**
    *@brief  Get tool numeric input
    *@param  [in] id  I/O number, range[0~1]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  0- low, 1- high
    *@return  Error code
	 */	
	errno_t  GetToolDI(int id, uint8_t block, uint8_t *result);
	
	/**
    *@brief Wait for the control box digital input
    *@param  [in] id  I/O number，range[0~15]
    *@param  [in]  status 0- off, 1- on
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitDI(int id, uint8_t status, int max_time, int opt);

	/**
    *@brief Wait for control box multiplex digital input
    *@param  [in] mode 0- multiplexed and, 1- multiplexed or
    *@param  [in] id  I/O numbers. bit0 to bit7 corresponds to DI0 to DI7, and bit8 to bit15 corresponds to CI0 to CI7
    *@param  [in]  status 0- off, 1- on
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt);

	/**
    *@brief Wait for the tool number to enter
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in]  status 0- off, 1- on
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitToolDI(int id, uint8_t status, int max_time, int opt);
	
	/**
    *@brief  Get control box analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    *@return  Error code
	 */	
	errno_t  GetAI(int id, uint8_t block, float *result);	

	/**
    *@brief  Get the tool analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to voltage [0-10V]
    *@return  Error code
	 */	
	errno_t  GetToolAI(int id, uint8_t block, float *result);	

	/**
	 * @brief Get the robot end point record button status
	 * @param [out] state button state, 0-pressed, 1-released
	 * @return Error code
	 */
    errno_t  GetAxlePointRecordBtnState(uint8_t *state);

	/**
	 * @brief Get the DO output status at the end of the robot
	 * @param [out] do_state DO output state, do0~do1 corresponds to bit1~bit2, starting from bit0
	 * @return Error code
	 */
    errno_t  GetToolDO(uint8_t *do_state);

	/**
	 * @brief Get the DO output status of the robot controller
	 * @param [out] do_state_h DO output status, co0~co7 corresponds to bit0~bit7
	 * @param [out] do_state_l DO output status, do0~do7 correspond to bit0~bit7
	 * @return Error code
	 */
    errno_t  GetDO(uint8_t *do_state_h, uint8_t *do_state_l);
	
	/**
    *@brief Wait for control box analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in]  sign 0-greater than，1-less than
    *@param  [in]  value Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    *@param  [in]  max_time Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitAI(int id, int sign, float value, int max_time, int opt);	
	
	/**
    *@brief Wait for tool analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in]  sign 0-greater than，1-less than
    *@param  [in]  value Percentage of input current or voltage value, range [0-100] corresponding to voltage [0-10V]
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitToolAI(int id, int sign, float value, int max_time, int opt);	

    /**
    *@brief  Set global speed
    *@param  [in]  vel  Percentage of velocity, range[0~100]
    *@return  Error code
	 */
	errno_t  SetSpeed(int vel);
	
	/**
    *@brief  Set the value of a system variable
    *@param  [in]  id  Variable number, range[1~20]
    *@param  [in]  value Variable value
    *@return  Error code
	 */
	errno_t  SetSysVarValue(int id, float value);

	/**
	 * @brief Setting tool reference points - six-point method
	 * @param [in] point_num point number, range [1~6]
	 * @return Error code
	 */
	errno_t SetToolPoint(int point_num);

	/**
	 * @brief  Calculation tool coordinate system
	 * @param [out] tcp_pose tool coordinate system
	 * @return Error code
	 */	
    errno_t ComputeTool(DescPose *tcp_pose);

	/**
	 * @brief Setting tool reference points - four-point method
	 * @param [in] point_num point number, range [1~4]
	 * @return Error code
	 */
	errno_t SetTcp4RefPoint(int point_num);

	/**
	 * @brief  Calculation tool coordinate system
	 * @param [out] tcp_pose tool coordinate system
	 * @return Error code
	 */
	errno_t ComputeTcp4(DescPose *tcp_pose);


	/**
	 * @brief  Set tool coordinate system
	 * @param  [in] id Frame number, range[0~14]
	 * @param  [in] coord  Tool center position relative to end flange center position
	 * @param  [in] type  0- tool coordinates, 1- sensor coordinates
	 * @param  [in] install Installation position, 0- robot end, 1- robot outside
	 * @param  [in] toolID tool ID
	 * @param  [in] loadNum loadNum
	 * @return  Error code
	 */
	errno_t  SetToolCoord(int id, DescPose* coord, int type, int install, int toolID, int loadNum);
	
	/**
    *@brief  Set the tool coordinate list
    *@param  [in] id Frame number, range[0~14]
    *@param  [in] coord  Tool center position relative to end flange center position
    *@param  [in] type  0- tool coordinates, 1- sensor coordinates
    *@param  [in] install Installation position, 0- robot end, 1- robot outside
	*@param  [in] loadNum Load number
    *@return  Error code
	 */
	errno_t  SetToolList(int id, DescPose* coord, int type, int install, int loadNum);


	/**
	 * @brief Setting External Tool Reference Points - Four-Point Method
	 * @param [in] point_num point number, range [1~4]
	 * @return Error code
	 */
	errno_t SetExTCPPoint(int point_num);

	/**
	 * @brief  Calculate external tool coordinate system
	 * @param [out] tcp_pose External tool coordinate system
	 * @return Error code
	 */	
	errno_t ComputeExTCF(DescPose *tcp_pose);

	/**
	 * @brief  Set the external tool coordinate system
	 * @param  [in] id Frame number, range[0~14]
	 * @param  [in] etcp  Tool center position relative to end flange center position
	 * @param  [in] etool  To be determined
	 * @return  Error code
	 */
	errno_t  SetExToolCoord(int id, DescPose *etcp, DescPose *etool);
	
	/**
    *@brief  Set the list of external tool coordinate systems
    *@param  [in] id Frame number, range[0~14]
    *@param  [in] etcp  Tool center position relative to end flange center position
    *@param  [in] etool  To be determined
    *@return  Error code
	 */
	errno_t  SetExToolList(int id, DescPose *etcp, DescPose *etool);	

	/**
	 * @brief Set the workpiece reference point - three-point method
	 * @param [in] point_num point number, range [1~3]
	 * @return Error code
	 */
	errno_t SetWObjCoordPoint(int point_num);

	/**
	 * @brief  Calculate workpiece coordinate system
	 * @param [in] Calculation method 0: origin-x axis-z axis 1: origin-x axis-xy plane
	 * @param [in] refFrame ref frame num
	 * @param [out] wobj_pose Workpiece coordinate system
	 * @return Error code
	 */	
	errno_t ComputeWObjCoord(int method, int refFrame, DescPose* wobj_pose);

	/**
	 * @brief  Set workpiece coordinate system
	 * @param  [in] id Coordinate system number, range [0~14]
	 * @param  [in] coord  The workpiece coordinate system relative to the end flange center pose
	 * @param  [in] refFrame Reference coordinate system
	 * @return  Error code
     */	 
	errno_t  SetWObjCoord(int id, DescPose* coord, int refFrame);
	
	/**
    *@brief  Set the list of work coordinate systems
    *@param  [in] id Frame number, range[0~14]
    *@param  [in] coord  Tool center position relative to end flange center position
	*@param  [in] refFrame Reference coordinate system
    *@return  Error code
     */	 
	errno_t  SetWObjList(int id, DescPose* coord, int refFrame);
	
	/**
    *@brief  Set the end load weight
	*@param  [in] loadNum load Num
    *@param  [in] weight  Load weight, unit: kg
    *@return  Error code
	 */
	errno_t  SetLoadWeight(int loadNum, float weight);
	
	/**
    *@brief  Set the end-load centroid coordinates
    *@param  [in] coord Centroid coordinates, unit: mm
    *@return  Error code
	 */
	errno_t  SetLoadCoord(DescTran *coord);

	/**
	 * @brief  Set the end-load centroid coordinates
	 * @param  [in] loadNum load Num
	 * @param  [in] coord Centroid coordinates, unit: mm
	 * @return  Error code
	 */
	errno_t SetLoadCoord(int loadNum, DescTran* coord);

	/**
    *@brief  Set the robot installation mode
    *@param  [in] install  Installation mode: 0- formal installation, 1- side installation, 2- inverted installation
    *@return  Error code
	 */
	errno_t  SetRobotInstallPos(uint8_t install);	

	/**
    *@brief  Set the robot installation Angle, free installation
    *@param  [in] yangle  Angle of inclination
    *@param  [in] zangle  Angle of rotation
    *@return  Error code
	 */
	errno_t  SetRobotInstallAngle(double yangle, double zangle);	

	/**
    *@brief  Wait for the specified time
    *@param  [in]  t_ms  unit: ms
    *@return  Error code
	 */
	errno_t  WaitMs(int t_ms);
	
	/**
    *@brief Set collision level
    *@param  [in]  mode  0- grade, 1- percentage
    *@param  [in]  level Collision threshold, grade range [1-10], percentage range [0~1]
    *@param  [in]  config 0- Do not update the configuration file. 1- Update the configuration file
    *@return  Error code
	 */
	errno_t  SetAnticollision(int mode, float level[6], int config);
	
	/**
	* @brief  Set the post-collision policy
	* @param  [in] strategy  0- Error stop, 1- Continue running
	* @param  [in] safeTime  Safe stop time[1000 - 2000]ms
	* @param  [in] safeDistance  Safe stopping distance[1-150]mm
	* @param  [in] safeVel safety velocity[50-250] mm/s
	* @param  [in] safetyMargin  j1-j6 Safety factor[1-10]
	* @return  Error code
	 */
	errno_t  SetCollisionStrategy(int strategy, int safeTime, int safeDistance, int safeVel, int safetyMargin[]);
	
	/**
    *@brief  Set the positive limit
    *@param  [in] limit Six joint positions, unit: deg
    *@return  Error code
	 */
	errno_t  SetLimitPositive(float limit[6]);
	
	/**
    *@brief  Set the negative limit
    *@param  [in] limit Six joint positions, unit: deg
    *@return  Error code
	 */
	errno_t  SetLimitNegative(float limit[6]);	
	
	/**
    *@brief  Error status clearing
    *@return  Error code
	 */
	errno_t  ResetAllError();
	
	/**
    *@brief  Joint friction compensation switch
    *@param  [in]  state  0- off, 1- on
    *@return  Error code
	 */
	errno_t  FrictionCompensationOnOff(uint8_t state);
	
	/**
    *@brief  Set joint friction compensation coefficient - formal
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_level(float coeff[6]);

	/**
    *@brief  Set joint friction compensation coefficient - side mount
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_wall(float coeff[6]);

	/**
    *@brief  Set joint friction compensation coefficient - inverted
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_ceiling(float coeff[6]);

	/**
    *@brief  Set joint friction compensation coefficient - free mount
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_freedom(float coeff[6]);
	
	/**
    *@brief  Obtain robot mounting Angle
    *@param  [out] yangle Angle of inclination
    *@param  [out] zangle Angle of rotation
    *@return  Error code
	 */
	errno_t  GetRobotInstallAngle(float *yangle, float *zangle);
	
	/**
    *@brief  Get the system variable value
    *@param  [in] id System variable number, range[1~20]
    *@param  [out] value  System variable value
    *@return  Error code
	 */
	errno_t  GetSysVarValue(int id, float *value);
	
	/**
    *@brief  Get the current joint position (Angle)
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] jPos Six joint positions, unit: deg
    *@return  Error code
	 */
	errno_t  GetActualJointPosDegree(uint8_t flag, JointPos *jPos);

	/**
	 * @brief  Get joint feedback velocity-deg/s
	 * @param  [in] flag 0 - blocking, 1 - non-blocking
	 * @param  [out] speed [j1,j2,j3,j4,j5,j6] speed
	 * @return  Error code 
	 */	
	errno_t  GetActualJointSpeedsDegree(uint8_t flag, float speed[6]);


	/**
	 * @brief  Get joint feedback acceleration-deg/s^2
	 * @param  [in] flag 0-blocking, 1-non-blocking
	 * @param  [out] acc speed Six joint acceleration
	 * @return  Error code 
	 */ 	
    errno_t  GetActualJointAccDegree(uint8_t flag, float acc[6]);    

	/**
	 *@brief  Get TCP instruction speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] tcp_speed linear speed
	 *@param  [out] ori_speed attitude speed
	 *@return  Error code 
	 */
	errno_t  GetTargetTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

	/**
	 *@brief  Get TCP instruction speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] tcp_speed linear speed
	 *@param  [out] ori_speed attitude speed
	 *@return  Error code 
	 */	
	errno_t  GetActualTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

	/**
	 *@brief  Get TCP instruction speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] speed [x,y,z,rx,ry,rz] speed
	 *@return  Error code  
	 */	
	errno_t  GetTargetTCPSpeed(uint8_t flag, float speed[6]);

	/**
	 *@brief  Get TCP Feedback Speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] speed [x,y,z,rx,ry,rz] speed
	 *@return  Error code  
	 */	
	errno_t  GetActualTCPSpeed(uint8_t flag, float speed[6]);
	
	/**
    *@brief  Get the current tool pose
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] desc_pos  Tool position
    *@return  Error code
	 */
	errno_t  GetActualTCPPose(uint8_t flag, DescPose *desc_pos);
	
	/**
    *@brief  Get the current tool coordinate system number
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] id  Tool coordinate system number
    *@return  Error code
	 */
	errno_t  GetActualTCPNum(uint8_t flag, int *id);
	
	/**
    *@brief  Get the current workpiece coordinate system number
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] id  Job coordinate system number
    *@return  Error code
	 */
	errno_t  GetActualWObjNum(uint8_t flag, int *id);	
	
	/**
    *@brief  Get the current end flange pose
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] desc_pos  Flange pose
    *@return  Error code
	 */
	errno_t  GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos);	
	
	/**
    *@brief  Inverse kinematics solution
    *@param  [in] type 0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    *@param  [in] desc_pos Cartesian pose
    *@param  [in] config Joint space configuration, [-1]- based on the current joint position, [0~7]- based on the specific joint space configuration
    *@param  [out] joint_pos Joint position
    *@return  Error code
	 */
	errno_t  GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos);

	/**
    *@brief  Inverse kinematics is solved by referring to the specified joint position
    *@param  [in] desc_pos Cartesian pose
	*@param  [in] type 0-absolute pose (base coordinate system), 1-incremental pose (base coordinate system), 2-incremental pose (tool coordinate system)
    *@param  [in] joint_pos_ref Reference joint position
    *@param  [out] joint_pos Joint position
    *@return  Error code
	 */	
	errno_t GetInverseKinRef(int type, DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos);

	/**
	 *@brief  Inverse kinematics is solved by referring to the specified joint position
    *@param  [in] desc_pos Cartesian pose
    *@param  [in] joint_pos_ref Reference joint position
    *@param  [out] result 0- no solution, 1-solution
    *@return  Error code
	 */	
	errno_t  GetInverseKinHasSolution(int type, DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result);

    /**
    *@brief  Forward kinematics solution
    *@param  [in] joint_pos Joint position
    *@param  [out] desc_pos Cartesian pose
    *@return  Error code
	 */
    errno_t  GetForwardKin(JointPos *joint_pos, DescPose *desc_pos);
	
	/**
    *@brief Obtain the current joint torque
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] torques Joint torque
    *@return  Error code
	 */
	errno_t GetJointTorques(uint8_t flag, float torques[6]);
	
	/**
    *@brief  Gets the weight of the current load
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] weight Load weight, unit: kg
    *@return  Error code
	 */
	errno_t  GetTargetPayload(uint8_t flag, float *weight);
	
	/**
    *@brief  Get the center of mass of the current load
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] cog Load center of mass, unit: mm
    *@return  Error code
	 */	
	errno_t  GetTargetPayloadCog(uint8_t flag, DescTran *cog);
	
	/**
    *@brief  Get the current tool coordinate system
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] desc_pos Tool coordinate position
    *@return  Error code
	 */
	errno_t  GetTCPOffset(uint8_t flag, DescPose *desc_pos);
	
	/**
    *@brief  Get the current work frame
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] desc_pos Position of workpiece coordinate system
    *@return  Error code
	 */	
	errno_t  GetWObjOffset(uint8_t flag, DescPose *desc_pos);
	
	/**
    *@brief  Obtain joint soft limit Angle
    *@param  [in] flag 0- blocking, 1- non-blocking    
    *@param  [out] negative  Negative limit Angle, unit: deg
    *@param  [out] positive  Positive limit Angle, unit: deg
    *@return  Error code
	 */
	errno_t  GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6]);
	
	/**
    *@brief  Get system time
    *@param  [out] t_ms unit: ms
    *@return  Error code
	 */
	errno_t  GetSystemClock(float *t_ms);
	
	/**
    *@brief  Get the current joint configuration of the robot
    *@param  [out]  config  Joint space configuration, range [0~7]
    *@return  Error code
	 */
	errno_t  GetRobotCurJointsConfig(int *config);

	/**
    *@brief  Get the robot's current speed
    *@param  [out]  vel  The unit is mm/s
    *@return  Error code
	 */	
	errno_t  GetDefaultTransVel(float *vel);

	/**
    *@brief  Query whether the robot movement is complete
    *@param  [out]  state  0- Incomplete, 1- completed
    *@return  Error code
	 */	
	errno_t  GetRobotMotionDone(uint8_t *state);

	/**
	 * @brief  Query robot error code
	 * @param  [out]  maincode  main error code 
	 * @param  [out]  subcode   sub main code
	 * @return  error code
	 */ 
	errno_t  GetRobotErrorCode(int *maincode, int *subcode);

	/**
	 * @brief  Query robot teaching and management point data
	 * @param  [in]  name  Point name
	 * @param  [out]  data   point data
	 * @return  error code
	 */ 
	errno_t  GetRobotTeachingPoint(char name[64], float data[20]);


	/**
	 * @brief  Query the robot motion queue cache length
	 * @param  [out]  len  cache length
	 * @return  error code
	 */ 	
	errno_t  GetMotionQueueLength(int *len);

	
	/**
	 * @brief  Set track recording parameters
	 * @param  [in] type  Record data type, 1- joint position
	 * @param  [in] name  Track file name
	 * @param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
	 * @param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
	 * @param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
	 * @return  Error code
	 */
	errno_t  SetTPDParam(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);
	
	/**
    *@brief  Start track recording
    *@param  [in] type  Record data type, 1- joint position
    *@param  [in] name  Track file name
    *@param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    *@param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
    *@param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
    *@return  Error code
	 */
	errno_t  SetTPDStart(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);	
	
	/**
    *@brief  Stop track recording
    *@return  Error code
	 */
	errno_t  SetWebTPDStop();
	
	/**
    *@brief  Delete track record
    *@param  [in] name  Track file name
    *@return  Error code
	 */	
	errno_t  SetTPDDelete(char name[30]);
	
	/**
    *@brief  Trajectory preloading
    *@param  [in] name  Track file name
    *@return  Error code
	 */		
	errno_t  LoadTPD(char name[30]);

	/**
	 * @brief  Get the starting pose of the trajectory
	 * @param  [in] name trajectory file name, file suffix is not required
	 * @return  error code
	 */		
    errno_t  GetTPDStartPose(char name[30], DescPose *desc_pose);    
	
	/**
    *@brief  Trajectory recurrence
    *@param  [in] name  Track file name
    *@param  [in] blend 0- not smooth, 1- smooth
    *@param  [in] ovl  Speed scaling percentage, range [0~100]
    *@return  Error code
	 */
	errno_t  MoveTPD(char name[30], uint8_t blend, float ovl);

	/**
	 * @brief  trajectory preprocessing
	 * @param  [in] name  trajectory file name
	 * @param  [in] ovl speed scaling percentage, range [0~100]
	 * @param  [in] opt 1-control point, default is 1
	 * @return  error code 
	 */     	
	errno_t  LoadTrajectoryJ(char name[30], float ovl, int opt);

	/**
	 * @brief  trajectory recurrence
	 * @return  error code 
	 */     	
	errno_t  MoveTrajectoryJ();

	/**
	 * @brief  Get the starting pose of the trajectory
	 * @param  [in] name trajectory file name
	 * @return  error code
	 */     	
    errno_t  GetTrajectoryStartPose(char name[30], DescPose *desc_pose);

	/**
	 * @brief  Get the track point number
	 * @return  error code
	 */     	
	errno_t  GetTrajectoryPointNum(int *pnum);

	/**
	 * @brief  Set the speed during trajectory running
	 * @param  [in] ovl speed percentage
	 * @return  error code 
	 */     	
	errno_t  SetTrajectoryJSpeed(float ovl);

	/**
	 * @brief  Set the force and torque during trajectory operation
	 * @param  [in] ft Force and torque in three directions, units N and Nm
	 * @return  error code
	 */     	
	errno_t  SetTrajectoryJForceTorque(ForceTorque *ft);

	/**
	 * @brief  Set the force along the x direction during trajectory operation
	 * @param  [in] fx Force along the x direction, unit N
	 * @return  error code  
	 */     	
	errno_t  SetTrajectoryJForceFx(double fx);

	/**
	 * @brief  Set the force along the y direction during trajectory operation
	 * @param  [in] fy Force along the y direction, unit N
	 * @return  error code  
	 */     	
	errno_t  SetTrajectoryJForceFy(double fy);

	/**
	 * @brief  Set the force along the z direction during trajectory operation
	 * @param  [in] fz Force along the z direction, unit N
	 * @return  error code 
	 */     	
	errno_t  SetTrajectoryJForceFz(double fz);

	/**
	 * @brief  Set the torque around the x-axis during trajectory operation
	 * @param  [in] tx Torque around the x-axis, unit Nm
	 * @return  error code
	 */     	
	errno_t  SetTrajectoryJTorqueTx(double tx);

	/**
	 * @brief  Set the torque around the y-axis during trajectory operation
	 * @param  [in] ty Torque around the y-axis, unit Nm
	 * @return  error code
	 */     	
	errno_t  SetTrajectoryJTorqueTy(double ty);

	/**
	 * @brief  Set the torque around the z-axis during trajectory operation
	 * @param  [in] tz Torque around the z-axis, unit Nm
	 * @return  error code
	 */     	
	errno_t  SetTrajectoryJTorqueTz(double tz);

	/**
	 * @brief  Set the default job program to be automatically loaded upon startup
	 * @param  [in] flag  0- boot does not automatically load the default program, 1- boot automatically load the default program
	 * @param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
	 * @return  Error code
	 */
	errno_t  LoadDefaultProgConfig(uint8_t flag, char program_name[64]);
	
	/**
    *@brief  Load the specified job program
    *@param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    *@return  Error code
	 */
	errno_t  ProgramLoad(char program_name[64]);
	
	/**
    *@brief  Get the loaded job program name
    *@param  [out] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    *@return  Error code
	 */
	errno_t  GetLoadedProgram(char program_name[64]);	
	
	/**
    *@brief  Get the line number of the current robot job program
    *@param  [out] line  line number
    *@return  Error code
	 */	
	errno_t  GetCurrentLine(int *line);
	
	/**
    *@brief  Run the currently loaded job program
    *@return  Error code
	 */
	errno_t  ProgramRun();
	
	/**
    *@brief  Pause the current running job program
    *@return  Error code
	 */ 
	errno_t  ProgramPause();
	
	/**
    *@brief  Resume the currently suspended job program
    *@return  Error code
	 */ 
	errno_t  ProgramResume();	
	
	/**
    *@brief  Terminates the currently running job program
    *@return  Error code
	 */ 
	errno_t  ProgramStop();		
	
	/**
    *@brief  Get the robot job program execution state
    *@param  [out]  state 1- program stop or no program running, 2- program running, 3- program pause
    *@return  Error code
	 */
	errno_t  GetProgramState(uint8_t *state);
	
	/**
    *@brief  Configure the gripper
    *@param  [in] company  Claw manufacturer, to be determined
    *@param  [in] device  Device number, not used yet. The default value is 0
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  SetGripperConfig(int company, int device, int softvesion, int bus);
	
	/**
    *@brief  Configure the gripper
    *@param  [in] company  Claw manufacturer, to be determined
    *@param  [in] device  Device number, not used yet. The default value is 0
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

	/**
    *@brief  Activate Activate gripper
    *@param  [in] index  gripper gripper
    *@param  [in] act  0- reset, 1- activate
    *@return  Error code
	 */
	errno_t  ActGripper(int index, uint8_t act);

	/**
    * @brief  Control gripper
    * @param  [in] index  gripper number
    * @param  [in] pos  Percentage of position, range[0~100]
    * @param  [in] vel  Percentage of velocity, range[0~100]
    * @param  [in] force  Percentage of torque, range[0~100]
    * @param  [in] max_time  Maximum wait time, range[0~30000], unit: ms
    * @param  [in] block  0- blocking, 1- non-blocking
	* @param  [in] type grippr type, 0-parallel clamp; 1- Rotate the claw
	* @param  [in] rotNum Number of turns[0-100]
	* @param  [in] rotVel Percent rotation speed[100]
	* @param  [in] rotTorque Percentage of rotating torque [0-100]
    * @return  Error code
	 */
	errno_t  MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block, int type, double rotNum, int rotVel, int rotTorque);

	/**
	 * @brief  Get the gripper motion status
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] staus 0-the movement is not completed, 1-the movement is completed
	 * @return  Error code 
	 */
    errno_t  GetGripperMotionDone(uint16_t *fault, uint8_t *status);

	/**
	 * @brief  Get the gripper activation status
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] status bit0~bit15 corresponds to the gripper number 0~15, bit=0 is inactive, bit=1 is activated
	 * @return  Error code
	 */    
	errno_t  GetGripperActivateStatus(uint16_t *fault, uint16_t *status);

	/**
	 * @brief  Get the gripper position
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] position position percentage, range 0~100%
	 * @return  Error code 
	 */    
	errno_t  GetGripperCurPosition(uint16_t *fault, uint8_t *position);

	/**
	 * @brief  Get the gripper speed
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] speed speed percentage, range 0~100%
	 * @return  Error code 
	 */
    errno_t  GetGripperCurSpeed(uint16_t *fault, int8_t *speed);

	/**
	 * @brief  Get the gripper current
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] current current percentage, range 0~100%
	 * @return  Error code 
	 */
    errno_t  GetGripperCurCurrent(uint16_t *fault, int8_t *current);

	/**
	 * @brief  Get the gripper voltage
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] voltage voltage, unit 0.1V
	 * @return  Error code 
	 */
    errno_t  GetGripperVoltage(uint16_t *fault, int *voltage);

	/**
	 * @brief  Get the gripper temperature
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] temp temperature, unit °C
	 * @return  Error code 
	 */    
	errno_t  GetGripperTemp(uint16_t *fault, int *temp);

	/**
	 * @brief  Gets the number of turns of the rotary gripper
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] num  Number of turns
	 * @return  Error code
	 */
	errno_t GetGripperRotNum(uint16_t* fault, double* num);

	/**
	 * @brief  Gets the rotation speed of the rotating gripper
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] speed  Percent rotation speed
	 * @return  Error code
	 */
	errno_t GetGripperRotSpeed(uint16_t* fault, int* speed);

	/**
	 * @brief  Obtain the rotating torque of the rotating gripper
	 * @param  [out] fault 0-no error, 1-error
	 * @param  [out] torque  Percent torque of rotation
	 * @return  Error code
	 */
	errno_t GetGripperRotTorque(uint16_t* fault, int* torque);

	/**
	 *@brief  Computing Prefetch Points - Vision
	 *@param  [in] desc_pos  Grab point Cartesian pose
	 *@param  [in] zlength   z-axis offset
	 *@param  [in] zangle    Rotate the offset around the z-axis
	 *@return  Error code 
	 */
	errno_t  ComputePrePick(DescPose *desc_pos, double zlength, double zangle, DescPose *pre_pos);

	/**
	 *@brief  Calculating Retreat Points - Visual
	 *@param  [in] desc_pos  Grab point Cartesian pose
	 *@param  [in] zlength   z axis offset
	 *@param  [in] zangle    Rotate the offset around the z-axis
	 *@return  Error code 
	 */
	errno_t  ComputePostPick(DescPose *desc_pos, double zlength, double zangle, DescPose *post_pos);

	/**
    *@brief  Configured force sensor
    *@param  [in] company  Manufacturer of force sensors, 17-KUNWEI，19-CAAA，20-ATI，21-HKM，22-GZCX，23-NBIT，24-XJC，26-NSR
    *@param  [in] device  Device number,  KUNWEI(0-KWR75B)，CAAA(0-MCS6A-200-4)，ATI(0-AXIA80-M8)，HKM(0-MST2010)，GZCX(0-WHC6L-YB-10A)，NBIT(0-XLH93003ACS)，XJC(0-XJC-6F-D82)，NSR(0-NSR-FTSensorA)
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  FT_SetConfig(int company, int device, int softvesion, int bus);

	/**
    *@brief  Get the force sensor configuration
    *@param  [in] company  Force sensor manufacturer, to be determined
    *@param  [in] device  Device number, not used yet. The default value is 0
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  FT_GetConfig(int *company, int *device, int *softvesion, int *bus);

	/**
    *@brief  Force sensor activation
    *@param  [in] act  0- reset, 1- activate
    *@return  Error code
	 */
	errno_t  FT_Activate(uint8_t act);
	
	/**
    *@brief  Force sensor calibration
    *@param  [in] act  0- zero removal, 1- zero correction
    *@return  Error code
	 */
	errno_t  FT_SetZero(uint8_t act);	

	/**
    *@brief  Set the reference coordinate system of the force sensor
    *@param  [in] ref  0- tool frame, 1- base frame
    *@return  Error code
	 */
	errno_t FT_SetRCS(uint8_t ref, DescPose coord);
	
	/**
    *@brief  Load weight identification record
    *@param  [in] id  Sensor coordinate system number, range [1~14]
    *@return  Error code
	 */
	errno_t  FT_PdIdenRecord(int id);	
	
	/**
    *@brief  Load weight identification calculation
    *@param  [out] weight  Load weight, unit: kg
    *@return  Error code
	 */	
	errno_t  FT_PdIdenCompute(float *weight);
	
	/**
    *@brief  Load centroid identification record
    *@param  [in] id  Sensor coordinate system number, range [1~14]
    *@param  [in] index Point number, range [1~3]
    *@return  Error code
	 */
	errno_t  FT_PdCogIdenRecord(int id, int index);		
	
	/**
    *@brief  Load centroid identification calculation
    *@param  [out] cog  Load center of mass, unit: mm
    *@return  Error code
	 */	
	errno_t  FT_PdCogIdenCompute(DescTran *cog);	

	/**
	 *@brief  Obtain force/torque data in the reference coordinate system
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] ft  Force/torque，fx,fy,fz,tx,ty,tz
	 *@return  Error code
	 */	
	errno_t  FT_GetForceTorqueRCS(uint8_t flag, ForceTorque *ft);	

	/**
	 *@brief  Obtain the raw force/torque data of the force sensor
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
    *@param  [out] ft  Force/torque，fx,fy,fz,tx,ty,tz
    *@return  Error code
	 */	
	errno_t  FT_GetForceTorqueOrigin(uint8_t flag, ForceTorque *ft);	

	/**
    *@brief  Collision guard
    *@param  [in] flag 0- Disable collision guard. 1- Enable collision guard
    *@param  [in] sensor_id Force sensor number
    *@param  [in] select  Select the six degrees of freedom whether to detect collision, 0- no detection, 1- detection
    *@param  [in] ft  Impact force/torque，fx,fy,fz,tx,ty,tz
    *@param  [in] max_threshold Maximum threshold
    *@param  [in] min_threshold Minimum threshold
    *@note   Force/torque detection range：(ft-min_threshold, ft+max_threshold)
    *@return  Error code
	 */	
	errno_t  FT_Guard(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float max_threshold[6], float min_threshold[6]);	
	
	/**
    * @brief  Constant force control
    * @param  [in] flag 0- turn off constant force control, 1- turn on constant force control
    * @param  [in] sensor_id Force sensor number
    * @param  [in] select  Select the six degrees of freedom whether to detect collision, 0- no detection, 1- detection
    * @param  [in] ft  Impact force/torque，fx,fy,fz,tx,ty,tz
    * @param  [in] ft_pid Force pid parameter, torque pid parameter
    * @param  [in] adj_sign Adaptive start-stop control, 0- off, 1- on
    * @param  [in] ILC_sign ILC start stop control, 0- stop, 1- training, 2- operation
    * @param  [in] max_dis Adjustment distance, unit: mm
    * @param  [in] max_ang Adjustment Angle, unit: deg
	* @param  [in] filter_Sign Filter on indicator 0- off; 1- On, off by default
	* @param  [in] posAdapt_sign The posture conforms to the opening mark 0-off. 1- On, off by default
	* @param  [in] isNoBlock Block flag, 0- block; 1- Non-blocking
    * @return  Error code
	*/	
	errno_t FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float ft_pid[6], uint8_t adj_sign, 
		uint8_t ILC_sign, float max_dis, float max_ang, int filter_Sign = 0, int posAdapt_sign = 0, int isNoBlock = 0);

	/**
	* @brief  Constant force control
	* @param  [in] flag 0- turn off constant force control, 1- turn on constant force control
	* @param  [in] sensor_id Force sensor number
	* @param  [in] select  Select the six degrees of freedom whether to detect collision, 0- no detection, 1- detection
	* @param  [in] ft  Impact force/torque，fx,fy,fz,tx,ty,tz
	* @param  [in] ft_pid Force pid parameter, torque pid parameter
	* @param  [in] adj_sign Adaptive start-stop control, 0- off, 1- on
	* @param  [in] ILC_sign ILC start stop control, 0- stop, 1- training, 2- operation
	* @param  [in] max_dis Adjustment distance, unit: mm
	* @param  [in] max_ang Adjustment Angle, unit: deg
	* @param  [in] M Quality parameters
	* @param  [in] B Damping parameter
	* @param  [in] polishRadio Polish radius, unit: mm
	* @param  [in] filter_Sign Filter on indicator 0- off; 1- On, off by default
	* @param  [in] posAdapt_sign The posture conforms to the opening mark 0-off. 1- On, off by default
	* @param  [in] isNoBlock Block flag, 0- block; 1- Non-blocking
	* @return  Error code
	*/
	errno_t FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque* ft, float ft_pid[6], uint8_t adj_sign, 
		uint8_t ILC_sign, float max_dis, float max_ang, double M[2], double B[2], double polishRadio = 0.0, int filter_Sign = 0, int posAdapt_sign = 0, int isNoBlock = 0);


	/**
    *@brief  Spiral exploration
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] dr Feed per circle radius
    *@param  [in] ft Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    *@param  [in] max_t_ms Maximum exploration time, unit: ms
    *@param  [in] max_vel Maximum linear velocity, unit: mm/s
    *@return  Error code
	 */	
    errno_t  FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel);	
	
	/**
    *@brief  Rotary insertion
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] angVelRot Angular velocity of rotation, unit: deg/s
    *@param  [in] ft  Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    *@param  [in] max_angle Maximum rotation Angle, unit: deg
    *@param  [in] orn Force/torque direction, 1- along the z axis, 2- around the z axis
    *@param  [in] max_angAcc Maximum rotational acceleration, in deg/s^2, not used yet, default is 0
    *@param  [in] rotorn  Rotation direction, 1- clockwise, 2- counterclockwise
    *@return  Error code
	 */	
    errno_t  FT_RotInsertion(int rcs, float angVelRot, float ft, float max_angle, uint8_t orn, float max_angAcc, uint8_t rotorn);		
	
	/**
    *@brief  Linear insertion
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] ft  Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    *@param  [in] lin_v Linear velocity, unit: mm/s
    *@param  [in] lin_a Linear acceleration, unit: mm/s^2, not used yet
    *@param  [in] max_dis Maximum insertion distance, unit: mm
    *@param  [in] linorn  Insert direction, 0- negative, 1- positive
    *@return  Error code
	 */	
    errno_t  FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, uint8_t linorn);		

	/**
    *@brief  Surface positioning
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] dir  The direction of travel, 1- positive, 2- negative
    *@param  [in] axis Axis of movement, 1-x axis, 2-y axis, 3-z axis
    *@param  [in] lin_v Explore the linear velocity in mm/s
    *@param  [in] lin_a Explore linear acceleration, in mm/s^2, not used yet, default to 0
    *@param  [in] max_dis Maximum exploration distance, in mm
    *@param  [in] ft  Action termination force/torque threshold，fx,fy,fz,tx,ty,tz  
    *@return  Error code
	 */	
    errno_t  FT_FindSurface(int rcs, uint8_t dir, uint8_t axis, float lin_v, float lin_a, float max_dis, float ft);	
	
	/**
    *@brief  Calculation of midplane position starts
    *@return  Error code
	 */	
	errno_t  FT_CalCenterStart();
	
	/**
    *@brief  Calculation of midplane position ends
    *@param  [out] pos Intermediate plane position
    *@return  Error code
	 */		
	errno_t  FT_CalCenterEnd(DescPose *pos);
	
	/**
    *@brief  Compliant control on
    *@param  [in] p Coefficient of position adjustment or compliance
    *@param  [in] force Compliant opening force threshold, unit: N
    *@return  Error code
	 */	
	errno_t  FT_ComplianceStart(float p, float force);	
	
	/**
    *@brief  Compliant control off
    *@return  Error code
	 */	
	errno_t  FT_ComplianceStop();	

	/**
	 * @brief Load identification initialization
	 * @return Error code 
	 */
	errno_t LoadIdentifyDynFilterInit();

	/**
	 * @brief Load identification initialization
	 * @return Error code 
	 */
	errno_t LoadIdentifyDynVarInit();

	/**
	 * @brief load identification main program
	 * @param [in] joint_torque joint torque
	 * @param [in] joint_pos joint position
	 * @param [in] t sampling period
	 * @return Error code 
	 */
	errno_t LoadIdentifyMain(double joint_torque[6], double joint_pos[6], double t);


	/**
	 * @brief Get load identification results
	 * @param [in] gain  
	 * @param [out] weight load weight
	 * @param [out] cog cog load center of mass
	 * @return Error code 
	 */
	errno_t LoadIdentifyGetResult(double gain[12], double *weight, DescTran *cog);

	/**
	 * @brief The transmission belt starts and stops
	 * @param [in] status status, 1-start, 0-stop
	 * @return Error code 
	 */
	errno_t ConveyorStartEnd(uint8_t status);

	/**
	 * @brief records IO detection points
	 * @return Error code 
	 */
	errno_t ConveyorPointIORecord();

	/**
	 * @brief Record point A
	 * @return Error code 
	 */
	errno_t ConveyorPointARecord();

	/**
	 * @brief record reference point
	 * @return Error code 
	 */
	errno_t ConveyorRefPointRecord();

	/**
	 * @brief Record point B
	 * @return Error code 
	 */
	errno_t ConveyorPointBRecord();

	/**
	 * @brief Conveyor belt workpiece IO detection
	 * @param [in] max_t maximum detection time, unit ms
	 * @return Error code 
	 */
	errno_t ConveyorIODetect(int max_t);

	/**
	 * @brief Get the current position of the object
	 * @param [in] mode 
	 * @return Error code 
	 */
	errno_t ConveyorGetTrackData(int mode);

	/**
	 * @brief Belt tracking starts
	 * @param [in] status status , 1-start, 0-stop
	 * @return Error code 
	 */
	errno_t ConveyorTrackStart(uint8_t status);

	/**
	 * @brief Drive belt tracking stopped
	 * @return Error code 
	 */
	errno_t ConveyorTrackEnd();

	/**
	 * @brief Drive belt Parameter Configuration
	 * @param [in] para[0] Encoder channels 1-2
	 * @param [in] para[1] The number of pulses for one revolution of the encoder
	 * @param [in] para[2] The distance traveled by the conveyor belt when the encoder rotates one full circle
	 * @param [in] para[3] Select the workpiece coordinate system number for the tracking motion function, and set the tracking grasping and TPD tracking to 0
	 * @param [in] para[4] Does it have vision? 0 does not have 1
	 * @param [in] para[5] Speed ratio for the conveyor belt tracking grasping option (1-100), the default for other options is 1
	 * @param [in] followType Tracking motion type, 0- tracking motion; 1- Follow-up inspection campaign
	 * @param [in] startDis Tracking and grasping need to be set. The starting distance for tracking is -1: automatic calculation (automatic tracking and grasping after the workpiece reaches under the robot), unit: mm, default value: 0
	 * @param [in] endDis tracking and grasping need to be set. The tracking termination distance, in mm, the default value is 100
	 * @return 错误码
	 */
	errno_t ConveyorSetParam(float para[6], int followType = 0, int startDis = 0, int endDis = 100);

	/**
	 * @brief belt grab point compensation
	 * @param [in] cmp compensation position
	 * @return Error code 
	 */
	errno_t ConveyorCatchPointComp(double cmp[3]);

	/**
	 * @brief linear motion
	 * @param [in] status status, 1-start, 0-stop
	 * @return Error code 
	 */
	errno_t TrackMoveL(char name[32], int tool, int wobj, float vel, float acc, float ovl, float blendR, uint8_t flag, uint8_t type);

	/**
	 * @brief Get SSH public key
	 * @param [out] keygen public key
	 * @return Error code 
	 */
	errno_t GetSSHKeygen(char keygen[1024]);

	/**
	 * @brief issues SCP instructions
	 * @param [in] mode 0-upload (host computer->controller), 1-download (controller->host computer)
	 * @param [in] sshname host computer user name
	 * @param [in] sship host computer ip address
	 * @param [in] usr_file_url host computer file path
	 * @param [in] robot_file_url robot controller file path
	 * @return Error code 
	 */
	errno_t SetSSHScpCmd(int mode, char sshname[32], char sship[32], char usr_file_url[128], char robot_file_url[128]);

	/**
	 * @brief Calculate the MD5 value of the file under the specified path
	 * @param [in] file_path The file path contains the file name. The default Traj folder path is: "/fruser/traj/", such as "/fruser/traj/trajHelix_aima_1.txt"
	 * @param [out] md5 file MD5 value
	 * @return Error code 
	 */
    errno_t ComputeFileMD5(char file_path[256], char md5[256]);

	/**
	 * @brief Get the emergency stop status of the robot
	 * @param [out] state emergency stop status, 0-non-emergency stop, 1-emergency stop
	 * @return Error code   
	 */
    errno_t GetRobotEmergencyStopState(uint8_t *state);

	/**
	 * @brief Get the communication status between SDK and robot
	 * @param [out]  state communication status, 0-communication is normal, 1-communication is abnormal
	 * @return Error code 
	 */
    errno_t GetSDKComState(int *state);


	/**
	 * @brief Get the safe stop signal
	 * @param [out]  si0_state safety stop signal SI0, 0-invalid, 1-valid
	 * @param [out]  si1_state safety stop signal SI1, 0-invalid, 1-valid
	 * @return Error code 
	 */
	errno_t GetSafetyStopState(uint8_t *si0_state, uint8_t *si1_state);

	/**
	 * @brief Get robot software version
	 * @param[out]	robotModel Robot model
	 * @param[out]	webversion web version
	 * @param[out]	controllerVersion controller version
	 * @return Error code 
	*/
	errno_t GetSoftwareVersion(char robotModel[64], char webVersion[64], char controllerVersion[64]);

	/**
	 * @brief Get the robot hardware version
	 * @param[out] ctrlBoxBoardversion Control box carrier board hardware version
	 * @param[out] driver1version Drive 1 Hardware Version
	 * @param[out] driver2version Drive 2 Hardware Version
	 * @param[out] driver3version Drive 3 Hardware Version
	 * @param[out] driver4version Drive 4 Hardware Version
	 * @param[out] driver5version Drive 5 Hardware Version
	 * @param[out] driver6version Drive 6 Hardware Version
	 * @param[out] endBoardversion End version hardware version
	 * @return Error code 
	*/
	errno_t GetHardwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128],
							char driver3version[128], char driver4version[128], char driver5version[128],
							char driver6version[128], char endBoardversion[128]);

	/**
	 * @brief Get the robot firmware version
	 * @param[out] ctrlBoxBoardversion Control box carrier board firmware version
	 * @param[out] driver1version Drive 1 firmware version
	 * @param[out] driver2version Drive 2 firmware version
	 * @param[out] driver3version Drive 3 firmware version
	 * @param[out] driver4version Drive 4 firmware version
	 * @param[out] driver5version Drive 5 firmware version
	 * @param[out] driver6version Drive 6 firmware version
	 * @param[out] endBoardversion End version firmware version
	 * @return Error code 
	 */
	errno_t GetFirmwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128],
							char driver3version[128], char driver4version[128], char driver5version[128],
							char driver6version[128], char endBoardversion[128]);

	/**
	 * @brief Obtain the compensation value of the DH parameter of the robot
	 * @param [out] dhCompensation Robot DH parameter compensation value (mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
	 * @return Error code 
	 */
	errno_t GetDHCompensation(double dhCompensation[6]);

	/**
	 * @brief 点位表切换
	 * @param [in] pointTableName 要切换的点位表名称    pointTable1.db
	 * @return error code
	 */
	errno_t PointTableSwitch(const std::string pointTableName);
	/**
	 * @brief Download the point table database
	 * @param [in] pointTableName The name of the point table to be downloaded   pointTable1.db
	 * @param [in] saveFilePath the storage path of the point table  C://test/
	 * @return Error code 
	 */
	errno_t PointTableDownLoad(const std::string &pointTableName, const std::string &saveFilePath);

	/**
	 * @brief Upload the point table database
	 * @param [in] pointTableFilePath the full pathname of the point table    C://test/pointTable1.db
	 * @return Error code
	 */
	errno_t PointTableUpLoad(const std::string &pointTableFilePath);

	/**
	 * @brief Update the LUA file for the point table
	 * @param [in] pointTableName The name of the point table to be switched  
	 * @param [in] luaFileName name of lua file to be updated   "testPointTable.lua"
	 * @return Error code
	 */
	errno_t PointTableUpdateLua(const std::string &pointTableName, const std::string &luaFileName);

	/**
	 * @brief Welding starts
	 * @param [in] IO Type 0 - Controller IO; 1 - Extended IO
	 * @param [in] arcNum welder profile number
	 * @param [in] timeout time limit
	 * @return Error code
	 */
	errno_t ARCStart(int ioType, int arcNum, int timeout);

	/**
	 * @brief Welding ended
	 * @param [in] ioType IO Type 0 - Controller IO; 1 - Extended IO
	 * @param [in] arcNum welder profile number
	 * @param [in] timeout arc extinguishing timeout
	 * @return Error code
	 */
	errno_t ARCEnd(int ioType, int arcNum, int timeout);

	/**
	 * @brief Set the corresponding relationship between welding current and output analog quantity
	 * @param [in] currentMin current value at the left point of the linear relationship between welding current and analog output (A)
	 * @param [in] currentMax current value at the right point of the linear relationship between welding current and analog output (A)
	 * @param [in] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding current and analog output
	 * @param [in] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding current and analog output
	 * @return Error code
	 */
	errno_t WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax, int AOIndex = 0);

	/**
	 * @brief Set the corresponding relationship between welding voltage and output analog quantity
	 * @param [in] weldVoltageMin eldVoltageMin Welding voltage value (A) at the left point of the linear relationship between welding voltage and analog output
	 * @param [in] weldVoltageMax Welding voltage-analog output linear relationship right point welding voltage value (A)
	 * @param [in] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding voltage and analog output
	 * @param [in] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding voltage and analog output
	 * @return Error code
	 */
	errno_t WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax, int AOIndex = 1);

	/**
	 * @brief Get the corresponding relationship between welding current and output analog quantity
	 * @param [out] currentMin current value at the left point of the linear relationship between welding current and analog output (A)
	 * @param [out] currentMax welding current and analog output (A)
	 * @param [out] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding current and analog output
	 * @param [out] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding current and analog output
	 * @return Error code
	 */
	errno_t WeldingGetCurrentRelation(double *currentMin, double *currentMax, double *outputVoltageMin, double *outputVoltageMax, int* AOIndex);

	/**
	 * @brief Get the corresponding relationship between welding voltage and output analog quantity
	 * @param [out] weldVoltageMin Welding voltage value (A) at the left point of the linear relationship between welding voltage and analog output
	 * @param [out] weldVoltageMax Welding voltage-analog output linear relationship right point welding voltage value (A)
	 * @param [out] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding voltage and analog output
	 * @param [out] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding voltage and analog output
	 * @return Error code
	 */
	errno_t WeldingGetVoltageRelation(double *weldVoltageMin, double *weldVoltageMax, double *outputVoltageMin, double *outputVoltageMax, int* AOIndex);

	/**
	 * @brief Set welding current
	 * @param [in] ioType 0-control box IO； 1-extend IO
	 * @param [in] current welding current(A)
	 * @param [in] AOIndexWelding current control box analog output port(0-1)
	 * @return Error code
	 */
	errno_t WeldingSetCurrent(int ioType, double current, int AOIndex, int blend);

	/**
	 * @brief Set welding voltage
	 * @param [in] ioType 0-control box IO； 1-extend IO
	 * @param [in] voltage welding voltage(V)
	 * @param [in] AOIndex Welding voltage control box analog output port(0-1)
	 * @return Error code
	 */
	errno_t WeldingSetVoltage(int ioType, double voltage, int AOIndex, int blend);

	/**
	 * @brief Set weave parameters
	 * @param [in] weaveNum parameters number
	 * @param [in] weaveType weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave;
	 * @param [in] weaveFrequency weave frequency(Hz)
	 * @param [in] weaveIncStayTime Wait mode 0- period does not contain wait time; 1- Period contains the wait time
	 * @param [in] weaveRange weave amplitude(mm)
	 * @param [in] weaveLeftRange Vertical triangle swing left chord length (mm)
	 * @param  [in] weaveRightRange Vertical triangle swing right chord length (mm)
	 * @param  [in] additionalStayTime Vertical triangle swing vertical triangle point stay time (mm)
	 * @param [in] weaveLeftStayTime weave left residence time(ms)
	 * @param [in] weaveRightStayTime weave right residence time(ms)
	 * @param [in] weaveCircleRadio Circular wiggle-pullback ratio(0-100%)
	 * @param [in] weaveStationary weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time
	 * @param [in] weaveYawAngle Swing direction azimuth Angle (rotation around the z-axis of swing), unit °
	 * @param [in] weaveRotAngle Swing direction lateral Angle (deflection around the X-axis of the swing), unit °
	 * @return Error code
	 */
	errno_t WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, 
                            int weaveIncStayTime, double weaveRange, double weaveLeftRange, 
                            double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, 
                            int weaveRightStayTime, int weaveCircleRadio, int weaveStationary, double weaveYawAngle, double weaveRotAngle = 0);

	/**
	 * @brief Set weave parameters in real time
	 * @param [in] weaveNum parameters number
	 * @param [in] weaveType weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
	 * @param [in] weaveFrequency weave frequency(Hz)
	 * @param [in] weaveIncStayTime Wait mode 0- period does not contain wait time; 1- Period contains the wait time
	 * @param [in] weaveRange weave amplitude(mm)
	 * @param [in] weaveLeftStayTime weave left residence time(ms)
	 * @param [in] weaveRightStayTime weave right residence time(ms)
	 * @param [in] weaveCircleRadio Circular wiggle-pullback ratio(0-100%)
	 * @param [in] weaveStationary weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time
	 * @return Error code
	 */
	errno_t WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

	/**
	 * @brief Weave start
	 * @param [in] weaveNum Weave welding parameter configuration number
	 * @return Error code
	 */
	errno_t WeaveStart(int weaveNum);

	/**
	 * @brief Weave end
	 * @param [in] weaveNum Weave welding parameter configuration number
	 * @return Error code
	 */
	errno_t WeaveEnd(int weaveNum);

	/**
	 * @brief Forward Wire Feed
	 * @param [in] ioType 0-control box IO； 1-extend IO
	 * @param [in] wireFeed wire control: 0-stop wire feed ；1-wire feed
	 * @return Error code
	 */
	errno_t SetForwardWireFeed(int ioType, int wireFeed);

	/**
	 * @brief Reverse wire feed
	 * @param [in] ioType 0-control box IO； 1-extend IO
	 * @param [in] wireFeed wire control: 0-stop wire feed ；1-wire feed
	 * @return Error code
	 */
	errno_t SetReverseWireFeed(int ioType, int wireFeed);

	/**
	 * @brief aspirated
	 * @param [in] ioType  0-control box IO； 1-extend IO
	 * @param [in] airControl aspirated control: 0-stop aspirated；1-aspirated
	 * @return Error code
	 */
	errno_t SetAspirated(int ioType, int airControl);

/**
	 *@brief Segment weld start
	 *@param[in]startDesePos Starting point Cartesian position
	 *@param[in]endDesePos Ending point Cartesian position
	 *@param[in]startJPos Starting point joint position
	 *@param[in]endJPos Ending point joint position
	 *@param[in]weldLength Weld length(mm)
	 *@param[in]noWeldLength Length of unwelded section(mm)
	 *@param[in]weldIOType 0-control box IO； 1-extend IO(0-控制箱IO；1-扩展IO)
	 *@param[in]arcNum Welder configuration file number
	 *@param[in]weldTimeout Arcing timeout time
	 *@param[in]isWeave Weave or not
	 *@param[in]weaveNum Weave welding parameter configuration number
	 *@param[in]tool tool number
	 *@param[in]user Workpiece coordinate number, range [0~14]
	 *@param[in]vel Percentage of speed [0~100]
	 *@param[in]acc Acceleration percentage, range[0~100]
	 *@param[in]ovl Velocity scaling factor, range[0~100]
	 *@param[in]blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
	 *@param[in]epos Position of expansion shaft, unit: mm
 	 *@param[in]search 0- no wire seeking, 1- wire seeking
	 *@param[in]offset_flag 0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	 *@param[in]offset_pos The pose offset
	 *@return Error code
	 */
	errno_t SegmentWeldStart(DescPose *startDesePos, DescPose *endDesePos, JointPos *startJPos, JointPos *endJPos,
							 double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout,
							 bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR,
							 ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

	/**
	 * @brief Initialize log parameters;
	 * @param output_model：Output mode, 0-direct output; 1-buffered output; 2 - asynchronous output;
	 * @param file_path: File save path + name, the maximum length is 256, and the name must be in the form of xxx.log, such as /home/fr/linux/fairino.log;
	 * @param file_num：Scroll the number of files stored, 1~20. The maximum size of a single file is 50M;
	 * @return errno_t Error code;
	 */
	errno_t LoggerInit(int output_model = 0, std::string file_path = "", int file_num = 5);

    /**
     * @brief Set the log filtering level;
     * @param lvl: Filter the level value, the smaller the value, the less the output log, the default value is 1. 1-error, 2-warnning, 3-inform, 4-debug;
    */
    void SetLoggerLevel(int lvl = 1);

	/**
     * @brief Download Lua file
     * @param [in] fileName The name of the lua file to be downloaded "test.lua"
     * @param [in] savePath Save the file local path "D://Down/"
     * @return error code
     */
	errno_t LuaDownLoad(std::string fileName, std::string savePath);

	/**
     * @brief Upload Lua file
     * @param [in] filePath local lua file path name
     * @return error code
     */
	errno_t LuaUpload(std::string filePath);

	/**
     * @brief Delete Lua files
     * @param [in] fileName The name of the lua file to be deleted "test.lua"
     * @return error code
     */
	errno_t LuaDelete(std::string fileName);

	/**
     * @brief Get the names of all current lua files
     * @param [out] luaNames lua file name list
     * @return error code
     */
	errno_t GetLuaList(std::list<std::string>* luaNames);

	/**
	  * @brief Set 485 extended axis parameters
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [in] servoCompany Servo drive manufacturer, 1-Dynatec
      * @param [in] servoModel servo drive model, 1-FD100-750C
      * @param [in] servoSoftVersion servo driver software version, 1-V1.0
      * @param [in] servoResolution encoder resolution
      * @param [in] axisMechTransRatio mechanical transmission ratio
      * @return error code
	 */
	errno_t AuxServoSetParam(int servoId, int servoCompany, int servoModel,
							 int servoSoftVersion, int servoResolution, double axisMechTransRatio);
	/**
	 * @brief Get 485 extended axis configuration parameters
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [out] servoCompany Servo drive manufacturer, 1-Dynatec
      * @param [out] servoModel Servo drive model, 1-FD100-750C
      * @param [out] servoSoftVersion servo driver software version, 1-V1.0
      * @param [out] servoResolution encoder resolution
      * @param [out] axisMechTransRatio mechanical transmission ratio
      * @return error code
	 */
	errno_t AuxServoGetParam(int servoId, int* servoCompany, int* servoModel,
                                  int* servoSoftVersion, int* servoResolution, double* axisMechTransRatio);

	/**
	 * @brief Set 485 expansion axis enable/disable
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [in] status enable status, 0-disabled, 1-enabled
      * @return error code
	 */
	errno_t AuxServoEnable(int servoId, int status);

	/**
	 * @brief Set 485 extended axis control mode
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [in] mode control mode, 0-position mode, 1-speed mode
      * @return error code
	 */
	errno_t AuxServoSetControlMode(int servoId, int mode);
	/**
	 * @brief Set the 485 extended axis target position (position mode)
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [in] pos target position, mm or °
      * @param [in] speed target speed, mm/s or °/s
      * @return error code
	 */
	errno_t AuxServoSetTargetPos(int servoId, double pos, double speed, double acc = 100.0);

	/**
	 * @brief Set the 485 extended axis target speed (speed mode)
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [in] speed target speed, mm/s or °/s
      * @return error code
	 */
	errno_t AuxServoSetTargetSpeed(int servoId, double speed, double acc = 100.0);

	/**
	 * @brief Set 485 extended axis target torque (torque mode)
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [in] torque target torque, Nm
      * @return error code
	 */
	errno_t AuxServoSetTargetTorque(int servoId, double torque);
	/**
	 * @brief Set 485 extended axis zero return
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [in] mode zero return mode, 0-current position return to zero; 1-limit return to zero
      * @param [in] searchVel zero return speed, mm/s or °/s
      * @param [in] latchVel hoop speed, mm/s or °/s
      * @return error code
	 */
	errno_t AuxServoHoming(int servoId, int mode, double searchVel, double latchVel, double acc = 100.0);

	/**
	 * @brief Clear 485 extended axis error message
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @return error code
	 */
	errno_t AuxServoClearError(int servoId);
	/**
	 * @brief Get 485 extended axis servo status
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @param [out] servoErrCode servo drive fault code
      * @param [out] servoState servo drive status [decimal number converted to binary, bit0-bit5: servo enable-servo running-positive limit trigger-negative limit trigger-positioning completed-zero return completed]
      * @param [out] servoPos servo current position mm or °
      * @param [out] servoSpeed Servo current speed mm/s or °/s
      * @param [out] servoTorque Servo current torque Nm
      * @return error code
	 */
	errno_t AuxServoGetStatus(int servoId, int* servoErrCode, int* servoState, double* servoPos,
                                   double* servoSpeed, double* servoTorque);
	/**
	  * @brief Set the 485 extended axis data axis number in status feedback
      * @param [in] servoId servo drive ID, range [1-16], corresponding slave ID
      * @return error code
	 */
	errno_t AuxServosetStatusID(int servoId);
	/**
	 * @brief Get the real-time status structure of the robot
      * @param [out] pkg robot real-time status structure
      * @return error code
	 */
	errno_t GetRobotRealTimeState(ROBOT_STATE_PKG *pkg);
	/**
	* @brief Get the robot peripheral protocol
      * @param [out] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
      * @return error code
	 */
	errno_t GetExDevProtocol(int *protocol);

	/**
	  * @brief Set the robot peripheral protocol
      * @param [in] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
      * @return error code
	 */
	errno_t SetExDevProtocol(int protocol);

	/**
	 * @brief set robot acceleration
	 * @param [in] acc acceleration
	 * @return error code
	 */
	errno_t SetOaccScale(double acc);

	/**
	* @brief Set control box AO when the robot moves Start
	* @param [in] AONum Control box AO num
	* @param [in] maxTCPSpeed the maximum TCP speed[1-5000mm/s]，default 1000
	* @param [in] maxAOPercent the AO percentage corresponding to the maximum TCP speed, default 100%
	* @param [in] zeroZoneCmp dead zone compensation value AO percentage, integer, default is 20, range [0-100]
	* @return error code
	*/
	errno_t MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

	/**
	* @brief Set control box AO when the robot moves stop
	* @return error code
	*/
	errno_t MoveAOStop();

	/**
	* @brief Set tool AO when the robot moves start
	* @param [in] AONum tool AO num
	* @param [in] maxTCPSpeed the maximum TCP speed[1-5000mm/s]，default 1000
	* @param [in] maxAOPercent the AO percentage corresponding to the maximum TCP speed, default 100%
	* @param [in] zeroZoneCmp dead zone compensation value AO percentage, integer, default is 20, range [0-100]
	* @return error code
	*/
	errno_t MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

	/**
	* @brief Set tool AO when the robot moves stop
	* @return error code
	*/
	errno_t MoveToolAOStop();

	/**
	* @brief Configure UDP extension axis communication parameters
	* @param [in] ip PLC IP address
	* @param [in] port	 port num
	* @param [in] period	Communication period(ms，default 2ms)
	* @param [in] lossPkgTime	Packet loss detection time(ms)
	* @param [in] lossPkgNum	the number of packet loss times
	* @param [in] disconnectTime	the duration of communication disconnection confirmation
	* @param [in] reconnectEnable	 Automatic reconnection when communication is disconnected Enable;0-Disable, 1-Enable
	* @param [in] reconnectPeriod	 Reconnection period(ms)
	* @param [in] reconnectNum Reconnection times
	* @param [in] selfConnect Whether the connection is automatically established after restarting the robot
	* @return error code
	*/
	errno_t ExtDevSetUDPComParam(std::string ip, int port, int period, int lossPkgTime, int lossPkgNum, int disconnectTime, int reconnectEnable, int reconnectPeriod, int reconnectNum, int selfConnect = 1);

	/**
	* @brief Get the UDP extension axis communication parameter configuration
	* @param [out] ip PLC IP address
	* @param [out] port port num
	* @param [out] period	Communication period(ms，default 2ms)
	* @param [out] lossPkgTime Packet loss detection time(ms)
	* @param [out] lossPkgNum	 Indicates the number of packet loss times
	* @param [out] disconnectTime	 the duration of communication disconnection confirmation
	* @param [out] reconnectEnable Automatic reconnection when communication is disconnected Enable;0-Disable, 1-Enable
	* @param [out] reconnectPeriod Reconnection period(ms)
	* @param [out] reconnectNum	Reconnection times
	* @return error code
	*/
	errno_t ExtDevGetUDPComParam(std::string& ip, int& port, int& period, int& lossPkgTime, int& lossPkgNum, int& disconnectTime, int& reconnectEnable, int& reconnectPeriod, int& reconnectNum);

	/**
	* @brief Load the UDP communication connection
	* @return error code
	*/
	errno_t ExtDevLoadUDPDriver();

	/**
	* @brief Unload the UDP communication connection
	* @return error code
	*/
	errno_t ExtDevUnloadUDPDriver();

	/**
	* @brief Set the UDP extension axis homing
	* @param [in] axisID Axis number[1-4]
	* @param [in] mode homing mode; 0-Current position homing, 1-negative limit homing, 2-positive limit homing
	* @param [in] searchVel homing velocity(mm/s)
	* @param [in] latchVel homing latch velocity(mm/s)
	* @return error code
	*/
	errno_t ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel);

	/**
	* @brief UDP extension axis jog start
	* @param [in] axisID Axis number[1-4]
	* @param [in] direction Rotation direction 0- reverse; 1-forward
	* @param [in] vel velocity(mm/s)
	* @param [in] acc Acceleration  (mm/s2)
	* @param [in] maxDistance maximum jog distance(mm)
	* @return error code
	*/
	errno_t ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance);

	/**
	* @brief UDP extension axis jog stop
	* @param [in] axisID Axis number[1-4]
	* @return error code
	*/
	errno_t ExtAxisStopJog(int axisID);

	/**
	* @brief Enable the UDP extension axis
	* @param [in] axisID Axis number [1-4]
	* @param [in] status 0-Disable, 1-Enable
	* @return error code
	*/
	errno_t ExtAxisServoOn(int axisID, int status);

	/**
	* @brief UDP extension axis movement
	* @param [in] pos target position
	* @param [in] ovl Speed percentage
	* @param [in] blend Smooth parameter (mm or ms)
	* @return error code
	*/
	errno_t ExtAxisMove(ExaxisPos pos, double ovl, double blend = -1);

	/**
	* @brief Set extended DO
	* @param [in] DONum DO number
	* @param [in] bOpen True- on,False- off
	* @param [in] smooth whether it is smooth; True-Yes, False-no
	* @param [in] block True-block, False-no block
	* @return error code
	*/
	errno_t SetAuxDO(int DONum, bool bOpen, bool smooth, bool block);

	/**
	* @brief Set extended AO
	* @param [in] AONum AO number
	* @param [in] value analog quantity value [0-4095]
	* @param [in] block True-block, False-no block
	* @return error code
	*/
	errno_t SetAuxAO(int AONum, double value, bool block);

	/**
	* @brief Set the extended DI input filtering time
	* @param [in] filterTime DI input filtering time(ms)
	* @return error code
	*/
	errno_t SetAuxDIFilterTime(int filterTime);

	/**
	* @brief Set the extended AI input filtering time
	* @param [in] filterTime AI input filtering time(ms)
	* @return error code
	*/
	errno_t SetAuxAIFilterTime(int AONum, int filterTime);

	/**
	* @brief Wait for the extended DI input
	* @param [in] DINum DI number
	* @param [in] bOpen True- on,False- off
	* @param [in] time Maximum waiting time(ms)
	* @param [in] errorAlarm Whether to continue a motion. True- Yes,False- no
	* @return error code
	*/
	errno_t WaitAuxDI(int DINum, bool bOpen, int time, bool errorAlarm);

	/**
	* @brief Wait for the extended AI input
	* @param [in] AINum AI number
	* @param [in] sign 0-greater than, 1-less than
	* @param [in] value AI value
	* @param [in] time Maximum waiting time(ms)
	* @param [in] errorAlarm Whether to continue a motion. True- Yes,False- no
	* @return error code
	*/
	errno_t WaitAuxAI(int AINum, int sign, int value, int time, bool errorAlarm);

	/**
	* @brief Gets the extended DI value
	* @param [in] DINum DI number
	* @param [in] isNoBlock True-block, False-no block
	* @param [out] isOpen True- on,False- off
	* @return error code
	*/
	errno_t GetAuxDI(int DINum, bool isNoBlock, bool& isOpen);

	/**
	* @brief Gets the extended AI value
	* @param [in] AINum AI number
	* @param [in] isNoBlock True-block, False-no block
	* @param [in] value input value
	* @return error code
	*/
	errno_t GetAuxAI(int AINum, bool isNoBlock, int& value);

	/**
	* @brief Reconnect UDP communication after abnormal disconnected
	* @return error code
	*/
	errno_t ExtDevUDPClientComReset();

	/**
	* @brief Close UDP communication after abnormal disconnected
	* @return error code
	*/
	errno_t ExtDevUDPClientComClose();

	/**
	* @brief Configure UDP extension axis parameters
	* @param [in] axisID Axis number [1-4]
	* @param [in] axisType Extended axis type; 0-translation, 1-rotation
	* @param [in] axisDirection Axis direction; 0-forward; 1-reverse
	* @param [in] axisMax The maximum position of the extension axis(mm)
	* @param [in] axisMin Minimum position of the extension axis (mm)
	* @param [in] axisVel Speed mm/s
	* @param [in] axisAcc Acceleration mm/s2
	* @param [in] axisLead Lead mm
	* @param [in] encResolution Encoder resolution
	* @param [in] axisOffect The start point of the weld extension axis offset
	* @param [in] axisCompany Driver manufacturer 1-Hechuan; 2- Huichuan; 3- Panasonic
	* @param [in] axisModel Driver models 1-Hechuan SV-XD3EA040L-E, 2-Hechuan SV-X2EA150A-A, 1-Huichuan SV620PT5R4I, 1-Matsushita MADLN15SG, 2-Matsushita MSDLN25SG, 3-Matsushita MCDLN35SG
	* @param [in] axisEncType Encoder type 0-increments; 1- absolute value
	* @return error code
	*/
	errno_t ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, int encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType);

	/**
	* @brief Set the installation position of the expansion shaft
	* @param [in] installType  0-The robot is installed on the external axis, 1-the robot is installed outside the external axis
	* @return error code
	*/
	errno_t SetRobotPosToAxis(int installType);

	/**
	* @brief Set the extended shaft system DH parameters
	* @param [in] axisConfig 0-single DOF linear slide, 1-2 DOF L-type positioner, 2-3 DOF, 3-4 DOF, 4-single DOF positioner
	* @param [in] axisDHd1 External axisDH parameter d1 mm
	* @param [in] axisDHd2 External axisDH parameter d2 mm
	* @param [in] axisDHd3 External axisDH parameter d3 mm
	* @param [in] axisDHd4 External axisDH parameter d4 mm
	* @param [in] axisDHa1 External axisDH parameter 11 mm
	* @param [in] axisDHa2 External axisDH parameter a2 mm
	* @param [in] axisDHa3 External axisDH parameter a3 mm
	* @param [in] axisDHa4 External axisDH parameter a4 mm
	* @return error code
	*/
	errno_t SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4);

	/**
	* @brief Set the reference point of the extended axis coordinate system - four-point method
	* @param [in] pointNum Point number [1-4]
	* @return error code
	*/
	errno_t ExtAxisSetRefPoint(int pointNum);

	/**
	* @brief Calculation of extended axis coordinate system - four-point method
	* @param [out] coord coordinate values
	* @return error code
	*/
	errno_t ExtAxisComputeECoordSys(DescPose& coord);

	/**
	* @brief Apply the extended axis coordinate system
	* @param [in]  axisCoordNum coordinate system number
	* @param [in]  toolNum tool number
	* @param [in]  coord coordinate values
	* @param [in]  calibFlag calibflag 0-No, 1-yes
	* @return error code
	*/
	errno_t ExtAxisActiveECoordSys(int axisCoordNum, int toolNum, DescPose coord, int calibFlag);

	/**
	* @brief Set the pose of the calibration reference point in the end coordinate system of the positioner
	* @param [in] pos Position value
	* @return error code
	*/
	errno_t SetRefPointInExAxisEnd(DescPose pos);

	/**
	* @brief Positioner coordinate system reference point setting - four-point method
	* @param [in] pointNum Point number[1-4]
	* @return error code
	*/
	errno_t PositionorSetRefPoint(int pointNum);

	/**
	* @brief Coordinate system calculation of positioner - four-point method
	* @param [out] coord coordinate values
	* @return error code
	*/
	errno_t PositionorComputeECoordSys(DescPose& coord);

	/**
	* @brief The UDP expansion axis moves synchronously with the robot joint movement
	* @param [in] joint_pos Position of the target joint, unit[°]
	* @param [in] desc_pos target Cartesian pose, unit[mm]
	* @param [in] tool Tool number [0~14]
	* @param [in] user workpiece number [0~14]
	* @param [in] vel Speed percentage [0~100]
	* @param [in] acc Acceleration percentage[0~100]
	* @param [in] ovl Speed scaling factor[0~100]
	* @param [in] epos the external axis position[mm]
	* @param [in] blendT [1.0] - movement in place (block), [0 ~ 500.0]-smooth time (non-blocking), unit[ms]
	* @param [in] offset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos position offset
	* @return error code
	*/
	errno_t ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, uint8_t offset_flag, DescPose offset_pos);

	/**
	* @brief The UDP expansion axis moves synchronously with the robot joint movement(Overloaded functions do not require the input of Cartesian positions)
	* @param [in] joint_pos Position of the target joint, unit[°]
	* @param [in] tool Tool number [0~14]
	* @param [in] user workpiece number [0~14]
	* @param [in] vel Speed percentage [0~100]
	* @param [in] acc Acceleration percentage[0~100]
	* @param [in] ovl Speed scaling factor[0~100]
	* @param [in] epos the external axis position[mm]
	* @param [in] blendT [1.0] - movement in place (block), [0 ~ 500.0]-smooth time (non-blocking), unit[ms]
	* @param [in] offset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos position offset
	* @return error code
	*/
	errno_t ExtAxisSyncMoveJ(JointPos joint_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, uint8_t offset_flag, DescPose offset_pos);

	/**
	* @brief  The UDP extension axis moves synchronously with the robot’s linear motion
	* @param [in] joint_pos Position of the target joint, unit[°]
	* @param [in] desc_pos target Cartesian pose, unit[mm]
	* @param [in] tool Tool number [0~14]
	* @param [in] user workpiece number [0~14]
	* @param [in] vel Speed percentage [0~100]
	* @param [in] acc Acceleration percentage[0~100]
	* @param [in] ovl Speed scaling factor[0~100]
	* @param [in] blendR 1.0-movement in place (block), [0 ~ 1000] - smooth radius (non-blocking), unit (mm) 1.0 by default
	* @param [in] epos the external axis position[mm]
	* @param [in] offset_flag  0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos position offset
	* @return error code
	*/
	errno_t ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, uint8_t offset_flag, DescPose offset_pos);

	/**
	* @brief  The UDP extension axis moves synchronously with the robot’s linear motion(Overloaded functions do not require the input of robot joint positions)
	* @param [in] desc_pos target Cartesian pose, unit[mm]
	* @param [in] tool Tool number [0~14]
	* @param [in] user workpiece number [0~14]
	* @param [in] vel Speed percentage [0~100]
	* @param [in] acc Acceleration percentage[0~100]
	* @param [in] ovl Speed scaling factor[0~100]
	* @param [in] blendR 1.0-movement in place (block), [0 ~ 1000] - smooth radius (non-blocking), unit (mm) 1.0 by default
	* @param [in] epos the external axis position[mm]
	* @param [in] offset_flag  0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos position offset
	* @param [in] config Robot inverse kinematics calculation joint configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration
	* @return error code
	*/
	errno_t ExtAxisSyncMoveL(DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, uint8_t offset_flag, DescPose offset_pos, int config = -1);

	/**
	* @brief The UDP extension axis moves synchronously with the robot arc motion
	* @param [in] joint_pos_p joint position of a pathpoint [°]
	* @param [in] desc_pos_p path point Cartesian pose[mm]
	* @param [in] ptool  path point tool number[0~14]
	* @param [in] puser  path point workpiece number[0~14]
	* @param [in] pvel  Speed percentage [0~100]
	* @param [in] pacc  Acceleration percentage[0~100]
	* @param [in] epos_p Pathpoint external axis position mm
	* @param [in] poffset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos_p  path point position offset
	* @param [in] joint_pos_t joint position of the target point[°]
	* @param [in] desc_pos_t Cartesian position of the target point[mm]
	* @param [in] ttool  target point tool number[0~14]
	* @param [in] tuser  target point workpiece number[0~14]
	* @param [in] tvel  Speed percentage[0~100]
	* @param [in] tacc  Acceleration percentage[0~100]
	* @param [in] epos_t target point external axis position mm
	* @param [in] toffset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos_t target point position offset
	* @param [in] ovl Speed scaling factor [0~100]
	* @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
	* @return error code
	*/
	errno_t ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, uint8_t poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, uint8_t toffset_flag, DescPose offset_pos_t, float ovl, float blendR);
	
	/**
	* @brief The UDP extension axis moves synchronously with the robot arc motion(Overloaded functions do not require the input of robot joint positions)
	* @param [in] desc_pos_p path point Cartesian pose[mm]
	* @param [in] ptool  path point tool number[0~14]
	* @param [in] puser  path point workpiece number[0~14]
	* @param [in] pvel  Speed percentage [0~100]
	* @param [in] pacc  Acceleration percentage[0~100]
	* @param [in] epos_p Pathpoint external axis position mm
	* @param [in] poffset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos_p  path point position offset
	* @param [in] desc_pos_t Cartesian position of the target point[mm]
	* @param [in] ttool  target point tool number[0~14]
	* @param [in] tuser  target point workpiece number[0~14]
	* @param [in] tvel  Speed percentage[0~100]
	* @param [in] tacc  Acceleration percentage[0~100]
	* @param [in] epos_t target point external axis position mm
	* @param [in] toffset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
	* @param [in] offset_pos_t target point position offset
	* @param [in] ovl Speed scaling factor [0~100]
	* @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
	* @param [in] config Robot inverse kinematics calculation joint configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration
	* @return error code
	*/
	errno_t ExtAxisSyncMoveC(DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, uint8_t poffset_flag, DescPose offset_pos_p, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, uint8_t toffset_flag, DescPose offset_pos_t, float ovl, float blendR, int config = -1);
	
	/**
	* @brief  Wire search begins
	* @param  [in] refPos  1- Reference point 2- contact point
	* @param  [in] searchVel   Search speed %
	* @param  [in] searchDis  Seeking distance mm
	* @param  [in] autoBackFlag Automatic return flag, 0- not automatic; - Auto
	* @param  [in] autoBackVel  Automatic return speed %
	* @param  [in] autoBackDis  Automatic return distance mm
	* @param  [in] offectFlag  1- Find with offset; 2- Find the teaching point
	* @return  error code
	*/
	 errno_t WireSearchStart(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag);

	 /**
	  * @brief  Wire locating is complete
	  * @param  [in] refPos  1- Reference point 2- contact point
	  * @param  [in] searchVel   Search speed %
	  * @param  [in] searchDis  Seeking distance mm
	  * @param  [in] autoBackFlag Automatic return flag, 0- not automatic; - Auto
	  * @param  [in] autoBackVel  Automatic return speed %
	  * @param  [in] autoBackDis  Automatic return distance mm
	  * @param  [in] offectFlag  1- Find with offset; 2- Find the teaching point
	  * @return  error code
	  */
	 errno_t WireSearchEnd(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag);

	 /**
	  * @brief  Calculate the seeking offset of the welding wire
	  * @param  [in] seamType  Weld type
	  * @param  [in] method   Calculation method
	  * @param  [in] varNameRef Reference points 1-6, "#" indicates no point variable
	  * @param  [in] varNameRes Contact points 1-6, "#" indicates no point variable
	  * @param  [out] offectFlag 0- offset is superimposed directly to the instruction point; 1- Offset requires a coordinate transformation of the instruction point
	  * @param  [out] offect Offset pose[x, y, z, a, b, c]
	  * @return  error code
	  */
	 errno_t GetWireSearchOffset(int seamType, int method, std::vector<std::string> varNameRef, std::vector<std::string> varNameRes, int& offectFlag, DescPose& offect);

	 /**
	  * @brief  Wait for wire locating to complete
	  * @return  error code
	  */
	 errno_t WireSearchWait(std::string varName);

	 /**
	  * @brief  Wire seeking contact is written to the database
	  * @param  [in] varName  Contact point name: RES0 ~ RES99
	  * @param  [in] pos  Contact data[x, y, x, a, b, c]
	  * @return  error code
	  */
	 errno_t SetPointToDatabase(std::string varName, DescPose pos);
	
	 /**
	  * @brief  Arc tracking control
	  * @param  [in] flag Switch, 0-off; 1-on
	  * @param  [in] dalayTime Lag time, in ms
	  * @param  [in] isLeftRight Left-right deviation compensation
	  * @param  [in] klr Left-right adjustment coefficient (sensitivity);
	  * @param  [in] tStartLr Left-right start compensation time around cyc
	  * @param  [in] stepMaxLr Left-right the maximum compensation amount each time mm
	  * @param  [in] sumMaxLr Left-right total maximum compensation mm
	  * @param  [in] isUpLow Up-down compensation
	  * @param  [in] kud Up-down adjustment factor;
	  * @param  [in] tStartUd Start Up-down compensation time cyc
	  * @param  [in] stepMaxUd Maximum compensation amount Up-down each time mm
	  * @param  [in] sumMaxUd Total maximum compensation Up-down
	  * @param  [in] axisSelect Up-down coordinate system selection, 0-swing; 1- Tools; 2- Base
	  * @param  [in] referenceType Up-down reference current setting mode, 0-feedback; 1-constant
	  * @param  [in] referSampleStartUd Up-down reference current sampling start count (feedback);cyc
	  * @param  [in] referSampleCountUd Up-down reference current sampling cycle count;cyc
	  * @param  [in] referenceCurrent Up-down reference current mA
	  * @param  [in] offsetType Indicates the offset tracking type. 0- no offset. 1- Sampling; 2- percent
	  * @param  [in] offsetParameter Offset parameter; Sampling (offset sampling start time, default sampling cycle); Percentage (offset percentage (-100 ~ 100))
	  * @return  error code
	  */
	 errno_t ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent, int offsetType = 0, int offsetParameter = 0);


	 /**
	  * @brief  Wire seeking contact is written to the database
	  * @param  [in] channel Arc tracking AI passband selection,[0-3]
	  * @return  error code
	  */
	 errno_t ArcWeldTraceExtAIChannelConfig(int channel);


	 /**
	  * @brief  Force sensor assists drag
	  * @param  [in] status Control status, 0- off; 1- On
	  * @param  [in] asaptiveFlag Adaptive on flag, 0- off; 1- On
	  * @param  [in] interfereDragFlag Interference drag flag, 0- off; 1- On
	  * @param  [in] ingularityConstraintsFlag Singularity strategy, 0- evade; 1- Crossing
	  * @param  [in] M Inertia coefficient
	  * @param  [in] B Damping coefficient
	  * @param  [in] K Stiffness coefficient
	  * @param  [in] F Drag the six-dimensional force threshold
	  * @param  [in] Fmax Maximum towing power limit
	  * @param  [in] Vmax Maximum joint speed limit
	  * @return  error code
	  */
	 errno_t EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag, int ingularityConstraintsFlag, std::vector<double> M, std::vector<double> B, std::vector<double> K, std::vector<double> F, double Fmax, double Vmax);
	 
	 /**
	  * @brief  Force sensor assists drag
	  * @param  [in] status Control status, 0- off; 1- On
	  * @param  [in] asaptiveFlag Adaptive on flag, 0- off; 1- On
	  * @param  [in] interfereDragFlag Interference drag flag, 0- off; 1- On
	  * @param  [in] ingularityConstraintsFlag Singularity strategy, 0- evade; 1- Crossing
	  * @param  [in] forceCollisionFlag Robot collision detection mark during assisted dragging 0- Close 1- Open
	  * @param  [in] M Inertia coefficient
	  * @param  [in] B Damping coefficient
	  * @param  [in] K Stiffness coefficient
	  * @param  [in] F Drag the six-dimensional force threshold
	  * @param  [in] Fmax Maximum towing power limit
	  * @param  [in] Vmax Maximum joint speed limit
	  * @return  error code
	  */
	 errno_t EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag, int ingularityConstraintsFlag, int forceCollisionFlag, std::vector<double> M, std::vector<double> B, std::vector<double> K, std::vector<double> F, double Fmax, double Vmax);


	 /**
	  * @brief  The force sensor automatically On after the error is cleared
	  * @param  [in] status Control status, 0- off; 1- On
	  * @return  error code
	  */
	 errno_t SetForceSensorDragAutoFlag(int status);


	/**
	* @brief sets the hybrid drag switch and parameters of six-dimensional force and joint impedance
	* @param [in] status Control status, 0- off; 1- Open
	* @param [in] impedanceFlag Impedanceflag, 0-off; 1- Open
	* @param [in] lamdeDain Drag gain
	* @param [in] KGain Stiffness gain
	* @param [in] BGain Damping gain
	* @param [in] dragMaxTcpVel Drag the end maximum line speed limit
	* @param [in] dragMaxTcpOriVel Drag end maximum angular speed limit
	* @return  error code
	*/
	errno_t ForceAndJointImpedanceStartStop(int status, int impedanceFlag, std::vector<double> lamdeDain, std::vector<double> KGain, std::vector<double> BGain, double dragMaxTcpVel, double dragMaxTcpOriVel);

	/**
	* @brief get the drag switch status of the force sensor
	* @param [out] dragState Force sensor auxiliary drag control status, 0-off; 1- Open
	* @param [out] sixDimensionalDragState Control state, 0- off; 1- Open
	* @return  error code
	*/
	errno_t GetForceAndTorqueDragState(int& dragState, int& sixDimensionalDragState);

	/**
	* @brief Sets the load weight under the force sensor
	* @param [in] weight Load weight kg
	* @return  error code
	*/
	 errno_t SetForceSensorPayload(double weight);

	 /**
	  * @brief sets the load center of mass under the force sensor
	  * @param [in] x Load centroid x mm
	  * @param [in] y Load centroid y mm
	  * @param [in] z Load centroid z mm
	  * @return  error code
	  */
	 errno_t SetForceSensorPayloadCog(double x, double y, double z);

	 /**
	  * @brief obtains the load weight under the force sensor
	  * @param [in] weight Load weight kg
	  * @return  error code
	  */
	 errno_t GetForceSensorPayload(double& weight);

	 /**
	  * @brief obtains the load center of mass under the force sensor
	  * @param [out] x Load centroid x mm
	  * @param [out] y Load centroid y mm
	  * @param [out] z Load centroid z mm
	  * @return  error code
	  */
	 errno_t GetForceSensorPayloadCog(double& x, double& y, double& z);

	 /**
	  * @brief force sensor automatically adjusts to zero
	  * @param [out] weight Weight of the sensor kg
	  * @param [out] pos sensor centroid mm
	  * @return  error code
	  */
	 errno_t ForceSensorAutoComputeLoad(double& weight, DescTran& pos);
	
	 /**
	  * @brief sensor automatically zero data recording
	  * @param [in] recordCount Number of recorded data 1-3
	  * @return  error code
	  */
	 errno_t ForceSensorSetSaveDataFlag(int recordCount);

	/**
	* @brief sensor automatic zero correction calculation
	* @param [out] weight Weight of the sensor kg
	* @param [out] pos sensor centroid [x, y, z]
	* @return  error code
	*/
	errno_t ForceSensorComputeLoad(double& weight, DescTran& pos);
	
	/**
	* @brief segment welding get Position and attitude obtained 
	* @param [in] startPos Coordinates of the start point
	* @param [in] endPos end point coordinates
	* @param [in] startDistance Length from welding point to starting point
	* @param [out] weldPointDesc Cartesian coordinate information of the weld point
	* @param [out] weldPointJoint Cartesian coordinate information of weldpointjoint
	* @param [out] tool Indicates the tool number
	* @param [out] user Job ID
	* @return  error code
	*/
	errno_t GetSegmentWeldPoint(DescPose startPos, DescPose endPos, double startDistance, DescPose& weldPointDesc, JointPos& weldPointJoint, int& tool, int& user);

	/**
	* @brief Set welding process curve parameters
	* @param [in] id Welding process number (1-99)
	* @param [in] startCurrent Arcing current (A)
	* @param [in] startVoltage Arc voltage (V)
	* @param [in] startTime Arc starting time (ms)
	* @param [in] weldCurrent Welding current (A)
	* @param [in] weldVoltage Welding voltage (V)
	* @param [in] endCurrent (A)
	* @param [in] endVoltage (V)
	* @param [in] endTime Arc recovery time (ms)
	* @return  error code
	*/
	errno_t WeldingSetProcessParam(int id, double startCurrent, double startVoltage, double startTime, double weldCurrent, double weldVoltage, double endCurrent, double endVoltage, double endTime);

	/**
	* @brief Obtain welding process curve parameters
	* @param [in] id Welding process number (1-99)
	* @param [out] startCurrent Arcing current (A)
	* @param [out] startVoltage Arc voltage (V)
	* @param [out] startTime Arc starting time (ms)
	* @param [out] weldCurrent Welding current (A)
	* @param [out] weldVoltage Welding voltage (V)
	* @param [out] endCurrent (A)
	* @param [out] endVoltage Return voltage (V)
	* @param [out] endTime Arc recovery time (ms)
	* @return  error code
	*/
	errno_t WeldingGetProcessParam(int id, double& startCurrent, double& startVoltage, double& startTime, double& weldCurrent, double& weldVoltage, double& endCurrent, double& endVoltage, double& endTime);

	/**
	* @brief end sensor configuration
	* @param [in] idCompany, 18-JUNKONG; 25-HUIDE
	* @param [in] idDevice type, 0-JUNKONG/RYR6T.V1.0
	* @param [in] idSoftware Software version, 0-J1.0/HuiDe1.0(not yet available)
	* @param [in] idBus mount position, 1- end port 1; 2- Terminal port 2... 8- Terminal Port 8 (not yet open)
	* @return  error code
	*/
	errno_t AxleSensorConfig(int idCompany, int idDevice, int idSoftware, int idBus);

	/**
	* @brief gets the end sensor configuration
	* @param [out] idCompany, 18-JUNKONG; 25-HUIDE
	* @param [out] idDevice type, 0-JUNKONG/RYR6T.V1.0
	* @return  error code
	*/
	errno_t AxleSensorConfigGet(int& idCompany, int& idDevice);

	/**
	* @brief end sensor activation
	* @param [in] actFlag 0- Reset; 1- Activation
	* @return  error code
	*/
	errno_t AxleSensorActivate(int actFlag);

	/**
	* @brief end sensor register write
	* @param [in] devAddr Indicates the device address number 0-255
	* @param [in] regHAddr register address high 8 bits
	* @param [in] regLAddr Lower 8 bits of the register address
	* @param [in] regNum Number of registers 0-255
	* @param [in] data1 writes the register value 1
	* @param [in] data2 writes the register value 2
	* @param [in] isNoBlock 0- Block; 1- Non-blocking
	* @return  error code
	*/
	errno_t AxleSensorRegWrite(int devAddr, int regHAddr, int regLAddr, int regNum, int data1, int data2, int isNoBlock);

	/**
	* @brief sets whether the output is reset after the DO stop/pause of the control box
	* @param [in] resetFlag 0- no more bits; 1- Reset
	* @return  error code
	*/
	errno_t SetOutputResetCtlBoxDO(int resetFlag);

	 /**
	  * @brief sets whether the output is reset after the control box AO is stopped/paused
	  * @param [in] resetFlag 0- no more bits; 1- Reset
	  * @return  error code
	  */
		errno_t SetOutputResetCtlBoxAO(int resetFlag);

	 /**
	  * @brief Sets whether the output is reset after the end tool DO is stopped/paused
	  * @param [in] resetFlag 0- no more bits; 1- Reset
	  * @return  error code
	  */
		errno_t SetOutputResetAxleDO(int resetFlag);
	 
	 /**
	  * @brief sets whether the output is reset after the end tool AO is stopped/paused
	  * @param [in] resetFlag 0- no more bits; 1- Reset
	  * @return  error code
	  */
		errno_t SetOutputResetAxleAO(int resetFlag);
	
	 /**
	  * @brief Sets whether the output is reset after the extension DO is stopped/paused
	  * @param [in] resetFlag 0- no more bits; 1- Reset
	  * @return  error code
	  */
		errno_t SetOutputResetExtDO(int resetFlag);

	 /**
	  * @brief sets whether the output is reset after the extended AO is stopped/paused
	  * @param [in] resetFlag 0- no more bits; 1- Reset
	  * @return  error code
	  */
		errno_t SetOutputResetExtAO(int resetFlag);

	 /**
	  * @brief sets whether the output is reset after SmartTool stops/pauses
	  * @param [in] resetFlag 0- no more bits; 1- Reset
	  * @return  error code
	  */
		errno_t SetOutputResetSmartToolDO(int resetFlag);
	
	 /**
	  * @brief simulation swing starts
	  * @param [in] weaveNum Swing parameter number
	  * @return  error code
	  */
		errno_t WeaveStartSim(int weaveNum);

	 /**
	  * @brief simulation swing is over
	  * @param [in] weaveNum Swing parameter number
	  * @return  error code
	  */
		errno_t WeaveEndSim(int weaveNum);

	 /**
	  * @brief start trajectory detection warning (no movement)
	  * @param [in] weaveNum Swing parameter number
	  * @return  error code
	  */
		errno_t WeaveInspectStart(int weaveNum);

	 /**
	  * @brief end track detection warning (no movement)
	  * @param [in] weaveNum Swing parameter number
	  * @return  error code
	  */
		errno_t WeaveInspectEnd(int weaveNum);

	 /**
	  * @brief Extension IO- Configure welder gas detection signal
	  * @param [in] DONum Gas detection signal extension DO number
	  * @return  error code
	  */
		errno_t SetAirControlExtDoNum(int DONum);
	
	 /**
	  * @brief extension IO- Configs the arc signal of the welder
	  * @param [in] DONum welding machine arc signal extension DO number
	  * @return  error code
	  */
		errno_t SetArcStartExtDoNum(int DONum);

	 /**
	  * @brief extension IO- Configure the welder reverse wire feed signal
	  * @param [in] DONum reverse feed signal extension DO number
	  * @return  error code
	  */
		errno_t SetWireReverseFeedExtDoNum(int DONum);

	 /**
	  * @brief Extension IO- Configure the welder forward wire feed signal
	  * @param [in] DONum extends the DO number to the forward wire feed signal
	  * @return  error code
	  */
		errno_t SetWireForwardFeedExtDoNum(int DONum);

	 /**
	  * @brief extension IO- Configure the welder arc success signal
	  * @param [in] DINum Indicates the extension DI number of the arc-starting successful signal
	  * @return  error code
	  */
		errno_t SetArcDoneExtDiNum(int DINum);

	 /**
	  * @brief Extension IO- Configure the welder ready signal
	  * @param [in] DINum welder ready signal extension DI number
	  * @return  error code
	  */
		errno_t SetWeldReadyExtDiNum(int DINum);

	 /**
	  * @brief Extension IO- Configs the welding interrupt recovery signal
	  * @param [in] reWeldDINum Resume welding signal extension DI number after welding interruption
	  * @param [in] abortWeldDINum Indicates the DI number of a welding signal that exits after a welding failure
	  * @return  error code
	  */
	    errno_t SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum);

	 /**
	  * @brief sets the collision detection method of the robot
	  * @param [in] method Collision detection method: 0- current mode; 1- Dual encoder; 2- Current and dual encoder turn on simultaneously
	  * @param [in] thresholdMode Collision level threshold method 0-Collision level fixed threshold mode 1- Customize collision detection thresholds
	  * @return  error code
	  */
		errno_t SetCollisionDetectionMethod(int method, int thresholdMode = 0);

	 /**
	  * @brief Indicates that collision detection is disabled in static mode
	  * @param [in] status 0- Off; 1- Open
	  * @return  error code
	  */
		errno_t SetStaticCollisionOnOff(int status);

	 /**
	* @brief joint torque power detection
	* @param [in] status 0- Off; 1- Open
	* @param [in] power Set maximum power (W);
	  * @return  error code
	  */
	 errno_t SetPowerLimit(int status, double power);
	
	/**
	* @brief Joint torque control starts
	* @return  error code
	*/
	errno_t ServoJTStart();

	/**
	* @brief joint torque control
	* @param [in] torque j1 to j6 Joint torque, unit: Nm
	* @param [in] interval Instruction period, unit s, range [0.001~0.016]
	* @return error code
	*/
	errno_t ServoJT(float torque[], double interval);

	/**
	* @brief joint torque control
	* @param [in] torque j1 to j6 Joint torque, unit: Nm
	* @param [in] interval Instruction period, unit s, range [0.001~0.016]
	* @param [in] checkFlag Detection strategy: 0- no limit; 1- Power limit; 2- Speed limit; 3- Both power and speed are limited simultaneously
	* @param [in] jPowerLimit Maximum power limit of the joint(W)
	* @param [in] jVelLimit Maximum speed limit of the joint(°/s)
	* @return error code
	*/
	errno_t ServoJT(float torque[], double interval, int checkFlag, double jPowerLimit[6], double jVelLimit[6]);

	/**
	* @brief Joint torque control end
	* @return  error code
	*/
	errno_t ServoJTEnd();

	/**
	* @brief set the robot 20004 port feedback cycle
	* @param[in] period Robot 20004 Port Feedback Period(ms)
	* @return  error code
	*/
	errno_t SetRobotRealtimeStateSamplePeriod(int period);

	/**
	* @brief obtains feedback cycle of robot 20004 port
	* @param[out] period Robot 20004 Port Feedback Period(ms)
	* @return  error code
	*/
	errno_t GetRobotRealtimeStateSamplePeriod(int& period);

	/**
	* @brief get robot joint driver temperature(℃)
	* @return error code
	*/
	errno_t GetJointDriverTemperature(double temperature[]);

	/**
	* @brief get robot joint drive torque(Nm)
	* @return error code
	*/
	errno_t GetJointDriverTorque(double torque[]);

	/**
	* @brief Arc tracing + multi - layer compensation open
	* @return error code
	*/
	errno_t ArcWeldTraceReplayStart();

	/**
	* @brief Arc Tracking + multi - layer compensation off
	* @return error code
	*/
	errno_t ArcWeldTraceReplayEnd();

	/**
	* @brief offset coordinate change - multipass welding
	* @return error code
	*/
	errno_t MultilayerOffsetTrsfToBase(DescTran pointO, DescTran pointX, DescTran pointZ, double dx, double dy, double db, DescPose & offset);

	/**
	* @brief specifies attitude speed on
	* @param[in] ratio Attitude velocity percentage[0 - 300]
	* @return  error code
	*/
	errno_t AngularSpeedStart(int ratio);

	/**
	* @brief specifies attitude speed off
	* @return  error code
	*/
	errno_t AngularSpeedEnd();

	/**
	* @brief robot software upgrade
	* @param[in] filePath Full path of the software upgrade package
	* @param[in] block Whether to block until the upgrade is complete true: blocks. false : not blocking
	* @return  error code
	*/
	errno_t SoftwareUpgrade(std::string filePath, bool block);

	/**
	* @brief get the robot software upgrade status
	* @param[out] state Upgrade status of the robot software package(0 - Idle or uploading the upgrade package; 1 to 100: percentage of upgrade completion. - 1 : The software upgrade fails. - 2 : The verification fails. - 3 : Version verification fails. - 4 : The decompression fails. - 5 : The user configuration upgrade fails. - 6 : The peripheral configuration fails to be upgraded. - 7 : The expansion axis configuration fails to be upgraded. - 8 : The robot configuration fails to be upgraded. - 9 : Failed to configure DH parameters.
	* @return  error code
	*/
	errno_t GetSoftwareUpgradeState(int& state);

	/**
	* @brief sets the speed of the 485 expansion axis
	* @param[in] acc 485 expansion axis motion acceleration
	* @param[in] dec 485 Expansion axis motion deceleration
	* @return  error code
	*/
	errno_t AuxServoSetAcc(double acc, double dec);

	/**
	* @brief sets the speed of 485 expansion shaft to stop
	* @param[in] acc 485 expansion axis acceleration
	* @param[in] dec 485 Expansion shaft stopped and slowed down
	* @return  error code
	*/
	errno_t AuxServoSetEmergencyStopAcc(double acc, double dec);

	/**
	* @brief gets the speed of the 485 expansion axis movement
	* @param[out] acc 485 expansion axis acceleration
	* @param[out] dec 485 expansion axis motion deceleration
	* @return  error code
	*/
	errno_t AuxServoGetAcc(double& acc, double& dec);

	/**
	* @brief gets the speed of 485 expansion axis
	* @param[out] acc 485 expansion axis acceleration
	* @param[out] dec 485 expansion shaft stopped and slowed down
	* @return  error code
	*/
	errno_t AuxServoGetEmergencyStopAcc(double& acc, double& dec);

	/**
	 * @brief gets the axle communication parameters
	 * @param param axle communication parameters
	 * @return  error code
	 */
	errno_t GetAxleCommunicationParam(AxleComParam* param);

	/**
	 * @brief sets the axle communication parameters
	 * @param param  axle communication parameters
	 * @return  error code
	 */
	errno_t SetAxleCommunicationParam(AxleComParam param);

	/**
	 * @brief Set the end file transfer type
	 * @param type 1-MCU；2-LUA
	 * @return  error code
	 */
	errno_t SetAxleFileType(int type);

	/**
	 * @brief Set to enable axle LUA execution
	 * @param enable 0- Disable; 1- Enable
	 * @return  error code
	 */
	errno_t SetAxleLuaEnable(int enable);

	/**
	* @brief Axle LUA file error reset
	* @param status 0- No recovery; 1- Recovery
	* @return  error code
	*/
	errno_t SetRecoverAxleLuaErr(int status);

	/**
	* @brief Gets the axle LUA execution enable status
	* @param status status[0]: 0- Is not enabled. 1- Enabled
	* @return  error code
	*/
	errno_t GetAxleLuaEnableStatus(int status[]);

	/**
	* @brief Set the axle LUA end device enablement type
	* @param forceSensorEnable Force sensor Enabled status, 0- Disable. 1- Enable
	* @param gripperEnable Specifies whether the gripper is enabled. 0- Disables the gripper. 1- Enable
	* @param IOEnable IO Indicates whether the device is enabled. 0- Indicates that the device is disabled. 1- Enable
	* @return  error code
	*/
	errno_t SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable);

	/**
	* @brief Gets the axle LUA end device enabled type
	* @param enable enable[0]: indicates whether the forceSensorEnable force sensor is enabled. 0- Indicates whether the forcesenSOrenable force sensor is disabled. 1- Enable
	* @param enable enable[1]:gripperEnable Indicates whether the gripper is enabled. 0- Disables the gripper. 1- Enable
	* @param enable enable[2]:IOEnable I/o Indicates whether the device is enabled. 0- Indicates whether the device is disabled. 1- Enable
	* @return  error code
	*/
	errno_t GetAxleLuaEnableDeviceType(int* forceSensorEnable, int* gripperEnable, int* IOEnable);

	/**
	* @brief gets the currently configured end device
	*  @param forceSensorEnable Force sensor Enabled Device number 0- Not enabled; 1- Enable
	* @param gripperEnable Number of the gripperenable device, 0- Disable; 1- Enable
	* @param IODeviceEnable IODevice enable Device ID. 0- Disable. 1- Enable
	* @return  error code
	*/
	errno_t GetAxleLuaEnableDevice(int forceSensorEnable[], int gripperEnable[], int IODeviceEnable[]);

	/**
	* @brief setting enables the gripper action control function
	* @param id ID of the gripper device
	* @param func func[0]- gripper enabled; func[1]- gripper initialization; 2- Position setting; 3- Speed setting; 4- Torque setting; 6- Read the gripper status; 7- Read the initialization state; 8- Read the fault code; 9- Read position; 10- Read speed; 11- Read torque
	* @return  error code
	*/
	errno_t SetAxleLuaGripperFunc(int id, int func[]);

	/**
	* @brief obtains the enable gripper action control function
	* @param id ID of the gripper device
	* @param func func[0]- gripper enabled; func[1]- gripper initialization; 2- Position setting; 3- Speed setting; 4- Torque setting; 6- Read the gripper status; 7- Read the initialization state; 8- Read the fault code; 9- Read position; 10- Read speed; 11- Read torque
	* @return  error code
	*/
	errno_t GetAxleLuaGripperFunc(int id, int func[]);

	/**
	* @brief Sets the controller peripheral protocol LUA file name
	* @param id Indicates the protocol ID
	* @param name lua file name CTRL_LUA_test.lua
	* @return  error code
	*/
	errno_t SetCtrlOpenLUAName(int id, std::string name);

	/**
	* @brief obtains the LUA file name of the currently configured controller peripheral protocol
	* @param name Four lua file names CTRL_LUA_test.lua
	* @return  error code
	*/
	errno_t GetCtrlOpenLUAName(std::string name[]);

	/**
	* @brief loads the controller LUA protocol
	* @param id Indicates the LUA protocol number of the controller
	* @return  error code
	*/
	errno_t LoadCtrlOpenLUA(int id);

	/**
	* @brief Uninstalls the controller LUA protocol
	* @param id Indicates the LUA protocol number of the controller
	* @return  error code
	*/
	errno_t UnloadCtrlOpenLUA(int id);

	/**
	* @brief Sets the error code of the controller LUA protocol
	* @param id Indicates the LUA protocol number of the controller
	* @return  error code
	*/
	errno_t SetCtrlOpenLuaErrCode(int id, int code);

	/**
	* @brief robot Ethercat writes files from the slave station
	* @param type Slave file type, 1- Upgrade slave file; 2- Upgrade the slave configuration file
	* @param slaveID Secondary station ID
	* @param fileName File name of the upload file
	* @return  error code
	*/
	errno_t SlaveFileWrite(int type, int slaveID, std::string fileName);

	/**
	* @brief uploals the axle Lua open protocol file
	* @param filePath Local lua file pathname "... /AXLE_LUA_End_DaHuan.lua"
	* @return error code
	*/
	errno_t AxleLuaUpload(std::string filePath);

	/**
	* @brief robot Ethercat entered boot mode from the station
	* @return  error code
	*/
	errno_t SetSysServoBootMode();

	/**
	 * @brief Movable device enable
	 * @param enable false-disable；true-enable
	 * @return error code
	 */
	errno_t TractorEnable(bool enable);

	/**
	 * @brief Movable device homing
	 * @return error code
	 */
	errno_t TractorHoming();

	/**
	 * @brief movable device linear motion
	 * @param distance Linear motion distance (mm)
	 * @param vel Linear speed percentage (0-100)
	 * @return error code
	 */
	errno_t TractorMoveL(double distance, double vel);

	/**
	 * @brief Circular motion of movable device
	 * @param radio Circular Motion radius (mm)
	 * @param angle Angle of arc motion (°)
	 * @param vel Linear speed percentage (0-100)
	 * @return error code
	 */
	errno_t TractorMoveC(double radio, double angle, double vel);

	/**
	 * @brief The movable device stops moving
	 * @return error code
	 */
	errno_t TractorStop();

	/**
	 * @brief Sets the solder wire seeking expansion IO port
	 * @param searchDoneDINum DO port (0-127)
	 * @param searchStartDONum DO port for wire search start stop control (0-127)
	 * @return error code
	 */
	errno_t SetWireSearchExtDIONum(int searchDoneDINum, int searchStartDONum);

	/**
	 * @brief Set the welding machine control mode expansion DO port
	 * @param DONum Welding machine control mode DO port (0-127)
	 * @return error code
	 */
	errno_t SetWeldMachineCtrlModeExtDoNum(int DONum);

	/**
	 * @brief sets the welder control mode
	 * @param mode welding machine control mode; 0-Unified mode； 1-separate mode
	 * @return error code
	 */
	errno_t SetWeldMachineCtrlMode(int mode);

	/**
	* @brief Start singular pose protection
	* @param [in] protectMode Singular protection mode, 0: joint mode; 1- Cartesian model
	* @param [in] minShoulderPos Shoulder Singular adjustment range (mm), default 100
	* @param [in] minElbowPos Elbow singular adjustment range (mm), default 50
	* @param [in] minWristPos Wrist singular adjustment range (°), default 10
	* @return error code
	*/
	errno_t SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos);

	/**
	* @brief Stop singular pose protection
	* @return error code
	*/
	errno_t SingularAvoidEnd();

	/**
	* @brief Start Ptp motion FIR filtering
	* @param [in] maxAcc Maximum acceleration(deg/s2)
	* @param [in] maxJek Unify the extreme values of joint urgency (deg/s3)
	* @return error code
	*/
	errno_t PtpFIRPlanningStart(double maxAcc, double maxJek = 1000);

	/**
	* @brief Stop Ptp motion FIR filtering
	* @return error code
	*/
	errno_t PtpFIRPlanningEnd();

	/**
	* @brief Start LIN, ARC motion FIR filtering
	* @param [in] maxAccLin Extreme linear acceleration(mm/s2)
	* @param [in] maxAccDeg Extreme angular acceleration(deg/s2)
	* @param [in] maxJerkLin Extreme linear plus acceleration(mm/s3)
	* @param [in] maxJerkDeg Extreme angular plus acceleration(deg/s3)
	* @return error code
	*/
	errno_t LinArcFIRPlanningStart(double maxAccLin, double maxAccDeg, double maxJerkLin, double maxJerkDeg);

	/**
	* @brief Stop LIN, ARC motion FIR filtering
	* @return error code
	*/
	errno_t LinArcFIRPlanningEnd();

	/**
	 * @brief upload TrajectoryJ file
	 * @param [in] filePath file path   C://test/testJ.txt
	 * @return error code
	 */
	errno_t TrajectoryJUpLoad(const std::string& filePath);

	/**
	 * @brief delete TrajectoryJ file
	 * @param [in] fileName file name   testJ.txt
	 * @return error code
	 */
	errno_t TrajectoryJDelete(const std::string& fileName);

	/**
	 * @brief calculates the tool coordinate system based on the point information
	 * @param [in] method Calculation method; 0-four point method; One - six point method
	 * @param [in] pos joint position group, the array length is 4 in four-point method and 6 in six-point method
	 * @param [out] coord tool coordinate results
	 * @return Error code
	 */
	errno_t ComputeToolCoordWithPoints(int method, JointPos pos[], DescPose& coord);

	/**
	 * @brief calculates the workpiece coordinate system based on the point information
	 * @param [in] method Calculation method; 0: origin - X-axis - Z-axis 1: origin - X-axis -xy plane
	 * @param [in] pos Three TCP location groups
	 * @param [in] refFrame Reference coordinate system
	 * @param [out] coord tool coordinate results
	 * @return Error code
	 */
	errno_t ComputeWObjCoordWithPoints(int method, DescPose pos[], int refFrame, DescPose& coord);

	/**
	 * @brief sets the detection parameters of unexpected interruption of robot welding arc
	 * @param [in] checkEnable Whether the check is enabled. 0: Indicates that the function is disabled. 1- Enable
	 * @param [in] arcInterruptTimeLength Duration for confirming arc interruption (ms)
	 * @return Error code
	 */
	errno_t WeldingSetCheckArcInterruptionParam(int checkEnable, int arcInterruptTimeLength);

	/**
	 * @brief get the detection parameters of unexpected interruption of robot welding arc
	 * @param [out] checkEnable Whether the check is enabled. 0: Indicates that the function is disabled. 1- Enable
	 * @param [out] arcInterruptTimeLength Duration for confirming arc interruption (ms)
	 * @return Error code
	 */
	errno_t WeldingGetCheckArcInterruptionParam(int* checkEnable, int* arcInterruptTimeLength);

	/**
	 * @brief set the parameters of robot welding interruption recovery
	 * @param [in] enable Whether to enable welding interrupt recovery
	 * @param [in] length Weld overlap distance (mm)
	 * @param [in] velocity Percentage of velocity at which the robot returns to the rearcing point (0-100)
	 * @param [in] moveType Indicates how the robot moves to the rearcing point. 0-LIN; 1-PTP
	 * @return Error code
	 */
	errno_t WeldingSetReWeldAfterBreakOffParam(int enable, double length, double velocity, int moveType);

	/**
	 * @brief get robot welding interrupt recovery parameters
	 * @param [out] enable Whether to enable welding interrupt recovery
	 * @param [out] length Weld overlap distance (mm)
	 * @param [out] velocity Percentage of robot return to rearcing point (0-100)
	 * @param [out] moveType Indicates how the robot moves to the rearcing point. 0-LIN; 1-PTP
	 * @return Error code
	 */
	errno_t WeldingGetReWeldAfterBreakOffParam(int* enable, double* length, double* velocity, int* moveType);

	/**
	 * @brief sets the robot to resume welding after welding interruption
	 * @return Error code
	 */
	errno_t WeldingStartReWeldAfterBreakOff();

	/**
	 * @brief sets the robot to exit welding after welding interruption
	 * @return Error code
	 */
	errno_t WeldingAbortWeldAfterBreakOff();

	/**
	 * @brief laser track record
	 * @param [in] enable Whether to enable welding interrupt recovery
	 * @param [in] length Weld overlap distance (mm)
	 * @param [in] velocity Percentage of velocity at which the robot returns to the rearcing point (0-100)
	 * @param [in] moveType Indicates how the robot moves to the rearcing point. 0-LIN; 1-PTP
	 * @return Error code
	 */
	errno_t LaserSensorRecord(int status, int delayMode, int delayTime, int delayDisExAxisNum, double delayDis, double sensitivePara, double speed);

	errno_t LaserTrackingLaserOn(int weldId);

	errno_t LaserTrackingLaserOff();

	errno_t LaserTrackingTrackOn(int coordId);

	errno_t LaserTrackingTrackOff();

	errno_t LaserTrackingSearchStart(int direction, DescTran directionPoint, int vel, int distance, int timeout, int posSensorNum);


	/**
	 * @brief Laser device on/off function
	 * @param [in] OnOff 0-Off 1-On
	 * @param [in] weldId Weld seam ID, default is 0
	 * @return Error code
	 */
	errno_t LaserTrackingLaserOnOff(int OnOff, int weldId);

	/**
	 * @brief Laser tracking start/stop function
	 * @param [in] OnOff 0-Stop 1-Start
	 * @param [in] coordId Tool coordinate system number of the laser device
	 * @return Error code
	 */
	errno_t LaserTrackingTrackOnOff(int OnOff, int coordId);

	/**
	 * @brief Laser search - Fixed direction
	 * @param [in] direction 0-X+ 1-X- 2-Y+ 3-Y- 4-Z+ 5-Z-
	 * @param [in] vel Velocity in %
	 * @param [in] distance Maximum search distance in mm
	 * @param [in] timeout Search timeout period in ms
	 * @param [in] posSensorNum Tool coordinate number calibrated by the laser
	 * @return Error code
	 */
	errno_t LaserTrackingSearchStart_xyz(int direction, int vel, int distance, int timeout, int posSensorNum);

	/**
	 * @brief Laser search - Arbitrary direction
	 * @param [in] directionPoint XYZ coordinates of the point input for search
	 * @param [in] vel Velocity in %
	 * @param [in] distance Maximum search distance in mm
	 * @param [in] timeout Search timeout period in ms
	 * @param [in] posSensorNum Tool coordinate number calibrated by the laser
	 * @return Error code
	 */
	errno_t LaserTrackingSearchStart_point(DescTran directionPoint, int vel, int distance, int timeout, int posSensorNum);

	/**
	 * @brief Stop laser search
	 * @return Error code
	 */
	errno_t LaserTrackingSearchStop();

	/**
	 * @brief Configure laser network parameters
	 * @param [in] ip IP address of the laser device
	 * @param [in] port Port number of the laser device
	 * @return Error code
	 */
	errno_t LaserTrackingSensorConfig(std::string ip, int port);

	/**
	 * @brief Configure laser device sampling period
	 * @param [in] period Sampling period of the laser device in ms
	 * @return Error code
	 */
	errno_t LaserTrackingSensorSamplePeriod(int period);

	/**
	 * @brief Load laser device driver
	 * @param [in] type Protocol type of the laser device driver: 101-Ruineng 102-Chuangxiang 103-Quanshi 104-Tongzhou 105-Aotai
	 * @return Error code
	 */
	errno_t LoadPosSensorDriver(int type);

	/**
	 * @brief Unload laser device driver
	 * @return Error code
	 */
	errno_t UnLoadPosSensorDriver();

	/**
	 * @brief Record laser weld seam trajectory
	 * @param [in] status 0-Stop recording 1-Real-time tracking 2-Start recording
	 * @param [in] delayTime Delay time in ms
	 * @return Error code
	 */
	errno_t LaserSensorRecord1(int status, int delayTime);

	/**
	 * @brief Replay laser weld seam trajectory
	 * @param [in] delayTime Delay time in ms
	 * @param [in] speed Velocity in %
	 * @return Error code
	 */
	errno_t LaserSensorReplay(int delayTime, double speed);


	/**
	 * @brief Laser tracking replay
	 * @return Error code
	 */
	errno_t MoveLTR();

	/**
	 * @brief Record and replay laser weld seam trajectory
	 * @param [in] delayMode Mode: 0-Delay time 1-Delay distance
	 * @param [in] delayTime Delay time in ms
	 * @param [in] delayDisExAxisNum Extended axis number
	 * @param [in] delayDis Delay distance in mm
	 * @param [in] sensitivePara Compensation sensitivity coefficient
	 * @param [in] speed Velocity in %
	 * @return Error code
	 */
	errno_t LaserSensorRecordandReplay(int delayMode, int delayTime, int delayDisExAxisNum, double delayDis, double sensitivePara, double speed);

	/**
	 * @brief Move to the starting point of the laser record
	 * @param [in] moveType 0-moveJ 1-moveL
	 * @param [in] ovl Velocity in %
	 * @return Error code
	 */
	errno_t MoveToLaserRecordStart(int moveType, double ovl);

	/**
	 * @brief Move to the endpoint of the laser record
	 * @param [in] moveType 0-moveJ 1-moveL
	 * @param [in] ovl Velocity in %
	 * @return Error code
	 */
	errno_t MoveToLaserRecordEnd(int moveType, double ovl);


	/**
	 * @brief Move to the laser sensor search position
	 * @param [in] moveFlag Motion type: 0-PTP; 1-LIN
	 * @param [in] ovl Velocity scaling factor, 0-100
	 * @param [in] dataFlag Weld seam buffer data selection: 0-Execute planning data; 1-Execute recorded data
	 * @param [in] plateType Plate type: 0-Corrugated plate; 1-Corrugated cardboard; 2-Fence plate; 3-Oil drum; 4-Corrugated shell steel
	 * @param [in] trackOffectType Laser sensor offset type: 0-No offset; 1-Base coordinate system offset; 2-Tool coordinate system offset; 3-Laser sensor raw data offset
	 * @param [in] offset Offset value
	 * @return Error code
	 */
	errno_t MoveToLaserSeamPos(int moveFlag, double ovl, int dataFlag, int plateType, int trackOffectType, DescPose offset);

	/**
	 * @brief Get the coordinate information of the laser sensor search position
	 * @param [in] trackOffectType Laser sensor offset type: 0-No offset; 1-Base coordinate system offset; 2-Tool coordinate system offset; 3-Laser sensor raw data offset
	 * @param [in] offset Offset value
	 * @param [out] jPos Joint position [°]
	 * @param [out] descPos Cartesian position [mm]
	 * @param [out] tool Tool coordinate system
	 * @param [out] user User coordinate system
	 * @param [out] exaxis Extended axis position [mm]
	 * @return Error code
	 */
	errno_t GetLaserSeamPos(int trackOffectType, DescPose offset, JointPos& jPos, DescPose& descPos, int& tool, int& user, ExaxisPos& exaxis);

	/**
	 * @brief Wobble gradient begins
	 * @param [in] weaveChangeFlag 1- Variable swing parameters; 2- Variable swing parameters + welding speed
	 * @param [in] weaveNum swing number
	 * @param [in] velStart welding start speed, (cm/min)
	 * @param [in] velEnd welding end speed, (cm/min)
	 * @return Error code
	 */
	errno_t WeaveChangeStart(int weaveChangeFlag, int weaveNum, double velStart, double velEnd);

	/**
	 * @brief swing gradient ends
	 * @return Error code
	 */
	errno_t WeaveChangeEnd();

   /**
	* @brief trajectory Preprocessing (trajectory Foresight)
	* @param [in] name Indicates the track file name
	* @param [in] mode Sampling mode. 0- Sampling is not performed. 1- equal data interval sampling; 2- Equal error limit sampling
	* @param [in] errorLim Error limit, using line fitting takes effect
	* @param [in] type Indicates the smoothing mode, 0-Bessel smoothing
	* @param [in] precision Smoothing precision. This parameter takes effect when Bezier smoothing is used
	* @param [in] vamx set maximum speed, mm/s
	* @param [in] Maximum acceleration set by amax, mm/s2
	* @param [in] Max acceleration set by jmax, mm/s3
	* @param [in] flag Constant speed forward-looking open switch 0-not open; 1-Open
	* @return Error code
	*/
	errno_t LoadTrajectoryLA(char name[30], int mode, double errorLim, int type, double precision, double vamx, double amax, double jmax, int flag = 0);

	/**
	* @brief trajectory reproduction(trajectory Foresight)
	* @return Error code
	*/
	errno_t MoveTrajectoryLA();

	/**
	 * @brief custom collision detection threshold function starts, set the collision detection thresholds of the joint end and TCP end
	 * @param[in] flag 1 - Only joint detection is enabled; 2 - Only TCP detection is enabled. 3 - Joint and TCP detection are enabled simultaneously
	 * @param[in] jointDetectionThreshould Joint Collision Detection threshold j1 - j6
	 * @param[in] tcpDetectionThreshould TCP collision detection threshold, xyzabc
	 * @param[in] block 0 - non blocking; 1 - block
	 * @return Error code
	 */
	errno_t CustomCollisionDetectionStart(int flag, double jointDetectionThreshould[6], double tcpDetectionThreshould[6], int block);

	/**
	 * @brief custom collision detection threshold function ends
	 * @return Error code
	 */
	errno_t CustomCollisionDetectionEnd();

	/**
	 * @brief  Acceleration smooth on
	 * @param  [in] saveFlag Power-off save or not
	 * @return  Error code
	 */
	errno_t AccSmoothStart(bool saveFlag);

	/**
	 * @brief  Acceleration smooth off
	 * @param  [in] saveFlag Power-off save or not
	 * @return  Error code
	 */
	errno_t AccSmoothEnd(bool saveFlag);

	/**
	 * @brief Download controller log 
	 * @param [in] savePath Save file path "D://zDown/"
	 * @return Error code
	 */
	errno_t RbLogDownload(std::string savePath);

	/**
	 * @brief Download all data sources
	 * @param [in] savePath Save file path "D://zDown/"
	 * @return Error code
	 */
	errno_t AllDataSourceDownload(std::string savePath);

	/**
	 * @brief Download data backup package 
	 * @param [in] savePath Save file path "D://zDown/"
	 * @return Error code
	 */
	errno_t DataPackageDownload(std::string savePath);

	/**
	 * @brief Get the SN code of the control box
	 * @param [out] SNCode SN of the control box
	 * @return Error code
	 */
	errno_t GetRobotSN(std::string& SNCode);

	/**
	 * @brief Shut down the robot operating system
	 * @return Error code
	 */
	errno_t ShutDownRobotOS();

	/**
	 * @brief Conveyor communication input detection
	 * @param [in] timeout Waiting timeout(ms)
	 * @return Error code
	 */
	errno_t ConveyorComDetect(int timeout);

	/**
	 * @brief Conveyor communication input detection triggered
	 * @return Error code
	 */
	errno_t ConveyorComDetectTrigger();

	/**
	 * @brief Selection of AI channels for current feedback in arc tracking
	 * @param [in]  channel channels；0-Aux AI0；1-Aux AI1；2-Aux AI2；3-Aux AI3；4-Control Box AI0；5-Control Box AI1
	 * @return Error code
	 */
	errno_t ArcWeldTraceAIChannelCurrent(int channel);

	/**
	 * @brief Selection of AI channels for voltage feedback in arc tracking
	 * @param [in]  channel channels；0-Aux AI0；1-Aux AI1；2-Aux AI2；3-Aux AI3；4-Control Box AI0；5-Control Box AI1
	 * @return Error code
	 */
	errno_t ArcWeldTraceAIChannelVoltage(int channel);

	/**
	 * @brief Current feedback conversion parameters of arc tracking
     * @param [in] AILow AI channel lower limit, default value 0V, range [0-10V]
     * @param [in] AIHigh AI channel upper limit, default value 10V, range [0-10V]
     * @param [in] The lower limit of the currentLow AI channel corresponds to the current value of the welding machine. The default value is 0V, and the range is [0-200V]
	 * @param [in] The upper limit of the currentLow AI channel corresponds to the current value of the welding machine. The default value is 0V, and the range is [0-200V]
	 * @return Error code
	 */
	errno_t ArcWeldTraceCurrentPara(float AILow, float AIHigh, float currentLow, float currentHigh);

	/**
	 * @brief Voltage feedback Conversion Parameters of Arc Tracking Welding machine
	 * @param [in] AILow AI channel lower limit, default value 0V, range [0-10V]
	 * @param [in] AIHigh AI channel upper limit, default value 10V, range [0-10V]
	 * @param [in] The lower limit of the voltageLow AI channel corresponds to the welding machine voltage value. The default value is 0V, and the range is [0-200V]
	 * @param [in] The upper limit of the voltageHigh AI channel corresponds to the voltage value of the welding machine. The default value is 100V, and the range is [0-200V]
	 * @return Error code
	 */
	errno_t ArcWeldTraceVoltagePara(float AILow, float AIHigh, float voltageLow, float voltageHigh);

	/**
	 * @brief Set the welding voltage to start gradually
	 * @param [in] IOType control type; 0- Control Box IO 1- Digital Communication Protocol (UDP) 2- Digital Communication Protocol (ModbusTCP)
	 * @param [in] voltageStart Initial Welding Voltage (V)
	 * @param [in] voltageEnd Stop welding Voltage (V)
	 * @param [in] AOIndex control box AO port number (0-1)
	 * @param [in] Is blend smooth? 0- Not smooth; 1- Smooth
     * @return Error code
	 */
	errno_t WeldingSetVoltageGradualChangeStart(int IOType, double voltageStart, double voltageEnd, int AOIndex, int blend);

	/**
	 * @brief Set the welding voltage gradient to end
	 * @return Error code
	 */
	errno_t WeldingSetVoltageGradualChangeEnd();

	/**
	 * @brief Set the welding current to start gradually
	 * @param [in] IOType control type; 0- Control Box IO 1- Digital Communication Protocol (UDP) 2- Digital Communication Protocol (ModbusTCP)
	 * @param [in] voltageStart Initial welding Current (A)
	 * @param [in] voltageEnd Stop welding current (A)
	 * @param [in] AOIndex control box AO port number (0-1)
	 * @param [in] Is blend smooth? 0- Not smooth; 1- Smooth
	 * @return Error code
	 */
	errno_t WeldingSetCurrentGradualChangeStart(int IOType, double currentStart, double currentEnd, int AOIndex, int blend);

	/**
	 * @brief Set the welding current gradient to end
	 * @return Error code
	 */
	errno_t WeldingSetCurrentGradualChangeEnd();

	/**
	* @brief Get the status of the SmartTool button
    * @param [out] state SmartTool handle button status; (bit0:0- Communication is normal; 1- Communication disconnection; bit1- Undo; bit2- Clear the program;
      bit3-A key; bit4-B key; bit5-C key; bit6-D key; bit7-E key; bit8-IO key; bit9- Manual automatic; bit10-Start
    * @return error code
    */
	errno_t GetSmarttoolBtnState(int& state);

	/**
	 * @brief Get the extended axis coordinate system
	 * @param [out] coord extended axis coordinate system
	 * @return error code
	 */
	errno_t ExtAxisGetCoord(DescPose& coord);

	/**
	 * @brief Set the monitoring parameters for the temperature and fan current of the wide-voltage control box
	 * @param [in] enable 0-not enable; 1-Enable monitoring
	 * @param [in] period Monitoring period (unit: s), range 1-100
	 * @return error code
	 */
	errno_t SetWideBoxTempFanMonitorParam(int enable, int period);

	/**
	 * @brief Get the monitoring parameters for the temperature and fan current of the wide-voltage control box
	 * @param [out] enable enable 0-not enable; 1-Enable monitoring
	 * @param [out] period period Monitoring period (unit: s), range 1-100
	 * @return error code
	 */
	errno_t GetWideBoxTempFanMonitorParam(int& enable, int& period);

	/**
	 * @brief Set the focus calibration point
	 * @param [in] pointNum Focus calibration point number 1-8
	 * @param [in] point Coordinate of the calibration point
	 * @return error code
	 */
	errno_t SetFocusCalibPoint(int pointNum, DescPose point);

	/**
	 * @brief Calculate the focus calibration result
	 * @param [in] pointNum The number of calibration points
	 * @param [out] resultPos Calibration result XYZ
	 * @param [out] accuracy Calibration accuracy error
	 * @return error code
	 */
	errno_t ComputeFocusCalib(int pointNum, DescTran& resultPos, float& accuracy);

	/**
	 * @brief Enable focus following
	 * @param [in] kp Proportional parameter, default50.0
	 * @param [in] kpredict Feedforward parameter, default 19.0
	 * @param [in] aMax Maximum angular acceleration limit, default 1440°/s^2
	 * @param [in] vMax Maximum angular velocity limit, default 180°/s
	 * @param [in] type Lock the X-axis pointing (0- reference input vector; 1-level; 2- Vertical
	 * @return error code
	 */
	errno_t FocusStart(double kp, double kpredict, double aMax, double vMax, int type);

	/**
	 * @brief Stop focusing following
	 * @return error code
	 */
	errno_t FocusEnd();

	/**
	 * @brief Set the focus coordinates
	 * @param [in] pos Focal coordinate XYZ
	 * @return error code
	 */
	errno_t SetFocusPosition(DescTran pos);

	/**
	 * @brief Set the encoder upgrade
	 * @param [in] path Full path of local upgrade package(D://zUP/XXXXX.bin)
	 * @return error code
	 */
	errno_t SetEncoderUpgrade(std::string path);

	/**
	 * @brief Set the joint firmware upgrade
	 * @param [in] type Upgrade file type; 1- Upgrade the firmware (the robot needs to be put into boot mode before use); 2- Upgrade the slave station configuration file (Disable the robot before use)
	 * @param [in] path Full path of local upgrade package(D://zUP/XXXXX.bin)
	 * @return error code
	 */
	errno_t SetJointFirmwareUpgrade(int type, std::string path);

	/**
	 * @brief Set the control box firmware upgrade
	 * @param [in] type Upgrade file type; 1- Upgrade the firmware (the robot needs to be put into boot mode before use); 2- Upgrade the slave station configuration file (Disable the robot before use)
	 * @param [in] path Full path of local upgrade package(D://zUP/XXXXX.bin)
	 * @return error code
	 */
	errno_t SetCtrlFirmwareUpgrade(int type, std::string path);

	/**
	 * @brief Set robot end firmware upgrade
	 * @param [in] type Upgrade file type; 1- Upgrade the firmware (the robot needs to be put into boot mode before use); 2- Upgrade the slave station configuration file (Disable the robot before use)
	 * @param [in] path path Full path of local upgrade package(D://zUP/XXXXX.bin)
	 * @return error code
	 */
	errno_t SetEndFirmwareUpgrade(int type, std::string path);

	/**
	 * @brief Joint full parameter profile upgrade(Disable the robot before use)
	 * @param [in] path Full path of local upgrade package(D://zUP/XXXXX.db)
	 * @return error code
	 */
	errno_t JointAllParamUpgrade(std::string path);

	/**
	 * @brief Set the type of the robot(Disable the robot before use)
	 * @param [in] type Robot type
	 * @return error code
	 */
	errno_t SetRobotType(int type);

	/**
	 * @brief Laser sensor recording points
	 * @param [in] coordID Laser sensor coordinate system ID
	 * @param [out] desc Laser sensor identification point Descartes
	 * @param [out] joint Laser sensor identification joint position
	 * @param [out] exaxis Laser sensor identification point spread shaft position
	 * @return error code
	 */
	errno_t LaserRecordPoint(int coordID, DescPose& desc, JointPos& joint, ExaxisPos& exaxis);

	/**
	 * @brief Set the expansion axis and the robot synchronous motion strategy
	 * @param [in] strategy Strategy; 0- Mainly robots; 1- The extended axis is synchronized with the robot
	 * @return error code
	 */
	errno_t SetExAxisRobotPlan(int strategy);

	/**
	* @brief  Set communication reconnection parameters with the robot
	* @param [in] enable Enable reconnection when the network is faulty true- enabled false- disabled
	* @param [in] reconnectTime Reconnection time, unit: ms
	* @param [in] period Reconnection period, expressed in ms
	* @return  error code
	*/
	errno_t SetReConnectParam(bool enable, int reconnectTime = 30000, int period = 50);

	/**
	 * @brief  Get slave board parameters
	 * @param  [out] type  0-Ethercat，1-CClink, 3-Ethercat, 4-EIP
	 * @param  [out] version  Protocol version
	 * @param  [out] connState  0-Disconnected, 1-Connected
	 * @return  Error code
	 */
	errno_t GetFieldBusConfig(uint8_t* type, uint8_t* version, uint8_t* connState);

	/**
	 * @brief  Write slave DO (Digital Output)
	 * @param  [in] DOIndex  DO number
	 * @param  [in] wirteNum  Number of values to write
	 * @param  [in] status[8] Values to write (max 8)
	 * @return  Error code
	 */
	errno_t FieldBusSlaveWriteDO(uint8_t DOIndex, uint8_t wirteNum, uint8_t status[8]);

	/**
	 * @brief  Write slave AO (Analog Output)
	 * @param  [in] AOIndex  AO number
	 * @param  [in] wirteNum  Number of values to write
	 * @param  [in] status[8] Values to write (max 8)
	 * @return  Error code
	 */
	errno_t FieldBusSlaveWriteAO(uint8_t AOIndex, uint8_t wirteNum, int status[8]);

	/**
	 * @brief  Read slave DI (Digital Input)
	 * @param  [in] DOIndex  DI number
	 * @param  [in] readeNum  Number of values to read
	 * @param  [out] status[8] Read values (max 8)
	 * @return  Error code
	 */
	errno_t FieldBusSlaveReadDI(uint8_t DOIndex, uint8_t readNum, uint8_t status[8]);

	/**
	 * @brief  Read slave AI (Analog Input)
	 * @param  [in] AOIndex  AI number
	 * @param  [in] readeNum  Number of values to read
	 * @param  [out] status[8] Read values (max 8)
	 * @return  Error code
	 */
	errno_t FieldBusSlaveReadAI(uint8_t AIIndex, uint8_t readNum, int status[8]);

	/**
	 * @brief Wait for extended DI input
	 * @param [in] DIIndex DI number
	 * @param [in] status 0-Low level; 1-High level
	 * @param [in] waitMs Maximum wait time (ms)
	 * @return Error code
	 */
	errno_t FieldBusSlaveWaitDI(uint8_t DIIndex, bool status, int waitMs);

	/**
	 * @brief Wait for extended AI input
	 * @param [in] AIIndex AI number
	 * @param [in] waitType 0-Greater than; 1-Less than
	 * @param [in] value AI threshold value
	 * @param [in] waitMs Maximum wait time (ms)
	 * @return Error code
	 */
	errno_t FieldBusSlaveWaitAI(uint8_t AIIndex, uint8_t waitType, double value, int waitMs);

	/**
	 * @brief Control array-type suction cups
	 * @param [in] slaveID Slave station ID
	 * @param [in] len Data length
	 * @param [in] ctrlValue Control value array (size 20)
	 * @return Error code
	 */
	errno_t SetSuckerCtrl(uint8_t slaveID, uint8_t len, uint8_t ctrlValue[20]);

	/**
	 * @brief Get array-type suction cup status
	 * @param [in] slaveID Slave station ID
	 * @param [out] state Adsorption state:
	 *        0-Object released,
	 *        1-Workpiece detected (adsorption success),
	 *        2-No workpiece detected,
	 *        3-Object detached
	 * @param [out] pressValue Current vacuum (unit: kPa)
	 * @param [out] error Current error code
	 * @return Error code
	 */
	errno_t GetSuckerState(uint8_t slaveID, uint8_t* state, int* pressValue, int* error);

	/**
	 * @brief Wait for suction cup state
	 * @param [in] slaveID Slave station ID
	 * @param [in] state Target adsorption state:
	 *        0-Object released,
	 *        1-Workpiece detected,
	 *        2-No workpiece detected,
	 *        3-Object detached
	 * @param [in] ms Maximum wait time (ms)
	 * @return Error code
	 */
	errno_t WaitSuckerState(uint8_t slaveID, uint8_t state, int ms);

	/**
	 * @brief Upload Lua file
	 * @param [in] filePath local openlua file path name
	 * @return error code
	 */
	errno_t OpenLuaUpload(std::string filePath);

	/**
	* @brief Impedance Control
	* @param [in] status 0：OFF；1-ON
	* @param [in] workSpace 0-joint space;1 -Dicard space
	* @param [in] forceThreshold Trigger force threshold (N)
	* @param [in] m Quality parameters
	* @param [in] b Damping parameter
	* @param [in] k Stiffness parameter
	* @param [in] maxV Maximum linear velocity(mm/s)
	* @param [in] maxVA Maximum linear acceleration(mm/s2)
	* @param [in] maxW Maximum angular velocity(°/s)
	* @param [in] maxWA Maximum angular acceleration(°/s2)
	* @return Error code
	*/
	errno_t ImpedanceControlStartStop(int status, int workSpace, double forceThreshold[6], double m[6], double b[6], double k[6], double maxV, double maxVA, double maxW, double maxWA);

	/**
	 * @brief Set check the load state before starting drag flag
	 * @param [in] flag 0-OFF;1-ON
	 * @return Error code
	 */
	errno_t SetTorqueDetectionSwitch(uint8_t flag);

	/**
	* @brief Get the tool coordinate system according to the No
	* @param [in] index Tool coordinate system No
	* @param [out] coord Coordinate system value
	* @return Error code
	 */
	errno_t GetToolCoordWithID(int id, DescPose& coord);

	/**
	* @brief Get the workpiece coordinate system according to the No
	* @param [in] index Workpiece coordinate system No
	* @param [out] coord Coordinate system value
	* @return Error code
	*/
	errno_t GetWObjCoordWithID(int id, DescPose& coord);

	/**
	* @brief Get the external tool coordinate system according to the No
	* @param [in] index External tool coordinate system No
	* @param [out] coord Coordinate system value
	 * @return Error code
	 */
	errno_t GetExToolCoordWithID(int id, DescPose& coord);

	/**
	* @brief Get the extended axis coordinate system according to the No
	* @param [in] index Extended axis coordinate system No
	* @param [out] coord Coordinate system value
	* @return Error code
	*/
	errno_t GetExAxisCoordWithID(int id, DescPose& coord);

	/**
	* @brief Obtain the load mass and center of mass based on the number
	* @param [in] index load ID
	* @param [out] weight load Weight
	* @param [out] cog load centroid
	* @return Error code
	*/
	errno_t GetTargetPayloadWithID(int id, double& weight, DescTran& cog);
	/**
	* @brief Obtain the current tool coordinate system
	* @param [out] coord coord Coordinate system value
	* @return Error code
	*/
	errno_t GetCurToolCoord(DescPose& coord);

	/**
	* @brief Gets the current workpiece coordinate system
	* @param [out] coord coord Coordinate system value
	* @return Error code
	*/
	errno_t GetCurWObjCoord(DescPose& coord);

	/**
	* @brief Gets the current external tool coordinate system
	* @param [out] coord coord Coordinate system value
	* @return Error code
	*/
	errno_t GetCurExToolCoord(DescPose& coord);

	/**
	* @brief Gets the current extended axis coordinate system
	* @param [out] coord coord Coordinate system value
	* @return Error code
	*/
	errno_t GetCurExAxisCoord(DescPose& coord);

	/**
	 * @brief Robot Operating System Upgrade (LA Control Box)
	 * @param [in] filePath The full path of the operating system upgrade package
	 * @return Error code
	 */
	errno_t KernelUpgrade(std::string filePath);

	/**
	 * @brief Obtain the upgrade result of the robot operating system (LA control box)
	 * @param [out] result Upgrade result: 0: Success; -1: Failure
	 * @return  Error code
	 */
	errno_t GetKernelUpgradeResult(int& result);

	/**
	 * @brief Set custom weave parameters
	 * @param [in] id custom weave ID：0-2
	 * @param [in] pointNum Number of weave points 0-10
	 * @param [in] point Moving endpoint data x,y,z
	 * @param [in] stayTime weave residence time ms
	 * @param [in] frequency weave frequency Hz
	 * @param [in] incStayType Waiting mode: 0- Cycle does not include waiting time; 1- The cycle includes waiting time
	 * @param [in] stationary weave position waiting: 0- Continue to move within the waiting time; The position remains stationary during the waiting time
	 * @return Error code
	 */
	errno_t CustomWeaveSetPara(int id, int pointNum, DescTran point[10], double stayTime[10], double frequency, int incStayType, int stationary);

	/**
	 * @brief Gets custom swing parameters
	 * @param [in] id custom weave ID：0-2
	 * @param [out] pointNum Number of weave points 0-10
	 * @param [out] point Moving endpoint data x,y,z
	 * @param [out] stayTime weave residence time ms
	 * @param [out] frequency weave frequency Hz
	 * @param [out] incStayType Waiting mode: 0- Cycle does not include waiting time; 1- The cycle includes waiting time
	 * @param [out] stationary weave position waiting: 0- Continue to move within the waiting time; The position remains stationary during the waiting time
	 * @return  Error code
	 */
	errno_t CustomWeaveGetPara(int id, int& pointNum, DescTran point[10], double stayTime[10], double& frequency, int& incStayType, int& stationary);

	/**
	 * @brief Enable joint torque sensor sensitivity calibration function
	 * @param [in] status 0-Disable；1-Enable
	 * @return Error code
	 */
	errno_t JointSensitivityEnable(int status);

	/**
	 * @brief Get the sensitivity calibration results of the joint torque sensor
	 * @param [out] calibResult j1~j6 Joint sensitivity [0-1]
	 * @param [out] linearityn j1~j6 Joint linearity[0-1]
	 * @return Error code
	 */
	errno_t JointSensitivityCalibration(double result[6], double linearity[6]);

	/**
	 * @brief Sensitivity data acquisition of joint torque sensors
	 * @return Error code
	 */
	errno_t JointSensitivityCollect();

	errno_t Sleep(int ms);

	/**
	 * @brief Clear the motion command queue
	 * @return Error code
	 */
	errno_t MotionQueueClear();

	/**
	 * @brief Get the number of 8 slave port error frames of the robot
	 * @param [out] inRecvErr Input receiving error frames
	 * @param [out] inCRCErr Input CRC error frames
	 * @param [out] inTransmitErr Input transmit error frames
	 * @param [out] inLinkErr Input link error frames
	 * @param [out] outRecvErr Output receiving error frames
	 * @param [out] outCRCErr Output CRC error frames
	 * @param [out] outTransmitErr Output transmit error frames
	 * @param [out] outLinkErr Output link error frames
	 * @return Error code
	 */
	errno_t GetSlavePortErrCounter(int inRecvErr[8], int inCRCErr[8], int inTransmitErr[8], int inLinkErr[8],
		int outRecvErr[8], int outCRCErr[8], int outTransmitErr[8], int outLinkErr[8]);

	/**
	 * @brief Clear the slave port error num
	 * @param [in] slaveID slave id 0~7
	 * @return Error code
	 */
	errno_t SlavePortErrCounterClear(int slaveID);

	/**
	 * @brief Set the feedforward coefficients of the velocities of each axis
	 * @param [in] radio feedforward coefficients of the velocities of each axis
	 * @return Error code
	 */
	errno_t SetVelFeedForwardRatio(double radio[6]);

	/**
	 * @brief Get the feedforward coefficients of the velocities of each axis
	 * @param [out] radio feedforward coefficients of the velocities of each axis
	 * @return Error code
	 */
	errno_t GetVelFeedForwardRatio(double radio[6]);

	/**
	 * @brief Robot MCU log generation
	 * @return Error code
	 */
	errno_t RobotMCULogCollect();

	/**
	 * @brief Move to the starting point of the intersection line
	 * @param [in] mainPoint Cartesian poses of the six teaching points of the main pipeline
	 * @param [in] piecePoint Cartesian poses of the six teaching points of the auxiliary pipeline
	 * @param [in] tool Tool coordinate system ID
	 * @param [in] wobj Workpiece coordinate system ID
	 * @param [in] vel Velocity percentage
	 * @param [in] acc Acceleration percentage
	 * @param [in] ovl Velocity scaling factor
	 * @param [in] oacc Acceleration scaling factor
	 * @param [in] moveType Movement type; 0-PTP；1-LIN
	 * @return Error code
	 */
	errno_t MoveToIntersectLineStart(DescPose mainPoint[6], DescPose piecePoint[6], int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveType);

	/**
	 * @brief Move to the starting point of the intersection line
	 * @param [in] mainPoint Cartesian poses of the six teaching points of the main pipeline
	 * @param [in] mainExaxisPos Exaxis poses of the six teaching points of the main pipeline
	 * @param [in] piecePoint Cartesian poses of the six teaching points of the auxiliary pipeline
	 * @param [in] pieceExaxisPos Exaxis poses of the six teaching points of the auxiliary pipeline
	 * @param [in] extAxisFlag Whether to enable the extended axis; 0- Not enabled; 1- Enable
	 * @param [in] exaxisPos The position of the starting expansion axis
	 * @param [in] tool Tool coordinate system ID
	 * @param [in] wobj Workpiece coordinate system ID
	 * @param [in] vel Velocity percentage
	 * @param [in] acc Acceleration percentage
	 * @param [in] ovl Velocity scaling factor
	 * @param [in] oacc Acceleration scaling factor
	 * @param [in] moveType Movement type; 0-PTP；1-LIN
	 * @param [in] moveDirection Direction of movement; 0- clockwise; 1- Counterclockwise
	 * @param [in] offset Offset Descartes
	 * @return Error code
	 */
	errno_t MoveToIntersectLineStart(DescPose mainPoint[6], ExaxisPos mainExaxisPos[6], DescPose piecePoint[6], ExaxisPos pieceExaxisPos[6], int extAxisFlag, ExaxisPos exaxisPos, int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveType, int moveDirection, DescPose offset);

	/**
	 * @brief Intersection line movement
	 * @param [in] mainPoint Cartesian poses of the six teaching points of the main pipeline
	 * @param [in] piecePoint Cartesian poses of the six teaching points of the auxiliary pipeline
	 * @param [in] tool Tool coordinate system ID
	 * @param [in] wobj Workpiece coordinate system ID
	 * @param [in] vel Velocity percentage
	 * @param [in] acc Acceleration percentage
	 * @param [in] ovl Velocity scaling factor
	 * @param [in] oacc Acceleration scaling factor
	 * @param [in] moveDirection Direction of movement; 0- clockwise; 1- Counterclockwise
	 * @return Error code
	 */
	errno_t MoveIntersectLine(DescPose mainPoint[6], DescPose piecePoint[6], int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveDirection);

	/**
	 * @brief Intersection line movement
	 * @param [in] mainPoint Cartesian poses of the six teaching points of the main pipeline
	 * @param [in] mainExaxisPos Exaxis poses of the six teaching points of the main pipeline
	 * @param [in] piecePoint Cartesian poses of the six teaching points of the auxiliary pipeline
	 * @param [in] pieceExaxisPos Exaxis poses of the six teaching points of the auxiliary pipeline
	 * @param [in] extAxisFlag Whether to enable the extended axis; 0- Not enabled; 1- Enable
	 * @param [in] exaxisPos The position of the starting expansion axis
	 * @param [in] tool Tool coordinate system ID
	 * @param [in] wobj Workpiece coordinate system ID
	 * @param [in] vel Velocity percentage
	 * @param [in] acc Acceleration percentage
	 * @param [in] ovl Velocity scaling factor
	 * @param [in] oacc Acceleration scaling factor
	 * @param [in] moveDirection Direction of movement; 0- clockwise; 1- Counterclockwise
	 * @param [in] offset Offset Descartes
	 * @return Error code
	 */
	errno_t MoveIntersectLine(DescPose mainPoint[6], ExaxisPos mainExaxisPos[6], DescPose piecePoint[6], ExaxisPos pieceExaxisPos[6], int extAxisFlag, ExaxisPos exaxisPos[4], int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveDirection, DescPose offset);

	/**
	 * @brief Get the hysteresis error of the joint torque sensor
	 * @param [out] hysteresisError j1~j6 Joint hysteresis error
	 * @return Error code
	 */
	errno_t JointHysteresisError(double hysteresisError[6]);

	/**
	 * @brief Get the repeatability accuracy of the joint torque sensor
	 * @param [out] repeatability j1~j6 Repeatability accuracy of joint torque sensors
	 * @return Error code
	 */
	errno_t JointRepeatability(double repeatability[6]);

	/**
	 * @brief Set the parameters of the joint torque sensor
	 * @param [in] M J1-J6 Mass coefficient [0.001 ~ 10]
	 * @param [in] B J1-J6 Damping coefficient [0.001 ~ 10]
	 * @param [in] K J1-J6 Stiffness coefficient [0.001 ~ 10]
	 * @param [in] threshold Force control threshold，Nm
	 * @param [in] sensitivity Sensitivity,Nm/V [0 ~ 10]
	 * @param [in] setZeroFlag Function enable flag bit; 0- Close; 1- Turn on; Position 2- Position 1 records the zero point; Position 3- Position 2 records the zero point
	 * @return Error code
	 */
	errno_t SetAdmittanceParams(double M[6], double B[6], double K[6], double threshold[6], double sensitivity[6], int setZeroFlag);


	/**
	 *@brief  Robot interface class destructor
	 */
	~FRRobot();

private:
	void RobotStateRoutineThread();
	void RobotInstCmdSendRoutineThread();
	void RobotInstCmdRecvRoutineThread();
	void RobotTaskRoutineThread();
	char serverUrl[64];

	bool rpc_done = false;

	/**
* @brief download file
* @param [in] fileType File type 0-lua file
* @param [in] fileName File name "test.lua"
* @param [in] saveFilePath Save file path C: //test/
* @return error code
	 */
	errno_t FileDownLoad(int fileType, std::string fileName, std::string saveFilePath);

	/**
* @brief Upload file
* @param [in] fileType File type 0-lua file
* @param [in] fileName File name "test.lua"
* @param [in] upLoadFilePath Save file C: //test/
* @return error code
	 */
	errno_t FileUpLoad(int fileType, std::string filePath, int reUp = 0);

	errno_t GetFileUploadBreakState(int& breakFlag, std::string& md5, int& fileSize, int& curSentSize);

	/**
* @brief Upload file
* @param [in] fileType File type 0-lua file
* @param [in] fileName File name "test.lua"
	 * @return error code
	 */
	errno_t FileDelete(int fileType, std::string fileName);

	std::vector<std::string> split(const std::string& s, char delim);

	std::vector<std::string> split(std::string s, std::string delimiter);

	bool IsSockError();
	int GetSafetyCode();

private:
	uint8_t robot_realstate_exit = 0;
	uint8_t robot_instcmd_send_exit = 0;
	uint8_t robot_instcmd_recv_exit = 0;
	uint8_t robot_task_exit = 0;
	bool is_sendcmd = false;
	char g_sendbuf[1024 * 4] = { 0 };
	char g_recvbuf[1024 * 4] = { 0 };
	int g_sock_com_err;
	double fileUploadPercent;

	char robot_ip[64];
	std::shared_ptr<ROBOT_STATE_PKG> robot_state_pkg;

	std::shared_ptr <FRTcpClient> rtClient;
	std::shared_ptr <FRTcpClient> cmdClient;

};

#endif
