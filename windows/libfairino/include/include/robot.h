#ifndef FRROBOT_H_
#define FRROBOT_H_

#ifdef WINDOWS_OPTION
#define FR_LIB_EXPORT __declspec(dllexport)
#else
#define FR_LIB_EXPORT
#endif

#include "robot_types.h"
#include <iostream>
#include <list>
#include <vector>

class FR_LIB_EXPORT FRRobot
{
public:
	/**
	 * @brief  机器人接口类构造函数
	 */
	FRRobot();

	/**
	 * @brief  与机器人控制器建立通讯
	 * @param  [in] ip  控制器IP地址，出场默认为192.168.58.2
	 * @return 错误码
	 */
	errno_t RPC(const char *ip);

	/**
	 * @brief  与机器人控制器关闭通讯
	 * @return 错误码
	 */
	errno_t CloseRPC();

	/**
	 * @brief  查询SDK版本号
	 * @param  [out] version   SDK版本号
	 * @return  错误码
	 */
	errno_t GetSDKVersion(char *version);

	/**
	 * @brief  获取控制器IP
	 * @param  [out] ip  控制器IP
	 * @return  错误码
	 */
	errno_t GetControllerIP(char *ip);

	/**
	 * @brief  控制机器人进入或退出拖动示教模式
	 * @param  [in] state 0-退出拖动示教模式，1-进入拖动示教模式
	 * @return  错误码
	 */
	errno_t DragTeachSwitch(uint8_t state);

	/**
	 * @brief  查询机器人是否处于拖动示教模式
	 * @param  [out] state 0-非拖动示教模式，1-拖动示教模式
	 * @return  错误码
	 */
	errno_t IsInDragTeach(uint8_t *state);

	/**
	 * @brief  控制机器人上使能或下使能，机器人上电后默认自动上使能
	 * @param  [in] state  0-下使能，1-上使能
	 * @return  错误码
	 */
	errno_t RobotEnable(uint8_t state);

	/**
	 * @brief 控制机器人手自动模式切换
	 * @param [in] mode 0-自动模式，1-手动模式
	 * @return 错误码
	 */
	errno_t Mode(int mode);

	/**
	 * @brief  jog点动
	 * @param  [in]  ref 0-关节点动，2-基坐标系下点动，4-工具坐标系下点动，8-工件坐标系下点动
	 * @param  [in]  nb 1-关节1(或x轴)，2-关节2(或y轴)，3-关节3(或z轴)，4-关节4(或绕x轴旋转)，5-关节5(或绕y轴旋转)，6-关节6(或绕z轴旋转)
	 * @param  [in]  dir 0-负方向，1-正方向
	 * @param  [in]  vel 速度百分比，[0~100]
	 * @param  [in]  acc 加速度百分比， [0~100]
	 * @param  [in]  max_dis 单次点动最大角度，单位[°]或距离，单位[mm]
	 * @return  错误码
	 */
	errno_t StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis);

	/**
	 * @brief  jog点动减速停止
	 * @param  [in]  ref  1-关节点动停止，3-基坐标系下点动停止，5-工具坐标系下点动停止，9-工件坐标系下点动停止
	 * @return  错误码
	 */
	errno_t StopJOG(uint8_t ref);

	/**
	 * @brief jog点动立即停止
	 * @return  错误码
	 */
	errno_t ImmStopJOG();

	/**
	 * @brief  关节空间运动
	 * @param  [in] joint_pos  目标关节位置,单位deg
	 * @param  [in] desc_pos   目标笛卡尔位姿
	 * @param  [in] tool  工具坐标号，范围[0~14]
	 * @param  [in] user  工件坐标号，范围[0~14]
	 * @param  [in] vel  速度百分比，范围[0~100]
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @param  [in] epos  扩展轴位置，单位mm
	 * @param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
	 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 * @param  [in] offset_pos  位姿偏移量
	 * @return  错误码
	 */
	errno_t MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos);

	/**
	 * @brief  笛卡尔空间直线运动
	 * @param  [in] joint_pos  目标关节位置,单位deg
	 * @param  [in] desc_pos   目标笛卡尔位姿
	 * @param  [in] tool  工具坐标号，范围[0~14]
	 * @param  [in] user  工件坐标号，范围[0~14]
	 * @param  [in] vel  速度百分比，范围[0~100]
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
	 * @param  [in] epos  扩展轴位置，单位mm
	 * @param  [in] search  0-不焊丝寻位，1-焊丝寻位
	 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 * @param  [in] offset_pos  位姿偏移量
	 * @return  错误码
	 */
	errno_t MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

	/**
	 * @brief  笛卡尔空间圆弧运动
	 * @param  [in] joint_pos_p  路径点关节位置,单位deg
	 * @param  [in] desc_pos_p   路径点笛卡尔位姿
	 * @param  [in] ptool  工具坐标号，范围[0~14]
	 * @param  [in] puser  工件坐标号，范围[0~14]
	 * @param  [in] pvel  速度百分比，范围[0~100]
	 * @param  [in] pacc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] epos_p  扩展轴位置，单位mm
	 * @param  [in] poffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 * @param  [in] offset_pos_p  位姿偏移量
	 * @param  [in] joint_pos_t  目标点关节位置,单位deg
	 * @param  [in] desc_pos_t   目标点笛卡尔位姿
	 * @param  [in] ttool  工具坐标号，范围[0~14]
	 * @param  [in] tuser  工件坐标号，范围[0~14]
	 * @param  [in] tvel  速度百分比，范围[0~100]
	 * @param  [in] tacc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] epos_t  扩展轴位置，单位mm
	 * @param  [in] toffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 * @param  [in] offset_pos_t  位姿偏移量
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
	 * @return  错误码
	 */
	errno_t MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t, float ovl, float blendR);

	/**
	 * @brief  笛卡尔空间整圆运动
	 * @param  [in] joint_pos_p  路径点1关节位置,单位deg
	 * @param  [in] desc_pos_p   路径点1笛卡尔位姿
	 * @param  [in] ptool  工具坐标号，范围[0~14]
	 * @param  [in] puser  工件坐标号，范围[0~14]
	 * @param  [in] pvel  速度百分比，范围[0~100]
	 * @param  [in] pacc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] epos_p  扩展轴位置，单位mm
	 * @param  [in] joint_pos_t  路径点2关节位置,单位deg
	 * @param  [in] desc_pos_t   路径点2笛卡尔位姿
	 * @param  [in] ttool  工具坐标号，范围[0~14]
	 * @param  [in] tuser  工件坐标号，范围[0~14]
	 * @param  [in] tvel  速度百分比，范围[0~100]
	 * @param  [in] tacc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] epos_t  扩展轴位置，单位mm
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 * @param  [in] offset_pos  位姿偏移量
	 * @return  错误码
	 */
	errno_t Circle(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos);

	/**
	 * @brief  笛卡尔空间螺旋线运动
	 * @param  [in] joint_pos  目标关节位置,单位deg
	 * @param  [in] desc_pos   目标笛卡尔位姿
	 * @param  [in] tool  工具坐标号，范围[0~14]
	 * @param  [in] user  工件坐标号，范围[0~14]
	 * @param  [in] vel  速度百分比，范围[0~100]
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] epos  扩展轴位置，单位mm
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 * @param  [in] offset_pos  位姿偏移量
	 * @param  [in] spiral_param  螺旋参数
	 * @return  错误码
	 */
	errno_t NewSpiral(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param);

	/**
	 * @brief 伺服运动开始，配合ServoJ、ServoCart指令使用
	 * @return  错误码
	 */
	errno_t ServoMoveStart();

	/**
	 * @brief 伺服运动结束，配合ServoJ、ServoCart指令使用
	 * @return  错误码
	 */
	errno_t ServoMoveEnd();

	/**
	 * @brief  关节空间伺服模式运动
	 * @param  [in] joint_pos  目标关节位置,单位deg
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
	 * @param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
	 * @param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
	 * @param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
	 * @param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
	 * @return  错误码
	 */
	errno_t ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);

	/**
	 * @brief  笛卡尔空间伺服模式运动
	 * @param  [in]  mode  0-绝对运动(基坐标系)，1-增量运动(基坐标系)，2-增量运动(工具坐标系)
	 * @param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
	 * @param  [in]  pos_gain  位姿增量比例系数，仅在增量运动下生效，范围[0~1]
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
	 * @param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
	 * @param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
	 * @param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
	 * @param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
	 * @return  错误码
	 */
	errno_t ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);

	/**
	 * @brief  笛卡尔空间点到点运动
	 * @param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
	 * @param  [in] tool  工具坐标号，范围[0~14]
	 * @param  [in] user  工件坐标号，范围[0~14]
	 * @param  [in] vel  速度百分比，范围[0~100]
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
	 * @param  [in] config  关节空间配置，[-1]-参考当前关节位置解算，[0~7]-参考特定关节空间配置解算，默认为-1
	 * @return  错误码
	 */
	errno_t MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);

	/**
	 * @brief  样条运动开始
	 * @return  错误码
	 */
	errno_t SplineStart();

	/**
	 * @brief  关节空间样条运动
	 * @param  [in] joint_pos  目标关节位置,单位deg
	 * @param  [in] desc_pos   目标笛卡尔位姿
	 * @param  [in] tool  工具坐标号，范围[0~14]
	 * @param  [in] user  工件坐标号，范围[0~14]
	 * @param  [in] vel  速度百分比，范围[0~100]
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @return  错误码
	 */
	errno_t SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl);

	/**
	 * @brief  样条运动结束
	 * @return  错误码
	 */
	errno_t SplineEnd();

	/**
	 * @brief 新样条运动开始
	 * @param  [in] type   0-圆弧过渡，1-给定点位为路径点
	 * @param  [in] averageTime  全局平均衔接时间(ms)(10 ~  )，默认2000
	 * @return  错误码
	 */
	errno_t NewSplineStart(int type, int averageTime=2000);

	/**
	 * @brief 新样条指令点
	 * @param  [in] joint_pos  目标关节位置,单位deg
	 * @param  [in] desc_pos   目标笛卡尔位姿
	 * @param  [in] tool  工具坐标号，范围[0~14]
	 * @param  [in] user  工件坐标号，范围[0~14]
	 * @param  [in] vel  速度百分比，范围[0~100]
	 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 * @param  [in] ovl  速度缩放因子，范围[0~100]
	 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
	 * @param  [in] lastFlag 是否为最后一个点，0-否，1-是
	 * @return  错误码
	 */
	errno_t NewSplinePoint(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag);

	/**
	 * @brief 新样条运动结束
	 * @return  错误码
	 */
	errno_t NewSplineEnd();

	/**
	 * @brief 终止运动
	 * @return  错误码
	 */
	errno_t StopMotion();

	/**
	 * @brief 暂停运动
	 * @return  错误码
	 */
	errno_t PauseMotion();

	/**
	 * @brief 恢复运动
	 * @return  错误码
	 */
	errno_t ResumeMotion();

	/**
	 * @brief  点位整体偏移开始
	 * @param  [in]  flag  0-基坐标系下/工件坐标系下偏移，2-工具坐标系下偏移
	 * @param  [in] offset_pos  位姿偏移量
	 * @return  错误码
	 */
	errno_t PointsOffsetEnable(int flag, DescPose *offset_pos);

	/**
	 * @brief  点位整体偏移结束
	 * @return  错误码
	 */
	errno_t PointsOffsetDisable();

	/**
	 * @brief  设置控制箱数字量输出
	 * @param  [in] id  io编号，范围[0~15]
	 * @param  [in] status 0-关，1-开
	 * @param  [in] smooth 0-不平滑， 1-平滑
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @return  错误码
	 */
	errno_t SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
	 * @brief  设置工具数字量输出
	 * @param  [in] id  io编号，范围[0~1]
	 * @param  [in] status 0-关，1-开
	 * @param  [in] smooth 0-不平滑， 1-平滑
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @return  错误码
	 */
	errno_t SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
	 * @brief  设置控制箱模拟量输出
	 * @param  [in] id  io编号，范围[0~1]
	 * @param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @return  错误码
	 */
	errno_t SetAO(int id, float value, uint8_t block);

	/**
	 * @brief  设置工具模拟量输出
	 * @param  [in] id  io编号，范围[0]
	 * @param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @return  错误码
	 */
	errno_t SetToolAO(int id, float value, uint8_t block);

	/**
	 * @brief  获取控制箱数字量输入
	 * @param  [in] id  io编号，范围[0~15]
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @param  [out] result  0-低电平，1-高电平
	 * @return  错误码
	 */
	errno_t GetDI(int id, uint8_t block, uint8_t *result);

	/**
	 * @brief  获取工具数字量输入
	 * @param  [in] id  io编号，范围[0~1]
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @param  [out] result  0-低电平，1-高电平
	 * @return  错误码
	 */
	errno_t GetToolDI(int id, uint8_t block, uint8_t *result);

	/**
	 * @brief 等待控制箱数字量输入
	 * @param  [in] id  io编号，范围[0~15]
	 * @param  [in]  status 0-关，1-开
	 * @param  [in]  max_time  最大等待时间，单位ms
	 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 * @return  错误码
	 */
	errno_t WaitDI(int id, uint8_t status, int max_time, int opt);

	/**
	 * @brief 等待控制箱多路数字量输入
	 * @param  [in] mode 0-多路与，1-多路或
	 * @param  [in] id  io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7
	 * @param  [in]  status 0-关，1-开
	 * @param  [in]  max_time  最大等待时间，单位ms
	 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 * @return  错误码
	 */
	errno_t WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt);

	/**
	 * @brief 等待工具数字量输入
	 * @param  [in] id  io编号，范围[0~1]
	 * @param  [in]  status 0-关，1-开
	 * @param  [in]  max_time  最大等待时间，单位ms
	 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 * @return  错误码
	 */
	errno_t WaitToolDI(int id, uint8_t status, int max_time, int opt);

	/**
	 * @brief  获取控制箱模拟量输入
	 * @param  [in] id  io编号，范围[0~1]
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 * @return  错误码
	 */
	errno_t GetAI(int id, uint8_t block, float *result);

	/**
	 * @brief  获取工具模拟量输入
	 * @param  [in] id  io编号，范围[0]
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 * @return  错误码
	 */
	errno_t GetToolAI(int id, uint8_t block, float *result);

	/**
	 * @brief 获取机器人末端点记录按钮状态
	 * @param [out] state 按钮状态，0-按下，1-松开
	 * @return 错误码
	 */
	errno_t GetAxlePointRecordBtnState(uint8_t *state);

	/**
	 * @brief 获取机器人末端DO输出状态
	 * @param [out] do_state DO输出状态，do0~do1对应bit1~bit2,从bit0开始
	 * @return 错误码
	 */
	errno_t GetToolDO(uint8_t *do_state);

	/**
	 * @brief 获取机器人控制器DO输出状态
	 * @param [out] do_state_h DO输出状态，co0~co7对应bit0~bit7
	 * @param [out] do_state_l DO输出状态，do0~do7对应bit0~bit7
	 * @return 错误码
	 */
	errno_t GetDO(uint8_t *do_state_h, uint8_t *do_state_l);

	/**
	 * @brief 等待控制箱模拟量输入
	 * @param  [in] id  io编号，范围[0~1]
	 * @param  [in]  sign 0-大于，1-小于
	 * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 * @param  [in]  max_time  最大等待时间，单位ms
	 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 * @return  错误码
	 */
	errno_t WaitAI(int id, int sign, float value, int max_time, int opt);

	/**
	 * @brief 等待工具模拟量输入
	 * @param  [in] id  io编号，范围[0]
	 * @param  [in]  sign 0-大于，1-小于
	 * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 * @param  [in]  max_time  最大等待时间，单位ms
	 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 * @return  错误码
	 */
	errno_t WaitToolAI(int id, int sign, float value, int max_time, int opt);

	/**
	 * @brief  设置全局速度
	 * @param  [in]  vel  速度百分比，范围[0~100]
	 * @return  错误码
	 */
	errno_t SetSpeed(int vel);

	/**
	 * @brief  设置系统变量值
	 * @param  [in]  id  变量编号，范围[1~20]
	 * @param  [in]  value 变量值
	 * @return  错误码
	 */
	errno_t SetSysVarValue(int id, float value);

	/**
	 * @brief 设置工具参考点-六点法
	 * @param [in] point_num 点编号,范围[1~6]
	 * @return 错误码
	 */
	errno_t SetToolPoint(int point_num);

	/**
	 * @brief  计算工具坐标系
	 * @param [out] tcp_pose 工具坐标系
	 * @return 错误码
	 */
	errno_t ComputeTool(DescPose *tcp_pose);

	/**
	 * @brief 设置工具参考点-四点法
	 * @param [in] point_num 点编号,范围[1~4]
	 * @return 错误码
	 */
	errno_t SetTcp4RefPoint(int point_num);

	/**
	 * @brief  计算工具坐标系
	 * @param [out] tcp_pose 工具坐标系
	 * @return 错误码
	 */
	errno_t ComputeTcp4(DescPose *tcp_pose);

	/**
	 * @brief  设置工具坐标系
	 * @param  [in] id 坐标系编号，范围[0~14]
	 * @param  [in] coord  工具中心点相对于末端法兰中心位姿
	 * @param  [in] type  0-工具坐标系，1-传感器坐标系
	 * @param  [in] install 安装位置，0-机器人末端，1-机器人外部
	 * @return  错误码
	 */
	errno_t SetToolCoord(int id, DescPose *coord, int type, int install);

	/**
	 * @brief  设置工具坐标系列表
	 * @param  [in] id 坐标系编号，范围[0~14]
	 * @param  [in] coord  工具中心点相对于末端法兰中心位姿
	 * @param  [in] type  0-工具坐标系，1-传感器坐标系
	 * @param  [in] install 安装位置，0-机器人末端，1-机器人外部
	 * @return  错误码
	 */
	errno_t SetToolList(int id, DescPose *coord, int type, int install);

	/**
	 * @brief 设置外部工具参考点-六点法
	 * @param [in] point_num 点编号,范围[1~4]
	 * @return 错误码
	 */
	errno_t SetExTCPPoint(int point_num);

	/**
	 * @brief  计算外部工具坐标系
	 * @param [out] tcp_pose 外部工具坐标系
	 * @return 错误码
	 */
	errno_t ComputeExTCF(DescPose *tcp_pose);

	/**
	 * @brief  设置外部工具坐标系
	 * @param  [in] id 坐标系编号，范围[0~14]
	 * @param  [in] etcp  工具中心点相对末端法兰中心位姿
	 * @param  [in] etool  待定
	 * @return  错误码
	 */
	errno_t SetExToolCoord(int id, DescPose *etcp, DescPose *etool);

	/**
	 * @brief  设置外部工具坐标系列表
	 * @param  [in] id 坐标系编号，范围[0~14]
	 * @param  [in] etcp  工具中心点相对末端法兰中心位姿
	 * @param  [in] etool  待定
	 * @return  错误码
	 */
	errno_t SetExToolList(int id, DescPose *etcp, DescPose *etool);

	/**
	 * @brief 设置工件参考点-三点法
	 * @param [in] point_num 点编号,范围[1~3]
	 * @return 错误码
	 */
	errno_t SetWObjCoordPoint(int point_num);

	/**
	 * @brief 计算工件坐标系
	 * @param [in] method 计算方式 0：原点-x轴-z轴  1：原点-x轴-xy平面
	 * @param [out] wobj_pose 工件坐标系
	 * @return 错误码
	 */
	errno_t ComputeWObjCoord(int method, DescPose *wobj_pose);

	/**
	 * @brief  设置工件坐标系
	 * @param  [in] id 坐标系编号，范围[0~14]
	 * @param  [in] coord  工件坐标系相对于末端法兰中心位姿
	 * @return  错误码
	 */
	errno_t SetWObjCoord(int id, DescPose *coord);

	/**
	 * @brief  设置工件坐标系列表
	 * @param  [in] id 坐标系编号，范围[0~14]
	 * @param  [in] coord  工件坐标系相对于末端法兰中心位姿
	 * @return  错误码
	 */
	errno_t SetWObjList(int id, DescPose *coord);

	/**
	 * @brief  设置末端负载重量
	 * @param  [in] weight  负载重量，单位kg
	 * @return  错误码
	 */
	errno_t SetLoadWeight(float weight);

	/**
	 * @brief  设置末端负载质心坐标
	 * @param  [in] coord 质心坐标，单位mm
	 * @return  错误码
	 */
	errno_t SetLoadCoord(DescTran *coord);

	/**
	 * @brief  设置机器人安装方式
	 * @param  [in] install  安装方式，0-正装，1-侧装，2-倒装
	 * @return  错误码
	 */
	errno_t SetRobotInstallPos(uint8_t install);

	/**
	 * @brief  设置机器人安装角度，自由安装
	 * @param  [in] yangle  倾斜角
	 * @param  [in] zangle  旋转角
	 * @return  错误码
	 */
	errno_t SetRobotInstallAngle(double yangle, double zangle);

	/**
	 * @brief  等待指定时间
	 * @param  [in]  t_ms  单位ms
	 * @return  错误码
	 */
	errno_t WaitMs(int t_ms);

	/**
	 * @brief 设置碰撞等级
	 * @param  [in]  mode  0-等级，1-百分比
	 * @param  [in]  level 碰撞阈值，等级对应范围[],百分比对应范围[0~1]
	 * @param  [in]  config 0-不更新配置文件，1-更新配置文件
	 * @return  错误码
	 */
	errno_t SetAnticollision(int mode, float level[6], int config);

	/**
	 * @brief  设置碰撞后策略
	 * @param  [in] strategy  0-报错停止，1-继续运行
	 * @return  错误码
	 */
	errno_t SetCollisionStrategy(int strategy);

	/**
	 * @brief  设置正限位
	 * @param  [in] limit 六个关节位置，单位deg
	 * @return  错误码
	 */
	errno_t SetLimitPositive(float limit[6]);

	/**
	 * @brief  设置负限位
	 * @param  [in] limit 六个关节位置，单位deg
	 * @return  错误码
	 */
	errno_t SetLimitNegative(float limit[6]);

	/**
	 * @brief  错误状态清除
	 * @return  错误码
	 */
	errno_t ResetAllError();

	/**
	 * @brief  关节摩擦力补偿开关
	 * @param  [in]  state  0-关，1-开
	 * @return  错误码
	 */
	errno_t FrictionCompensationOnOff(uint8_t state);

	/**
	 * @brief  设置关节摩擦力补偿系数-正装
	 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 * @return  错误码
	 */
	errno_t SetFrictionValue_level(float coeff[6]);

	/**
	 * @brief  设置关节摩擦力补偿系数-侧装
	 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 * @return  错误码
	 */
	errno_t SetFrictionValue_wall(float coeff[6]);

	/**
	 * @brief  设置关节摩擦力补偿系数-倒装
	 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 * @return  错误码
	 */
	errno_t SetFrictionValue_ceiling(float coeff[6]);

	/**
	 * @brief  设置关节摩擦力补偿系数-自由安装
	 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 * @return  错误码
	 */
	errno_t SetFrictionValue_freedom(float coeff[6]);

	/**
	 * @brief  获取机器人安装角度
	 * @param  [out] yangle 倾斜角
	 * @param  [out] zangle 旋转角
	 * @return  错误码
	 */
	errno_t GetRobotInstallAngle(float *yangle, float *zangle);

	/**
	 * @brief  获取系统变量值
	 * @param  [in] id 系统变量编号，范围[1~20]
	 * @param  [out] value  系统变量值
	 * @return  错误码
	 */
	errno_t GetSysVarValue(int id, float *value);

	/**
	 * @brief  获取当前关节位置(角度)
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] jPos 六个关节位置，单位deg
	 * @return  错误码
	 */
	errno_t GetActualJointPosDegree(uint8_t flag, JointPos *jPos);

	/**
	 * @brief  获取关节反馈速度-deg/s
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] speed [x,y,z,rx,ry,rz]速度
	 * @return  错误码
	 */
	errno_t GetActualJointSpeedsDegree(uint8_t flag, float speed[6]);

	/**
	 * @brief  获取关节反馈加速度-deg/s^2
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] acc 六个关节加速度
	 * @return  错误码
	 */
	errno_t GetActualJointAccDegree(uint8_t flag, float acc[6]);

	/**
	 * @brief  获取TCP指令速度
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] tcp_speed 线性速度
	 * @param  [out] ori_speed 姿态速度
	 * @return  错误码
	 */
	errno_t GetTargetTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

	/**
	 * @brief  获取TCP反馈速度
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] tcp_speed 线性速度
	 * @param  [out] ori_speed 姿态速度
	 * @return  错误码
	 */
	errno_t GetActualTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

	/**
	 * @brief  获取TCP指令速度
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] speed [x,y,z,rx,ry,rz]速度
	 * @return  错误码
	 */
	errno_t GetTargetTCPSpeed(uint8_t flag, float speed[6]);

	/**
	 * @brief  获取TCP反馈速度
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] speed [x,y,z,rx,ry,rz]速度
	 * @return  错误码
	 */
	errno_t GetActualTCPSpeed(uint8_t flag, float speed[6]);

	/**
	 * @brief  获取当前工具位姿
	 * @param  [in] flag  0-阻塞，1-非阻塞
	 * @param  [out] desc_pos  工具位姿
	 * @return  错误码
	 */
	errno_t GetActualTCPPose(uint8_t flag, DescPose *desc_pos);

	/**
	 * @brief  获取当前工具坐标系编号
	 * @param  [in] flag  0-阻塞，1-非阻塞
	 * @param  [out] id  工具坐标系编号
	 * @return  错误码
	 */
	errno_t GetActualTCPNum(uint8_t flag, int *id);

	/**
	 * @brief  获取当前工件坐标系编号
	 * @param  [in] flag  0-阻塞，1-非阻塞
	 * @param  [out] id  工件坐标系编号
	 * @return  错误码
	 */
	errno_t GetActualWObjNum(uint8_t flag, int *id);

	/**
	 * @brief  获取当前末端法兰位姿
	 * @param  [in] flag  0-阻塞，1-非阻塞
	 * @param  [out] desc_pos  法兰位姿
	 * @return  错误码
	 */
	errno_t GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos);

	/**
	 * @brief  逆运动学求解
	 * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
	 * @param  [in] desc_pos 笛卡尔位姿
	 * @param  [in] config 关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解
	 * @param  [out] joint_pos 关节位置
	 * @return  错误码
	 */
	errno_t GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos);

	/**
	 * @brief  逆运动学求解，参考指定关节位置求解
	 * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
	 * @param  [in] desc_pos 笛卡尔位姿
	 * @param  [in] joint_pos_ref 参考关节位置
	 * @param  [out] joint_pos 关节位置
	 * @return  错误码
	 */
	errno_t GetInverseKinRef(int type, DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos);

	/**
	 * @brief  逆运动学求解，参考指定关节位置判断是否有解
	 * @param  [in] posMode 0 绝对位姿，1 相对位姿-基坐标系，2 相对位姿-工具坐标系
	 * @param  [in] desc_pos 笛卡尔位姿
	 * @param  [in] joint_pos_ref 参考关节位置
	 * @param  [out] result 0-无解，1-有解
	 * @return  错误码
	 */
	errno_t GetInverseKinHasSolution(int type, DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result);

	/**
	 * @brief  正运动学求解
	 * @param  [in] joint_pos 关节位置
	 * @param  [out] desc_pos 笛卡尔位姿
	 * @return  错误码
	 */
	errno_t GetForwardKin(JointPos *joint_pos, DescPose *desc_pos);

	/**
	 * @brief 获取当前关节转矩
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] torques 关节转矩
	 * @return  错误码
	 */
	errno_t GetJointTorques(uint8_t flag, float torques[6]);

	/**
	 * @brief  获取当前负载的重量
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] weight 负载重量，单位kg
	 * @return  错误码
	 */
	errno_t GetTargetPayload(uint8_t flag, float *weight);

	/**
	 * @brief  获取当前负载的质心
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] cog 负载质心，单位mm
	 * @return  错误码
	 */
	errno_t GetTargetPayloadCog(uint8_t flag, DescTran *cog);

	/**
	 * @brief  获取当前工具坐标系
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] desc_pos 工具坐标系位姿
	 * @return  错误码
	 */
	errno_t GetTCPOffset(uint8_t flag, DescPose *desc_pos);

	/**
	 * @brief  获取当前工件坐标系
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] desc_pos 工件坐标系位姿
	 * @return  错误码
	 */
	errno_t GetWObjOffset(uint8_t flag, DescPose *desc_pos);

	/**
	 * @brief  获取关节软限位角度
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] negative  负限位角度，单位deg
	 * @param  [out] positive  正限位角度，单位deg
	 * @return  错误码
	 */
	errno_t GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6]);

	/**
	 * @brief  获取系统时间
	 * @param  [out] t_ms 单位ms
	 * @return  错误码
	 */
	errno_t GetSystemClock(float *t_ms);

	/**
	 * @brief  获取机器人当前关节位置
	 * @param  [out]  config  关节空间配置，范围[0~7]
	 * @return  错误码
	 */
	errno_t GetRobotCurJointsConfig(int *config);

	/**
	 * @brief  获取机器人当前速度
	 * @param  [out]  vel  速度，单位mm/s
	 * @return  错误码
	 */
	errno_t GetDefaultTransVel(float *vel);

	/**
	 * @brief  查询机器人运动是否完成
	 * @param  [out]  state  0-未完成，1-完成
	 * @return  错误码
	 */
	errno_t GetRobotMotionDone(uint8_t *state);

	/**
	 * @brief  查询机器人错误码
	 * @param  [out]  maincode  主错误码
	 * @param  [out]  subcode   子错误码
	 * @return  错误码
	 */
	errno_t GetRobotErrorCode(int *maincode, int *subcode);

	/**
	 * @brief  查询机器人示教管理点位数据
	 * @param  [in]  name  点位名
	 * @param  [out]  data   点位数据
	 * @return  错误码
	 */
	errno_t GetRobotTeachingPoint(char name[64], float data[20]);

	/**
	 * @brief  查询机器人运动队列缓存长度
	 * @param  [out]  len  缓存长度
	 * @return  错误码
	 */
	errno_t GetMotionQueueLength(int *len);

	/**
	 * @brief  设置轨迹记录参数
	 * @param  [in] type  记录数据类型，1-关节位置
	 * @param  [in] name  轨迹文件名
	 * @param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
	 * @param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
	 * @param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
	 * @return  错误码
	 */
	errno_t SetTPDParam(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);

	/**
	 * @brief  开始轨迹记录
	 * @param  [in] type  记录数据类型，1-关节位置
	 * @param  [in] name  轨迹文件名
	 * @param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
	 * @param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
	 * @param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
	 * @return  错误码
	 */
	errno_t SetTPDStart(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);

	/**
	 * @brief  停止轨迹记录
	 * @return  错误码
	 */
	errno_t SetWebTPDStop();

	/**
	 * @brief  删除轨迹记录
	 * @param  [in] name  轨迹文件名
	 * @return  错误码
	 */
	errno_t SetTPDDelete(char name[30]);

	/**
	 * @brief  轨迹预加载
	 * @param  [in] name  轨迹文件名
	 * @return  错误码
	 */
	errno_t LoadTPD(char name[30]);

	/**
	 * @brief  获取轨迹起始位姿
	 * @param  [in] name 轨迹文件名
	 * @return  错误码
	 */
	errno_t GetTPDStartPose(char name[30], DescPose *desc_pose);

	/**
	 * @brief  轨迹复现
	 * @param  [in] name  轨迹文件名
	 * @param  [in] blend 0-不平滑，1-平滑
	 * @param  [in] ovl  速度缩放百分比，范围[0~100]
	 * @return  错误码
	 */
	errno_t MoveTPD(char name[30], uint8_t blend, float ovl);

	/**
	 * @brief  轨迹预处理
	 * @param  [in] name  轨迹文件名
	 * @param  [in] ovl 速度缩放百分比，范围[0~100]
	 * @param  [in] opt 1-控制点，默认为1
	 * @return  错误码
	 */
	errno_t LoadTrajectoryJ(char name[30], float ovl, int opt);

	/**
	 * @brief  轨迹复现
	 * @return  错误码
	 */
	errno_t MoveTrajectoryJ();

	/**
	 * @brief  获取轨迹起始位姿
	 * @param  [in] name 轨迹文件名
	 * @return  错误码
	 */
	errno_t GetTrajectoryStartPose(char name[30], DescPose *desc_pose);

	/**
	 * @brief  获取轨迹点编号
	 * @return  错误码
	 */
	errno_t GetTrajectoryPointNum(int *pnum);

	/**
	 * @brief  设置轨迹运行中的速度
	 * @param  [in] ovl 速度百分比
	 * @return  错误码
	 */
	errno_t SetTrajectoryJSpeed(float ovl);

	/**
	 * @brief  设置轨迹运行中的力和扭矩
	 * @param  [in] ft 三个方向的力和扭矩，单位N和Nm
	 * @return  错误码
	 */
	errno_t SetTrajectoryJForceTorque(ForceTorque *ft);

	/**
	 * @brief  设置轨迹运行中的沿x方向的力
	 * @param  [in] fx 沿x方向的力，单位N
	 * @return  错误码
	 */
	errno_t SetTrajectoryJForceFx(double fx);

	/**
	 * @brief  设置轨迹运行中的沿y方向的力
	 * @param  [in] fy 沿y方向的力，单位N
	 * @return  错误码
	 */
	errno_t SetTrajectoryJForceFy(double fy);

	/**
	 * @brief  设置轨迹运行中的沿z方向的力
	 * @param  [in] fz 沿x方向的力，单位N
	 * @return  错误码
	 */
	errno_t SetTrajectoryJForceFz(double fz);

	/**
	 * @brief  设置轨迹运行中的绕x轴的扭矩
	 * @param  [in] tx 绕x轴的扭矩，单位Nm
	 * @return  错误码
	 */
	errno_t SetTrajectoryJTorqueTx(double tx);

	/**
	 * @brief  设置轨迹运行中的绕x轴的扭矩
	 * @param  [in] ty 绕y轴的扭矩，单位Nm
	 * @return  错误码
	 */
	errno_t SetTrajectoryJTorqueTy(double ty);

	/**
	 * @brief  设置轨迹运行中的绕x轴的扭矩
	 * @param  [in] tz 绕z轴的扭矩，单位Nm
	 * @return  错误码
	 */
	errno_t SetTrajectoryJTorqueTz(double tz);

	/**
	 * @brief  设置开机自动加载默认的作业程序
	 * @param  [in] flag  0-开机不自动加载默认程序，1-开机自动加载默认程序
	 * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
	 * @return  错误码
	 */
	errno_t LoadDefaultProgConfig(uint8_t flag, char program_name[64]);

	/**
	 * @brief  加载指定的作业程序
	 * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
	 * @return  错误码
	 */
	errno_t ProgramLoad(char program_name[64]);

	/**
	 * @brief  获取已加载的作业程序名
	 * @param  [out] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
	 * @return  错误码
	 */
	errno_t GetLoadedProgram(char program_name[64]);

	/**
	 * @brief  获取当前机器人作业程序执行的行号
	 * @param  [out] line  行号
	 * @return  错误码
	 */
	errno_t GetCurrentLine(int *line);

	/**
	 * @brief  运行当前加载的作业程序
	 * @return  错误码
	 */
	errno_t ProgramRun();

	/**
	 * @brief  暂停当前运行的作业程序
	 * @return  错误码
	 */
	errno_t ProgramPause();

	/**
	 * @brief  恢复当前暂停的作业程序
	 * @return  错误码
	 */
	errno_t ProgramResume();

	/**
	 * @brief  终止当前运行的作业程序
	 * @return  错误码
	 */
	errno_t ProgramStop();

	/**
	 * @brief  获取机器人作业程序执行状态
	 * @param  [out]  state 1-程序停止或无程序运行，2-程序运行中，3-程序暂停
	 * @return  错误码
	 */
	errno_t GetProgramState(uint8_t *state);

	/**
	 * @brief  配置夹爪
	 * @param  [in] company  夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行
	 * @param  [in] device  设备号，Robotiq(0-2F-85系列)，慧灵(0-NK系列,1-Z-EFG-100)，天机(0-TEG-110)，大寰(0-PGI-140)，知行(0-CTPM2F20)
	 * @param  [in] softvesion  软件版本号，暂不使用，默认为0
	 * @param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
	 * @return  错误码
	 */
	errno_t SetGripperConfig(int company, int device, int softvesion, int bus);

	/**
	 *@brief  获取夹爪配置
	 *@param  [out] company  夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行
	 *@param  [out] device  设备号，Robotiq(0-2F-85系列)，慧灵(0-NK系列,1-Z-EFG-100)，天机(0-TEG-110)，大寰(0-PGI-140)，知行(0-CTPM2F20)
	 *@param  [out] softvesion  软件版本号，暂不使用，默认为0
	 *@param  [out] bus 设备挂在末端总线位置，暂不使用，默认为0
	 *@return  错误码
	 */
	errno_t GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

	/**
	 * @brief  激活夹爪
	 * @param  [in] index  夹爪编号
	 * @param  [in] act  0-复位，1-激活
	 * @return  错误码
	 */
	errno_t ActGripper(int index, uint8_t act);

	/**
	 * @brief  控制夹爪
	 * @param  [in] index  夹爪编号
	 * @param  [in] pos  位置百分比，范围[0~100]
	 * @param  [in] vel  速度百分比，范围[0~100]
	 * @param  [in] force  力矩百分比，范围[0~100]
	 * @param  [in] max_time  最大等待时间，范围[0~30000]，单位ms
	 * @param  [in] block  0-阻塞，1-非阻塞
	 * @return  错误码
	 */
	errno_t MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block);

	/**
	 * @brief  获取夹爪运动状态
	 * @param  [out] fault  0-无错误，1-有错误
	 * @param  [out] staus  0-运动未完成，1-运动完成
	 * @return  错误码
	 */
	errno_t GetGripperMotionDone(uint16_t *fault, uint8_t *status);

	/**
	 * @brief  获取夹爪激活状态
	 * @param  [out] fault  0-无错误，1-有错误
	 * @param  [out] status  bit0~bit15对应夹爪编号0~15，bit=0为未激活，bit=1为激活
	 * @return  错误码
	 */
	errno_t GetGripperActivateStatus(uint16_t *fault, uint16_t *status);

	/**
	 * @brief  获取夹爪位置
	 * @param  [out] fault  0-无错误，1-有错误
	 * @param  [out] position  位置百分比，范围0~100%
	 * @return  错误码
	 */
	errno_t GetGripperCurPosition(uint16_t *fault, uint8_t *position);

	/**
	 * @brief  获取夹爪速度
	 * @param  [out] fault  0-无错误，1-有错误
	 * @param  [out] speed  速度百分比，范围0~100%
	 * @return  错误码
	 */
	errno_t GetGripperCurSpeed(uint16_t *fault, int8_t *speed);

	/**
	 * @brief  获取夹爪电流
	 * @param  [out] fault  0-无错误，1-有错误
	 * @param  [out] current  电流百分比，范围0~100%
	 * @return  错误码
	 */
	errno_t GetGripperCurCurrent(uint16_t *fault, int8_t *current);

	/**
	 * @brief  获取夹爪电压
	 * @param  [out] fault  0-无错误，1-有错误
	 * @param  [out] voltage  电压,单位0.1V
	 * @return  错误码
	 */
	errno_t GetGripperVoltage(uint16_t *fault, int *voltage);

	/**
	 * @brief  获取夹爪温度
	 * @param  [out] fault  0-无错误，1-有错误
	 * @param  [out] temp  温度，单位℃
	 * @return  错误码
	 */
	errno_t GetGripperTemp(uint16_t *fault, int *temp);

	/**
	 * @brief  计算预抓取点-视觉
	 * @param  [in] desc_pos  抓取点笛卡尔位姿
	 * @param  [in] zlength   z轴偏移量
	 * @param  [in] zangle    绕z轴旋转偏移量
	 * @param  [out] pre_pos  预抓取点
	 * @return  错误码
	 */
	errno_t ComputePrePick(DescPose *desc_pos, double zlength, double zangle, DescPose *pre_pos);

	/**
	 * @brief  计算撤退点-视觉
	 * @param  [in] desc_pos  抓取点笛卡尔位姿
	 * @param  [in] zlength   z轴偏移量
	 * @param  [in] zangle    绕z轴旋转偏移量
	 * @param  [out] post_pos 撤退点
	 * @return  错误码
	 */
	errno_t ComputePostPick(DescPose *desc_pos, double zlength, double zangle, DescPose *post_pos);

	/**
	 * @brief  配置力传感器
	 * @param  [in] company  力传感器厂商，17-坤维科技，19-航天十一院，20-ATI传感器，21-中科米点，22-伟航敏芯
	 * @param  [in] device  设备号，坤维(0-KWR75B)，航天十一院(0-MCS6A-200-4)，ATI(0-AXIA80-M8)，中科米点(0-MST2010)，伟航敏芯(0-WHC6L-YB-10A)
	 * @param  [in] softvesion  软件版本号，暂不使用，默认为0
	 * @param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
	 * @return  错误码
	 */
	errno_t FT_SetConfig(int company, int device, int softvesion, int bus);

	/**
	 * @brief  获取力传感器配置
	 * @param  [out] company  力传感器厂商，待定
	 * @param  [out] device  设备号，暂不使用，默认为0
	 * @param  [out] softvesion  软件版本号，暂不使用，默认为0
	 * @param  [out] bus 设备挂在末端总线位置，暂不使用，默认为0
	 * @return  错误码
	 */
	errno_t FT_GetConfig(int *company, int *device, int *softvesion, int *bus);

	/**
	 * @brief  力传感器激活
	 * @param  [in] act  0-复位，1-激活
	 * @return  错误码
	 */
	errno_t FT_Activate(uint8_t act);

	/**
	 * @brief  力传感器校零
	 * @param  [in] act  0-去除零点，1-零点矫正
	 * @return  错误码
	 */
	errno_t FT_SetZero(uint8_t act);

	/**
	 * @brief  设置力传感器参考坐标系
	 * @param  [in] ref  0-工具坐标系，1-基坐标系
	 * @return  错误码
	 */
	errno_t FT_SetRCS(uint8_t ref);

	/**
	 * @brief  负载重量辨识记录
	 * @param  [in] id  传感器坐标系编号，范围[1~14]
	 * @return  错误码
	 */
	errno_t FT_PdIdenRecord(int id);

	/**
	 * @brief  负载重量辨识计算
	 * @param  [out] weight  负载重量，单位kg
	 * @return  错误码
	 */
	errno_t FT_PdIdenCompute(float *weight);

	/**
	 * @brief  负载质心辨识记录
	 * @param  [in] id  传感器坐标系编号，范围[1~14]
	 * @param  [in] index 点编号，范围[1~3]
	 * @return  错误码
	 */
	errno_t FT_PdCogIdenRecord(int id, int index);

	/**
	 * @brief  负载质心辨识计算
	 * @param  [out] cog  负载质心，单位mm
	 * @return  错误码
	 */
	errno_t FT_PdCogIdenCompute(DescTran *cog);

	/**
	 * @brief  获取参考坐标系下力/扭矩数据
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
	 * @return  错误码
	 */
	errno_t FT_GetForceTorqueRCS(uint8_t flag, ForceTorque *ft);

	/**
	 * @brief  获取力传感器原始力/扭矩数据
	 * @param  [in] flag 0-阻塞，1-非阻塞
	 * @param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
	 * @return  错误码
	 */
	errno_t FT_GetForceTorqueOrigin(uint8_t flag, ForceTorque *ft);

	/**
	 * @brief  碰撞守护
	 * @param  [in] flag 0-关闭碰撞守护，1-开启碰撞守护
	 * @param  [in] sensor_id 力传感器编号
	 * @param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检测
	 * @param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
	 * @param  [in] max_threshold 最大阈值
	 * @param  [in] min_threshold 最小阈值
	 * @note   力/扭矩检测范围：(ft-min_threshold, ft+max_threshold)
	 * @return  错误码
	 */
	errno_t FT_Guard(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float max_threshold[6], float min_threshold[6]);

	/**
	 * @brief  恒力控制
	 * @param  [in] flag 0-关闭恒力控制，1-开启恒力控制
	 * @param  [in] sensor_id 力传感器编号
	 * @param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检测
	 * @param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
	 * @param  [in] ft_pid 力pid参数，力矩pid参数
	 * @param  [in] adj_sign 自适应启停控制，0-关闭，1-开启
	 * @param  [in] ILC_sign ILC启停控制， 0-停止，1-训练，2-实操
	 * @param  [in] max_dis 最大调整距离，单位mm
	 * @param  [in] max_ang 最大调整角度，单位deg
	 * @return  错误码
	 */
	errno_t FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float ft_pid[6], uint8_t adj_sign, uint8_t ILC_sign, float max_dis, float max_ang);

	/**
	 * @brief  螺旋线探索
	 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
	 * @param  [in] dr 每圈半径进给量
	 * @param  [in] ft 插入动作触发力，单位N
	 * @param  [in] max_t_ms 最大探索时间，单位ms
	 * @param  [in] max_vel 最大线速度，单位mm/s
	 * @return  错误码
	 */
	errno_t FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel);

	/**
	 * @brief  旋转插入
	 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
	 * @param  [in] angVelRot 旋转角速度，单位deg/s
	 * @param  [in] ft  力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
	 * @param  [in] max_angle 最大旋转角度，单位deg
	 * @param  [in] orn 力/扭矩方向，1-沿z轴方向，2-绕z轴方向
	 * @param  [in] max_angAcc 最大旋转加速度，单位deg/s^2，暂不使用，默认为0
	 * @param  [in] rotorn  旋转方向，1-顺时针，2-逆时针
	 * @return  错误码
	 */
	errno_t FT_RotInsertion(int rcs, float angVelRot, float ft, float max_angle, uint8_t orn, float max_angAcc, uint8_t rotorn);

	/**
	 * @brief  直线插入
	 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
	 * @param  [in] ft  动作终止力阈值，单位N，范围[0~100]
	 * @param  [in] lin_v 直线速度，单位mm/s
	 * @param  [in] lin_a 直线加速度，单位mm/s^2，暂不使用
	 * @param  [in] max_dis 最大插入距离，单位mm
	 * @param  [in] linorn  插入方向，0-负方向，1-正方向
	 * @return  错误码
	 */
	errno_t FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, uint8_t linorn);

	/**
	 * @brief  表面定位
	 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
	 * @param  [in] dir  移动方向，1-正方向，2-负方向
	 * @param  [in] axis 移动轴，1-x轴，2-y轴，3-z轴
	 * @param  [in] lin_v 探索直线速度，单位mm/s
	 * @param  [in] lin_a 探索直线加速度，单位mm/s^2，暂不使用，默认为0
	 * @param  [in] max_dis 最大探索距离，单位mm
	 * @param  [in] ft  动作终止力阈值，单位N
	 * @return  错误码
	 */
	errno_t FT_FindSurface(int rcs, uint8_t dir, uint8_t axis, float lin_v, float lin_a, float max_dis, float ft);

	/**
	 * @brief  计算中间平面位置开始
	 * @return  错误码
	 */
	errno_t FT_CalCenterStart();

	/**
	 * @brief  计算中间平面位置结束
	 * @param  [out] pos 中间平面位姿
	 * @return  错误码
	 */
	errno_t FT_CalCenterEnd(DescPose *pos);

	/**
	 * @brief  柔顺控制开启
	 * @param  [in] p 位置调节系数或柔顺系数
	 * @param  [in] force 柔顺开启力阈值，单位N
	 * @return  错误码
	 */
	errno_t FT_ComplianceStart(float p, float force);

	/**
	 * @brief  柔顺控制关闭
	 * @return  错误码
	 */
	errno_t FT_ComplianceStop();

	/**
	 * @brief 负载辨识初始化
	 * @return 错误码
	 */
	errno_t LoadIdentifyDynFilterInit();

	/**
	 * @brief 负载辨识初始化
	 * @return 错误码
	 */
	errno_t LoadIdentifyDynVarInit();

	/**
	 * @brief 负载辨识主程序
	 * @param [in] joint_torque 关节扭矩
	 * @param [in] joint_pos 关节位置
	 * @param [in] t 采样周期
	 * @return 错误码
	 */
	errno_t LoadIdentifyMain(double joint_torque[6], double joint_pos[6], double t);

	/**
	 * @brief 获取负载辨识结果
	 * @param [in] gain
	 * @param [out] weight 负载重量
	 * @param [out] cog 负载质心
	 * @return 错误码
	 */
	errno_t LoadIdentifyGetResult(double gain[12], double *weight, DescTran *cog);

	/**
	 * @brief 传动带启动、停止
	 * @param [in] status 状态，1-启动，0-停止
	 * @return 错误码
	 */
	errno_t ConveyorStartEnd(uint8_t status);

	/**
	 * @brief 记录IO检测点
	 * @return 错误码
	 */
	errno_t ConveyorPointIORecord();

	/**
	 * @brief 记录A点
	 * @return 错误码
	 */
	errno_t ConveyorPointARecord();

	/**
	 * @brief 记录参考点
	 * @return 错误码
	 */
	errno_t ConveyorRefPointRecord();

	/**
	 * @brief 记录B点
	 * @return 错误码
	 */
	errno_t ConveyorPointBRecord();

	/**
	 * @brief 传送带工件IO检测
	 * @param [in] max_t 最大检测时间，单位ms
	 * @return 错误码
	 */
	errno_t ConveyorIODetect(int max_t);

	/**
	 * @brief 获取物体当前位置
	 * @param [in] mode
	 * @return 错误码
	 */
	errno_t ConveyorGetTrackData(int mode);

	/**
	 * @brief 传动带跟踪开始
	 * @param [in] status 状态，1-启动，0-停止
	 * @return 错误码
	 */
	errno_t ConveyorTrackStart(uint8_t status);

	/**
	 * @brief 传动带跟踪停止
	 * @return 错误码
	 */
	errno_t ConveyorTrackEnd();

	/**
	 * @brief 传动带参数配置
	 * @param [in]
	 * @return 错误码
	 */
	errno_t ConveyorSetParam(float para[6]);

	/**
	 * @brief 传动带抓取点补偿
	 * @param [in] cmp 补偿位置
	 * @return 错误码
	 */
	errno_t ConveyorCatchPointComp(double cmp[3]);

	/**
	 * @brief 直线运动
	 * @param [in] status 状态，1-启动，0-停止
	 * @return 错误码
	 */
	errno_t TrackMoveL(char name[32], int tool, int wobj, float vel, float acc, float ovl, float blendR, uint8_t flag, uint8_t type);

	/**
	 * @brief 获取SSH公钥
	 * @param [out] keygen 公钥
	 * @return 错误码
	 */
	errno_t GetSSHKeygen(char keygen[1024]);

	/**
	 * @brief 下发SCP指令
	 * @param [in] mode 0-上传（上位机->控制器），1-下载（控制器->上位机）
	 * @param [in] sshname 上位机用户名
	 * @param [in] sship 上位机ip地址
	 * @param [in] usr_file_url 上位机文件路径
	 * @param [in] robot_file_url 机器人控制器文件路径
	 * @return 错误码
	 */
	errno_t SetSSHScpCmd(int mode, char sshname[32], char sship[32], char usr_file_url[128], char robot_file_url[128]);

	/**
	 * @brief 计算指定路径下文件的MD5值
	 * @param [in] file_path 文件路径包含文件名，默认Traj文件夹路径为:"/fruser/traj/",如"/fruser/traj/trajHelix_aima_1.txt"
	 * @param [out] md5 文件MD5值
	 * @return 错误码
	 */
	errno_t ComputeFileMD5(char file_path[256], char md5[256]);

	/**
	 * @brief 获取机器人急停状态
	 * @param [out] state 急停状态，0-非急停，1-急停
	 * @return 错误码
	 */
	errno_t GetRobotEmergencyStopState(uint8_t *state);

	/**
	 * @brief 获取SDK与机器人的通讯状态
	 * @param [out]  state 通讯状态，0-通讯正常，1-通讯异常
	 */
	errno_t GetSDKComState(int *state);

	/**
	 * @brief 获取安全停止信号
	 * @param [out]  si0_state 安全停止信号SI0
	 * @param [out]  si1_state 安全停止信号SI1
	 */
	errno_t GetSafetyStopState(uint8_t *si0_state, uint8_t *si1_state);

	/**
	 * @brief 获取机器人软件版本
	 * @param[out]	robotModel 机器人型号
	 * @param[out]	webversion web版本
	 * @param[out]	controllerVersion 控制器版本
	 * @return 错误码
	 */
	errno_t GetSoftwareVersion(char robotModel[64], char webVersion[64], char controllerVersion[64]);

	/**
	 * @brief 获取机器人硬件版本
	 * @param[out] ctrlBoxBoardversion 控制箱载板硬件版本
	 * @param[out] driver1version 驱动器1硬件版本
	 * @param[out] driver2version 驱动器2硬件版本
	 * @param[out] driver3version 驱动器3硬件版本
	 * @param[out] driver4version 驱动器4硬件版本
	 * @param[out] driver5version 驱动器5硬件版本
	 * @param[out] driver6version 驱动器6硬件版本
	 * @param[out] endBoardversion 未端版硬件版本
	 */
	errno_t GetHardwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128],
							   char driver3version[128], char driver4version[128], char driver5version[128],
							   char driver6version[128], char endBoardversion[128]);

	/**
	 * @brief 获取机器人固件版本
	 * @param[out] ctrlBoxBoardversion 控制箱载板固件版本
	 * @param[out] driver1version 驱动器1固件版本
	 * @param[out] driver2version 驱动器2固件版本
	 * @param[out] driver3version 驱动器3固件版本
	 * @param[out] driver4version 驱动器4固件版本
	 * @param[out] driver5version 驱动器5固件版本
	 * @param[out] driver6version 驱动器6固件版本
	 * @param[out] endBoardversion 未端版固件版本
	 */
	errno_t GetFirmwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128],
							   char driver3version[128], char driver4version[128], char driver5version[128],
							   char driver6version[128], char endBoardversion[128]);

	/**
	 * @brief 获取机器人DH参数补偿值
	 * @param [out] dhCompensation 机器人DH参数补偿值(mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
	 * @return 错误码
	 */
	errno_t GetDHCompensation(double dhCompensation[6]);

	/**
	 * @brief 下载点位表数据库
	 * @param [in] pointTableName 要下载的点位表名称    pointTable1.db
	 * @param [in] saveFilePath 下载点位表的存储路径   C://test/
	 * @return 错误码
	 */
	errno_t PointTableDownLoad(const std::string &pointTableName, const std::string &saveFilePath);

	/**
	 * @brief 上传点位表数据库
	 * @param [in] pointTableFilePath 上传点位表的全路径名   C://test/pointTable1.db
	 * @return 错误码
	 */
	errno_t PointTableUpLoad(const std::string &pointTableFilePath);

	/**
	 * @brief 点位表更新lua文件
	 * @param [in] pointTableName 要切换的点位表名称   "pointTable1.db",当点位表为空，即""时，表示将lua程序更新为未应用点位表的初始程序
	 * @param [in] luaFileName 要更新的lua文件名称   "testPointTable.lua"
	 * @param [out] errorStr 切换点位表错误信息
	 * @return 错误码
	 */
	errno_t PointTableUpdateLua(const std::string &pointTableName, const std::string &luaFileName);

	/**
	 * @brief 焊接开始
	 * @param [in] ioType io类型 0-控制器IO； 1-扩展IO
	 * @param [in] arcNum 焊机配置文件编号
	 * @param [in] timeout 起弧超时时间
	 * @return 错误码
	 */
	errno_t ARCStart(int ioType, int arcNum, int timeout);

	/**
	 * @brief 焊接结束
	 * @param [in] ioType io类型 0-控制器IO； 1-扩展IO
	 * @param [in] arcNum 焊机配置文件编号
	 * @param [in] timeout 熄弧超时时间
	 * @return 错误码
	 */
	errno_t ARCEnd(int ioType, int arcNum, int timeout);

	/**
	 * @brief 设置焊接电流与输出模拟量对应关系
	 * @param [in] currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
	 * @param [in] currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
	 * @param [in] outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
	 * @param [in] outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
	 * @return 错误码
	 */
	errno_t WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax);

	/**
	 * @brief 设置焊接电压与输出模拟量对应关系
	 * @param [in] weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
	 * @param [in] weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
	 * @param [in] outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
	 * @param [in] outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
	 * @return 错误码
	 */
	errno_t WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax);

	/**
	 * @brief 获取焊接电流与输出模拟量对应关系
	 * @param [out] currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
	 * @param [out] currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
	 * @param [out] outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
	 * @param [out] outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
	 * @return 错误码
	 */
	errno_t WeldingGetCurrentRelation(double *currentMin, double *currentMax, double *outputVoltageMin, double *outputVoltageMax);

	/**
	 * @brief 获取焊接电压与输出模拟量对应关系
	 * @param [out] weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
	 * @param [out] weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
	 * @param [out] outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
	 * @param [out] outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
	 * @return 错误码
	 */
	errno_t WeldingGetVoltageRelation(double *weldVoltageMin, double *weldVoltageMax, double *outputVoltageMin, double *outputVoltageMax);

	/**
	 * @brief 设置焊接电流
	 * @param [in] ioType 控制IO类型 0-控制箱IO；1-扩展IO
	 * @param [in] current 焊接电流值(A)
	 * @param [in] AOIndex 焊接电流控制箱模拟量输出端口(0-1)
	 * @return 错误码
	 */
	errno_t WeldingSetCurrent(int ioType, double current, int AOIndex);

	/**
	 * @brief 设置焊接电压
	 * @param [in] ioType 控制IO类型 0-控制箱IO；1-扩展IO
	 * @param [in] voltage 焊接电压值(A)
	 * @param [in] AOIndex 焊接电压控制箱模拟量输出端口(0-1)
	 * @return 错误码
	 */
	errno_t WeldingSetVoltage(int ioType, double voltage, int AOIndex);

	/**
	 * @brief 设置摆动参数
	 * @param [in] weaveNum 摆焊参数配置编号
	 * @param [in] weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
	 * @param [in] weaveFrequency 摆动频率(Hz)
	 * @param [in] weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
	 * @param [in] weaveRange 摆动幅度(mm)
	 * @param [in] weaveLeftRange 垂直三角摆动左弦长度(mm)
	 * @param [in] weaveRightRange 垂直三角摆动右弦长度(mm)
	 * @param [in] additionalStayTime 垂直三角摆动垂三角点停留时间(mm)
	 * @param [in] weaveLeftStayTime 摆动左停留时间(ms)
	 * @param [in] weaveRightStayTime 摆动右停留时间(ms)
	 * @param [in] weaveCircleRadio 圆形摆动-回调比率(0-100%)
	 * @param [in] weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
	 * @return 错误码
	 */
	errno_t WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, 
                            int weaveIncStayTime, double weaveRange, double weaveLeftRange, 
                            double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, 
                            int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

	/**
	 * @brief 即时设置摆动参数
	 * @param [in] weaveNum 摆焊参数配置编号
	 * @param [in] weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
	 * @param [in] weaveFrequency 摆动频率(Hz)
	 * @param [in] weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
	 * @param [in] weaveRange 摆动幅度(mm)
	 * @param [in] weaveLeftStayTime 摆动左停留时间(ms)
	 * @param [in] weaveRightStayTime 摆动右停留时间(ms)
	 * @param [in] weaveCircleRadio 圆形摆动-回调比率(0-100%)
	 * @param [in] weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
	 * @return 错误码
	 */
	errno_t WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

	/**
	 * @brief 摆动开始
	 * @param [in] weaveNum 摆焊参数配置编号
	 * @return 错误码
	 */
	errno_t WeaveStart(int weaveNum);

	/**
	 * @brief 摆动结束
	 * @param [in] weaveNum 摆焊参数配置编号
	 * @return 错误码
	 */
	errno_t WeaveEnd(int weaveNum);

	/**
	 * @brief 正向送丝
	 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
	 * @param [in] wireFeed 送丝控制  0-停止送丝；1-送丝
	 * @return 错误码
	 */
	errno_t SetForwardWireFeed(int ioType, int wireFeed);

	/**
	 * @brief 反向送丝
	 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
	 * @param [in] wireFeed 送丝控制  0-停止送丝；1-送丝
	 * @return 错误码
	 */
	errno_t SetReverseWireFeed(int ioType, int wireFeed);

	/**
	 * @brief 送气
	 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
	 * @param [in] airControl 送气控制  0-停止送气；1-送气
	 * @return 错误码
	 */
	errno_t SetAspirated(int ioType, int airControl);

	/**
	 *@brief段焊开始
	 *@param[in]startDesePos 起始点笛卡尔位置
	 *@param[in]endDesePos 结束点笛卡尔位姿
	 *@param[in]startJPos 起始点关节位姿
	 *@param[in]endJPos 结束点关节位姿
	 *@param[in]weldLength 焊接段长度(mm)
	 *@param[in]noWeldLength 非焊接段长度(mm)
	 *@param[in]weldIOType 焊接IO类型(0-控制箱IO；1-扩展IO)
	 *@param[in]arcNum 焊机配置文件编号
	 *@param[in]weldTimeout 起/收弧超时时间
	 *@param[in]isWeave 是否摆动
	 *@param[in]weaveNum 摆焊参数配置编号
	 *@param[in]tool 工具号
	 *@param[in]user 工件号
	 *@param[in]vel 速度百分比，范围[0~100]
	 *@param[in]acc 加速度百分比，范围[0~100],暂不开放
	 *@param[in]ovl 速度缩放因子，范围[0~100]
	 *@param[in]blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
	 *@param[in]epos 扩展轴位置，单位mm
 	 *@param[in]search 0-不焊丝寻位，1-焊丝寻位
	 *@param[in]offset_flag 0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param[in]offset_pos 位姿偏移量
	 *@return 错误码
	 */
	errno_t SegmentWeldStart(DescPose *startDesePos, DescPose *endDesePos, JointPos *startJPos, JointPos *endJPos,
							 double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout,
							 bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR,
							 ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

	/**
	 * @brief 初始化日志参数;
	 * @param output_model：输出模式，0-直接输出；1-缓冲输出；2-异步输出;
	 * @param file_path: 文件保存路径+名称，,长度上限256，名称必须是xxx.log的形式，比如/home/fr/linux/fairino.log;
	 * @param file_num：滚动存储的文件数量，1~20个.单个文件上限50M;
	 * @return errno_t 错误码;
	 */
	errno_t LoggerInit(int output_model = 0, std::string file_path = "", int file_num = 5);

    /**
     * @brief 设置日志过滤等级;
     * @param lvl: 过滤等级值，值越小输出日志越少，默认值是1. 1-error, 2-warnning, 3-inform, 4-debug;
    */
    void SetLoggerLevel(int lvl = 1);

	/**
	 * @brief 下载Lua文件
	 * @param [in] fileName 要下载的lua文件名“test.lua”
	 * @param [in] savePath 保存文件本地路径“D://Down/”
	 * @return 错误码
	 */
	errno_t LuaDownLoad(std::string fileName, std::string savePath);

	/**
	 * @brief 上传Lua文件
	 * @param [in] filePath 本地lua文件路径名
	 * @return 错误码
	 */
	errno_t LuaUpload(std::string filePath);

	/**
	 * @brief 删除Lua文件
	 * @param [in] fileName 要删除的lua文件名“test.lua”
	 * @return 错误码
	 */
	errno_t LuaDelete(std::string fileName);

	/**
	 * @brief 获取当前所有lua文件名称
	 * @param [out] luaNames lua文件名列表
	 * @return 错误码
	 */
	errno_t GetLuaList(std::list<std::string>* luaNames);
	/**
	 * @brief 设置485扩展轴参数
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [in] servoCompany 伺服驱动器厂商，1-戴纳泰克
	 * @param [in] servoModel 伺服驱动器型号，1-FD100-750C
	 * @param [in] servoSoftVersion 伺服驱动器软件版本，1-V1.0
	 * @param [in] servoResolution 编码器分辨率
	 * @param [in] axisMechTransRatio 机械传动比
	 * @return 错误码
	 */
	errno_t AuxServoSetParam(int servoId, int servoCompany, int servoModel,
							 int servoSoftVersion, int servoResolution, double axisMechTransRatio);
	/**
	 * @brief 获取485扩展轴配置参数
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [out] servoCompany 伺服驱动器厂商，1-戴纳泰克
	 * @param [out] servoModel 伺服驱动器型号，1-FD100-750C
	 * @param [out] servoSoftVersion 伺服驱动器软件版本，1-V1.0
	 * @param [out] servoResolution 编码器分辨率
	 * @param [out] axisMechTransRatio 机械传动比
	 * @return 错误码
	 */
	errno_t AuxServoGetParam(int servoId, int* servoCompany, int* servoModel,
                                  int* servoSoftVersion, int* servoResolution, double* axisMechTransRatio);

	/**
	 * @brief 设置485扩展轴使能/去使能
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [in] status 使能状态，0-去使能， 1-使能
	 * @return 错误码
	 */
	errno_t AuxServoEnable(int servoId, int status);

	/**
	 * @brief 设置485扩展轴控制模式
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [in] mode 控制模式，0-位置模式，1-速度模式
	 * @return 错误码
	 */
	errno_t AuxServoSetControlMode(int servoId, int mode);
	/**
	 * @brief 设置485扩展轴目标位置(位置模式)
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [in] pos 目标位置，mm或°
	 * @param [in] speed 目标速度，mm/s或°/s
	 * @return 错误码
	 */
	errno_t AuxServoSetTargetPos(int servoId, double pos, double speed);

	/**
	 * @brief 设置485扩展轴目标速度(速度模式)
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [in] speed 目标速度，mm/s或°/s
	 * @return 错误码
	 */
	errno_t AuxServoSetTargetSpeed(int servoId, double speed);

	/**
	 * @brief 设置485扩展轴目标转矩(力矩模式)
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [in] torque 目标力矩，Nm
	 * @return 错误码
	 */
	errno_t AuxServoSetTargetTorque(int servoId, double torque);

	/**
	 * @brief 设置485扩展轴回零
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [in] mode 回零模式，0-当前位置回零；1-限位回零
	 * @param [in] searchVel 回零速度，mm/s或°/s
	 * @param [in] latchVel 箍位速度，mm/s或°/s
	 * @return 错误码
	 */
	errno_t AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);

	/**
	 * @brief 清除485扩展轴错误信息
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @return 错误码
	 */
	errno_t AuxServoClearError(int servoId);

	/**
	 * @brief 获取485扩展轴伺服状态
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @param [out] servoErrCode 伺服驱动器故障码
	 * @param [out] servoState 伺服驱动器状态[十进制数转为二进制，bit0-bit5：伺服使能-伺服运行-正限位触发-负限位触发-定位完成-回零完成]
	 * @param [out] servoPos 伺服当前位置 mm或°
	 * @param [out] servoSpeed 伺服当前速度 mm/s或°/s
	 * @param [out] servoTorque 伺服当前转矩Nm
	 * @return 错误码
	 */
	errno_t AuxServoGetStatus(int servoId, int* servoErrCode, int* servoState, double* servoPos,
                                   double* servoSpeed, double* servoTorque);

	/**
	 * @brief 设置状态反馈中485扩展轴数据轴号
	 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
	 * @return 错误码
	 */
	errno_t AuxServosetStatusID(int servoId);

	/**
	 * @brief 获取机器人实时状态结构体
	 * @param [out] pkg 机器人实时状态结构体
	 * @return 错误码
	 */
	errno_t GetRobotRealTimeState(ROBOT_STATE_PKG *pkg);
	/**
	 * @brief 设置机器人外设协议
	 * @param [out] protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
	 * @return 错误码
	 */
	errno_t GetExDevProtocol(int *protocol);

	/**
	 * @brief 获取机器人外设协议
	 * @param [in] protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
	 * @return 错误码
	 */
	errno_t SetExDevProtocol(int protocol);

	/**
	 *@brief  机器人接口类析构函数
	 */
	~FRRobot();

private:
	static void RobotStateRoutineThread();
	static void RobotInstCmdSendRoutineThread();
	static void RobotInstCmdRecvRoutineThread();
	static void RobotTaskRoutineThread();
	char serverUrl[64];

	bool rpc_done = false;

	/**
	 * @brief 下载文件
	 * @param [in] fileType 文件类型    0-lua文件
	 * @param [in] fileName 文件名称    “test.lua”
	 * @param [in] saveFilePath 保存文件路径    “C：//test/”
	 * @return 错误码
	 */
	errno_t FileDownLoad(int fileType, std::string fileName, std::string saveFilePath);

	/**
	 * @brief 上传文件
	 * @param [in] fileType 文件类型    0-lua文件
	 * @param [in] fileName 文件名称    “test.lua”
	 * @param [in] upLoadFilePath 保存文件路径    “C：//test/”
	 * @return 错误码
	 */
	errno_t FileUpLoad(int fileType, std::string filePath);

	/**
	 * @brief 上传文件
	 * @param [in] fileType 文件类型    0-lua文件
	 * @param [in] fileName 文件名称    “test.lua”
	 * @return 错误码
	 */
	errno_t FileDelete(int fileType, std::string fileName);

	/* 根据字符分割字符串 */
	std::vector<std::string> split(const std::string& s, char delim);

	/* 根据字符串分割字符串 */
	std::vector<std::string> split(std::string s, std::string delimiter);
};

#endif
