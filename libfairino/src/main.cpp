#include "robot.h"
#ifdef WIN32
#include <string.h>
#include <windows.h>
#include <chrono>
#else
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#endif

#include <cmath>
#include <chrono>
#include <thread>
#include "md5.hpp"
#include "FRTcpClient.h"
#include <bitset>

using namespace std;
using std::chrono::duration_cast;
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;

int WeldingProcessParamConfig(FRRobot* robot)
{
    robot->WeldingSetProcessParam(1, 177, 27, 1000, 178, 28, 176, 26, 1000);
    robot->WeldingSetProcessParam(2, 188, 28, 555, 199, 29, 133, 23, 333);

    double startCurrent = 0;
    double startVoltage = 0;
    double startTime = 0;
    double weldCurrent = 0;
    double weldVoltage = 0;
    double endCurrent = 0;
    double endVoltage = 0;
    double endTime = 0;

    robot->WeldingGetProcessParam(1, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    cout << "the Num 1 process param is " << startCurrent << "  " << startVoltage<< "  " <<startTime<<"  " <<weldCurrent<< "  " <<weldVoltage<< "  " <<endCurrent<< "  " <<endVoltage<< "  " <<endTime << endl;

    robot->WeldingGetProcessParam(2, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    cout << "the Num 2 process param is " << startCurrent << "  " << startVoltage << "  " << startTime << "  " << weldCurrent << "  " << weldVoltage << "  " << endCurrent << "  " << endVoltage << "  " << endTime << endl;
    return 0;
}

int SetExtDIOFuntion(FRRobot* robot)
{
    robot->SetArcStartExtDoNum(10);
    robot->SetAirControlExtDoNum(20);
    robot->SetWireForwardFeedExtDoNum(30);
    robot->SetWireReverseFeedExtDoNum(40);

    robot->SetWeldReadyExtDiNum(50);
    robot->SetArcDoneExtDiNum(60);
    robot->SetExtDIWeldBreakOffRecover(70, 80);
    return 0;
}

int IOReset(FRRobot* robot)
{
    int resetFlag = 0;
    int rtn = robot->SetOutputResetCtlBoxDO(resetFlag);
    robot->SetOutputResetCtlBoxAO(resetFlag);
    robot->SetOutputResetAxleDO(resetFlag);
    robot->SetOutputResetAxleAO(resetFlag);
    robot->SetOutputResetExtDO(resetFlag);
    robot->SetOutputResetExtAO(resetFlag);
    robot->SetOutputResetSmartToolDO(resetFlag);
    return 0;
}

int StaticCollision(FRRobot* robot)
{
    robot->SetCollisionDetectionMethod(0);
    robot->SetStaticCollisionOnOff(1);
    robot->Sleep(5000);
    robot->SetStaticCollisionOnOff(0);
    return 0;
}

int DragControl(FRRobot* robot)
{
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    int rtn = robot->EndForceDragControl(1, 0, 0, 0, 1, M, B, K, F, 50, 100);
    printf("force drag control start rtn is %d\n", rtn);
    robot->Sleep(5000);

    rtn = robot->EndForceDragControl(0, 0, 0, 0, 1, M, B, K, F, 50, 100);
    printf("force drag control end rtn is %d\n", rtn);

    rtn = robot->ResetAllError();
    printf("ResetAllError rtn is %d\n", rtn);

    robot->EndForceDragControl(1, 0, 0, 0, 1, M, B, K, F, 50, 100);
    printf("force drag control start again rtn is %d\n", rtn);
    robot->Sleep(5000);

    rtn = robot->EndForceDragControl(0, 0, 0, 0, 1, M, B, K, F, 50, 100);
    printf("force drag control end again rtn is %d\n", rtn);

}

int SixDiaDrag(FRRobot* robot)
{
    robot->DragTeachSwitch(1);
    vector <double> lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
    vector <double> KGain = { 0, 0, 0, 0, 0, 0 };
    vector <double> BGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
    robot->ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000, 180);

    robot->Sleep(5000);

    robot->DragTeachSwitch(0);
    robot->ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000, 180);

    return 0;
}

int RobotGetFTDragState(FRRobot* robot)
{
    int dragState = 0;
    int sixDimensionalDragState = 0;
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);
    robot->Sleep(1000);
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot->EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);

    robot->Sleep(1000);
    robot->EndForceDragControl(0, 0, 0, 0, M, B, K, F, 50, 100);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);
    robot->Sleep(1000);

    robot->DragTeachSwitch(1);
    vector <double> lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
    vector <double> KGain = { 0, 0, 0, 0, 0, 0 };
    vector <double> BGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
    robot->ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000, 180);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);
    robot->Sleep(1000);
    robot->DragTeachSwitch(0);
    robot->ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000, 180);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);

    return 0;
}

int FTAutoOn(FRRobot* robot)
{
    robot->SetForceSensorDragAutoFlag(1);
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot->EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100);
    return 0;
}

int FTAutoOff(FRRobot* robot)
{
    robot->SetForceSensorDragAutoFlag(0);
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot->EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100);
    return 0;
}

int FTLoadSetGet(FRRobot* robot)
{
    robot->SetForceSensorPayload(0.824);
    robot->SetForceSensorPayloadCog(0.778, 2.554, 48.765);
    double weight = 0;
    double x = 0, y = 0, z = 0;
    robot->GetForceSensorPayload(weight);
    robot->GetForceSensorPayloadCog(x, y, z);
    printf("the FT load is   %lf,  %lf  %lf  %lf\n", weight, x, y, z);


    return 0;
}

int FTAutoComputeLoad(FRRobot* robot)
{
    robot->SetForceSensorPayload(0);
    robot->SetForceSensorPayloadCog(0, 0, 0);
    double weight = 0;
    DescTran tran = {};
    robot->ForceSensorAutoComputeLoad(weight, tran);
    cout << "the result is weight " << weight << " pos is  " << tran.x << "  " << tran.y << "  " << tran.z << endl;
    return 0;
}

int PowerLimitOn(FRRobot* robot)
{
    robot->DragTeachSwitch(1);
    robot->SetPowerLimit(1, 2);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot->GetJointTorques(1, torques);

    int count = 100;
    robot->ServoJTStart(); //   #servoJT开始
    int error = 0;
    while (count > 0)
    {
        torques[0] = torques[0] + 0.1;//  #每次1轴增加0.1NM，运动100次
        error = robot->ServoJT(torques, 0.001);  //# 关节空间伺服模式运动
        count = count - 1;
        robot->Sleep(1);
    }

    error = robot->ServoJTEnd();  //#伺服运动结束
    return 0;
}

int TestServoJ1(FRRobot* robot)
{
    robot->DragTeachSwitch(1);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot->GetJointTorques(1, torques);

    int count = 100;
    robot->ServoJTStart(); //   #servoJT开始
    int error = 0;
    while (count > 0)
    {
        error = robot->ServoJT(torques, 0.008);  //# 关节空间伺服模式运动
        count = count - 1;
        robot->Sleep(1);
    }

    error = robot->ServoJTEnd();  //#伺服运动结束
    robot->DragTeachSwitch(1);
    return 0;
}

int PowerLimitOff(FRRobot* robot)
{
    robot->DragTeachSwitch(1);
    robot->SetPowerLimit(0, 2);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot->GetJointTorques(1, torques);

    int count = 100;
    robot->ServoJTStart(); //   #servoJT开始
    int error = 0;
    while (count > 0)
    {
        torques[0] = torques[0] + 0.1;//  #每次1轴增加0.1NM，运动100次
        error = robot->ServoJT(torques, 0.001);  //# 关节空间伺服模式运动
        count = count - 1;
        robot->Sleep(1);
    }

    error = robot->ServoJTEnd();  //#伺服运动结束
    return 0;
}

int WeaveSim(FRRobot* robot)
{
    DescPose startdescPose = { 238.209, -403.633, 251.291, 177.222, -1.433, 133.675 };
    JointPos startjointPos = { -48.728, -86.235, -95.288, -90.025, 92.715, 87.595 };
    DescPose enddescPose = { 238.207, -596.305, 251.294, 177.223, -1.432, 133.675 };
    JointPos endjointPos = { -60.240, -110.743, -66.784, -94.531, 92.351, 76.078 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot->WeaveStartSim(0);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot->WeaveEndSim(0);
    return 0;
}

int WeaveInspect(FRRobot* robot)
{
    DescPose startdescPose = { 238.209, -403.633, 251.291, 177.222, -1.433, 133.675 };
    JointPos startjointPos = { -48.728, -86.235, -95.288, -90.025, 92.715, 87.595 };
    DescPose enddescPose = { 238.207, -596.305, 251.294, 177.223, -1.432, 133.675 };
    JointPos endjointPos = { -60.240, -110.743, -66.784, -94.531, 92.351, 76.078 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot->WeaveInspectStart(0);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot->WeaveInspectEnd(0);
    return 0;
}

int WeldTraceControl(FRRobot* robot)
{
    DescPose startdescPose = { -583.168, 325.637, 1.176, 75.262, 0.978, -3.571 };
    JointPos startjointPos = { -49.049, -77.203, 136.826, -189.074, -79.407, -11.811 };
    
    DescPose enddescPose = { -559.439, 420.491, 32.252, 77.745, 1.460, -10.130 };
    JointPos endjointPos = { -54.986, -77.639, 131.865, -185.707, -80.916, -12.218 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->WeldingSetCurrent(1, 230, 0, 0);
    robot->WeldingSetVoltage(1, 24, 0, 1);

    robot->MoveJ(&startjointPos, &startdescPose, 13, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot->ArcWeldTraceControl(1, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
    robot->ARCStart(1, 0, 10000);
    robot->MoveL(&endjointPos, &enddescPose, 13, 0, 5, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot->ARCEnd(1, 0, 10000);

    robot->ArcWeldTraceControl(0, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
    return 0;
}

int SegmentWeld(FRRobot* robot)
{
    DescPose startdescPose = { -91.037, -505.079, 85.895, 164.953, -13.906, -131.826 };
    JointPos startjointPos = { -77.530, -123.147, -60.904, -90.546, 70.124, -33.053 };
     
    DescPose enddescPose = { 261.458, -460.453, 79.089, -171.505, 16.632, -136.465 };
    JointPos endjointPos = { -58.637, -126.187, -60.936, -77.769, 107.931, -10.122, };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };
    
    robot->SegmentWeldStart(&startdescPose, &enddescPose, &startjointPos, &endjointPos, 20, 20, 0, 0, 5000, 0, 0, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);

    return 0;
}

void Wiresearch(FRRobot* robot)
{
    int rtn0, rtn1, rtn2 = 0;
    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    DescPose descStart = { 203.061, 56.768, 62.719, -177.249, 1.456, -83.597 };
    JointPos jointStart = { -127.012, -112.931, -94.078, -62.014, 87.186, 91.326 };

    DescPose descEnd = { 122.471, 55.718, 62.209, -177.207, 1.375, -76.310 };
    JointPos jointEnd = { -119.728, -113.017, -94.027, -62.061, 87.199, 91.326 };

    robot->MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese );
    robot->MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);

    DescPose descREF0A = { 147.139, -21.436, 60.717, -179.633, -3.051, -83.170 };
    JointPos jointREF0A = { -121.731, -106.193, -102.561, -64.734, 89.972, 96.171 };

    DescPose descREF0B = { 139.247, 43.721, 65.361, -179.634, -3.043, -83.170 };
    JointPos jointREF0B = { -122.364, -113.991, -90.860, -68.630, 89.933, 95.540 };

    DescPose descREF1A = { 289.747, 77.395, 58.390, -179.074, -2.901, -89.790 };
    JointPos jointREF1A = { -135.719, -119.588, -83.454, -70.245, 88.921, 88.819 };

    DescPose descREF1B = { 259.310, 79.998, 64.774, -179.073, -2.900, -89.790 };
    JointPos jointREF1B = { -133.133, -119.029, -83.326, -70.976, 89.069, 91.401 };

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("REF0");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("REF1");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("RES0");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("RES1");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
    vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
    int offectFlag = 0;
    DescPose offectPos = {0, 0, 0, 0, 0, 0};
    rtn0 = robot->GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
    robot->PointsOffsetEnable(0, &offectPos);
    robot->MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot->MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, 0, &exaxisPos, 1, 0, &offdese);
    robot->PointsOffsetDisable();
}

void AxleSensorConfig(FRRobot* robot)
{
    robot->AxleSensorConfig(18, 0, 0, 1);
    int company = -1;
    int type = -1;
    robot->AxleSensorConfigGet(company, type);
    printf("company is %d, type is %d\n", company, type);

    robot->AxleSensorActivate(1);

    robot->Sleep(5000);

    while (true)
    {
        robot->AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
    }

}

 void TestWeave(FRRobot* robot)
{
    DescPose  desc_p1, desc_p2;

    JointPos j1 ;
    JointPos j2 ;

    desc_p1.tran.x = -299.979;
    desc_p1.tran.y = -399.974;
    desc_p1.tran.z = 74.979;
    desc_p1.rpy.rx = 0.009;
    desc_p1.rpy.ry = 0.001;
    desc_p1.rpy.rz = -41.530;

    j1.jPos[0] = 41.476;
    j1.jPos[1] = -77.300;
    j1.jPos[2] = 118.714;
    j1.jPos[3] = -131.405;
    j1.jPos[4] = -90.002;
    j1.jPos[5] = -51.993;

    desc_p2.tran.x = -49.985;
    desc_p2.tran.y = -399.956;
    desc_p2.tran.z = 74.978;
    desc_p2.rpy.rx = 0.009;
    desc_p2.rpy.ry = 0.005;
    desc_p2.rpy.rz = -41.530;

    j2.jPos[0] = 68.366;
    j2.jPos[1] = -89.562;
    j2.jPos[2] = 133.018;
    j2.jPos[3] = -133.446;
    j2.jPos[4] = -90.002;
    j2.jPos[5] = -25.105;

    ExaxisPos* epos = new ExaxisPos();
    DescPose* offset_pos = new DescPose();

    robot->WeaveSetPara(0, 4, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 60.000000);
    robot->MoveJ(&j1, &desc_p1, 13, 0, 100, 100, 100, epos, -1, 0, offset_pos);
    robot->WeaveStart(0);
    robot->MoveL(&j2, &desc_p2, 13, 0, 10, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
    robot->WeaveEnd(0);

    robot->WeaveSetPara(0, 0, 1.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 30.000000);
    robot->MoveJ(&j1, &desc_p1, 13, 0, 100, 100, 100, epos, -1, 0, offset_pos);
    robot->WeaveStart(0);
    robot->MoveL(&j2, &desc_p2, 13, 0, 10, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
    robot->WeaveEnd(0);
}

 int TestWelding(FRRobot* robot)
 {
     robot->WeldingSetCurrentRelation(55.34, 1788.34, 2.4, 9.0987, 0);
     robot->WeldingSetVoltageRelation(3.4, 34.56, 0.33, 7.98, 1);

     double a = 0, b = 0, c = 0, d = 0;
     int e = 0;

     robot->WeldingGetCurrentRelation(&a, &b, &c, &d, &e);
     printf("value is %f %f %f %f %d\n", a, b, c, d, e);
     robot->WeldingGetVoltageRelation(&a, &b, &c, &d, &e);
     printf("value is %f %f %f %f %d\n", a, b, c, d, e);

     robot->WeldingSetCurrent(0, 600, 0, 0);
     robot->WeldingSetVoltage(0, 20, 1, 0);
     return 0;
 }

 void TestTorue(FRRobot* robot)
 {
     robot->ProgramLoad("/fruser/test2.lua");
     robot->ProgramRun();
     int rtn = 0;
     while (true)
     {
         double temperature[6] = {};
         rtn = robot->GetJointDriverTemperature(temperature);
         double torque[6] = {};
         rtn = robot->GetJointDriverTorque(torque);
         printf("test torque is %f %f %f %f %f %f  temperature is %f %f %f %f %f %f\n", torque[0], torque[1], torque[2], torque[3], torque[4], torque[5], temperature[0], temperature[1], temperature[2], temperature[3], temperature[4], temperature[5]);
         robot->Sleep(100);
     }
 }

 void TestRealTimePeriod(FRRobot* robot)
 {
     robot->SetRobotRealtimeStateSamplePeriod(10);
     int getPeriod = 0;
     robot->GetRobotRealtimeStateSamplePeriod(getPeriod);
     cout << "period is " << getPeriod << endl;
     robot->Sleep(1000);
 }

 void TestweServoJ(FRRobot* robot)
 {
     JointPos j;
     memset(&j, 0, sizeof(JointPos));

     float vel = 100;
     float acc = 100;
     float cmdT = 0.008;
     float filterT = 0.0;
     float gain = 0.0;
     uint8_t flag = 1;
     int count = 1000;
     double dt = 0.01;

     ExaxisPos axis;
     axis.ePos[0] = 0;
     axis.ePos[1] = 0;
     axis.ePos[2] = 0; 
     axis.ePos[3] = 0;
     robot->ExtAxisMove(axis, 10);
     robot->ResetAllError();
     int ret = robot->GetActualJointPosDegree(flag, &j);
     if (ret == 0)
     {
         while (count)
         {
             int ret = robot->ServoJ(&j, &axis, acc, vel, cmdT, filterT, gain);
             j.jPos[0] += dt * 7;
             axis.ePos[0] += dt;
             axis.ePos[1] += dt*10;
             count -= 1;

             robot->Sleep(8);
         }
     }
     else
     {
         printf("GetActualJointPosDegree errcode:%d\n", ret);
     }
     return ;
 }

 void TestWeldTraceReply(FRRobot* robot)
 {
     JointPos mulitilineorigin1_joint;
     mulitilineorigin1_joint.jPos[0] = -24.090;
     mulitilineorigin1_joint.jPos[1] = -63.501;
     mulitilineorigin1_joint.jPos[2] = 84.288;
     mulitilineorigin1_joint.jPos[3] = -111.940;
     mulitilineorigin1_joint.jPos[4] = -93.426;
     mulitilineorigin1_joint.jPos[5] = 57.669;
     
     DescPose mulitilineorigin1_desc;
     mulitilineorigin1_desc.tran.x = -677.559;
     mulitilineorigin1_desc.tran.y = 190.951;
     mulitilineorigin1_desc.tran.z = -1.205;
     mulitilineorigin1_desc.rpy.rx = 1.144;
     mulitilineorigin1_desc.rpy.ry = -41.482;
     mulitilineorigin1_desc.rpy.rz = -82.577;

     DescTran mulitilineX1_desc;
     mulitilineX1_desc.x = -677.556;
     mulitilineX1_desc.y = 211.949;
     mulitilineX1_desc.z = -1.206;

     DescTran mulitilineZ1_desc;
     mulitilineZ1_desc.x = -677.564;
     mulitilineZ1_desc.y = 190.956;
     mulitilineZ1_desc.z = 19.817;

     JointPos mulitilinesafe_joint;
     mulitilinesafe_joint.jPos[0] = -25.734;
     mulitilinesafe_joint.jPos[1] = -63.778;
     mulitilinesafe_joint.jPos[2] = 81.502;
     mulitilinesafe_joint.jPos[3] = -108.975;
     mulitilinesafe_joint.jPos[4] = -93.392;
     mulitilinesafe_joint.jPos[5] = 56.021;

     DescPose mulitilinesafe_desc;
     mulitilinesafe_desc.tran.x = -677.561;
     mulitilinesafe_desc.tran.y = 211.950;
     mulitilinesafe_desc.tran.z = 19.812;
     mulitilinesafe_desc.rpy.rx = 1.144;
     mulitilinesafe_desc.rpy.ry = -41.482;
     mulitilinesafe_desc.rpy.rz = -82.577;

     JointPos mulitilineorigin2_joint;
     mulitilineorigin2_joint.jPos[0] = -29.743;
     mulitilineorigin2_joint.jPos[1] = -75.623;
     mulitilineorigin2_joint.jPos[2] = 101.241;
     mulitilineorigin2_joint.jPos[3] = -116.354;
     mulitilineorigin2_joint.jPos[4] = -94.928;
     mulitilineorigin2_joint.jPos[5] = 55.735;

     DescPose mulitilineorigin2_desc;
     mulitilineorigin2_desc.tran.x = -563.961;
     mulitilineorigin2_desc.tran.y = 215.359;
     mulitilineorigin2_desc.tran.z = -0.681;
     mulitilineorigin2_desc.rpy.rx = 2.845;
     mulitilineorigin2_desc.rpy.ry = -40.476;
     mulitilineorigin2_desc.rpy.rz = -87.443;
 
     DescTran mulitilineX2_desc;
     mulitilineX2_desc.x = -563.965;
     mulitilineX2_desc.y = 220.355;
     mulitilineX2_desc.z = -0.680;

     DescTran mulitilineZ2_desc;
     mulitilineZ2_desc.x = -563.968;
     mulitilineZ2_desc.y = 215.362;
     mulitilineZ2_desc.z = 4.331;

     ExaxisPos epos;
     epos.ePos[0] = 0;
     epos.ePos[1] = 0;
     epos.ePos[2] = 0;
     epos.ePos[3] = 0;
     DescPose offset;
     offset.tran.x = 0;
     offset.tran.y = 0;
     offset.tran.z = 0;
     offset.rpy.rx = 0;
     offset.rpy.ry = 0;
     offset.rpy.rz = 0;

     robot->Sleep(10);
     int error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ARCStart(1, 0, 3000);
     printf("ARCStart return:  %d\n", error);

     error = robot->WeaveStart(0);
     printf("WeaveStart return:  %d\n", error);

     error = robot->ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
     printf("ArcWeldTraceControl return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 1, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
     printf("ArcWeldTraceControl return:  %d\n", error);

     error = robot->WeaveEnd(0);
     printf("WeaveEnd return:  %d\n", error);

     error = robot->ARCEnd(1, 0, 10000);
     printf("ARCEnd return:  %d\n", error);


     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ARCStart(1, 0, 3000);
     printf("ARCStart return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->ArcWeldTraceReplayStart();
     printf("ArcWeldTraceReplayStart return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ArcWeldTraceReplayEnd();
     printf("ArcWeldTraceReplayEnd return:  %d\n", error);

     error = robot->ARCEnd(1, 0, 10000);
     printf("ARCEnd return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ARCStart(1, 0, 3000);
     printf("ARCStart return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->ArcWeldTraceReplayStart();
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ArcWeldTraceReplayEnd();
     printf("ArcWeldTraceReplayEnd return:  %d\n", error);

     error = robot->ARCEnd(1, 0, 3000);
     printf("ARCEnd return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);
 }

 void TestAngular(FRRobot* robot)
 {
     JointPos JP1(-68.030, -63.537, -105.223, -78.368, 72.828, 24.876);
     DescPose DP1(-60.984, -533.958, 279.089, -22.052, -4.777, 172.406);

     JointPos JP2(-80.916, -76.030, -108.901, -70.956, 99.026, -74.533);
     DescPose DP2(36.750, -488.721, 145.781, -37.539, -11.211, -96.491);

     JointPos JP3(-86.898, -95.200, -103.665, -70.570, 98.266, -93.321);
     DescPose DP3(-21.462, -509.234, 25.706, -41.780, -1.042, -83.611);

     JointPos JP4(-85.364, -102.697, -94.674, -70.557, 95.302, -93.116);
     DescPose DP4(-24.075, -580.525, 25.881, -44.818, -2.357, -82.259);

     JointPos JP5(-78.815, -94.279, -105.315, -65.348, 87.328, 3.220);
     DescPose DP5(-29.155, -580.477, 25.884, -44.795, -2.374, -172.261);

     JointPos JP6(-81.057, -94.494, -105.107, -65.241, 87.527, 0.987);
     DescPose DP6(-49.270, -580.460, 25.886, -44.796, -2.374, -172.263);

     JointPos JP7(-76.519, -101.428, -94.915, -76.521, 85.041, 95.758);
     DescPose DP7(-54.189, -580.362, 25.878, -44.779, -2.353, 97.740);

     JointPos JP8(-74.406, -90.991, -106.574, -75.480, 85.150, 97.875);
     DescPose DP8(-54.142, -503.358, 25.865, -44.780, -2.353, 97.740);

     ExaxisPos epos(0, 0, 0, 0);
     DescPose offset(0, 0, 0, 0, 0, 0);

     int tool = 7;
     int user = 0;
     double vel = 100.0;
     double acc = 100.0;
     double ovl = 50.0;
     int blend = -1;
     int offsetFlag = 0;

     int error = robot->MoveJ(&JP1, &DP1, tool, user, vel, acc, ovl, &epos, blend, offsetFlag, &offset);
     error = robot->MoveJ(&JP2, &DP2, tool, user, vel, acc, ovl, &epos, blend, offsetFlag, &offset);
     error = robot->MoveL(&JP3, &DP3, tool, user, vel, acc, ovl, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->SetOaccScale(100);
     error = robot->MoveL(&JP4, &DP4, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedStart(50);
     error = robot->MoveL(&JP5, &DP5, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedEnd();
     error = robot->MoveL(&JP6, &DP6, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedStart(50);
     error = robot->MoveL(&JP7, &DP7, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedEnd();
     error = robot->MoveL(&JP8, &DP8, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);

 }

 void TestAuxservo(FRRobot* robot)
 {
     robot->AuxServoSetParam(1, 1, 1, 1, 130172, 15.45);
     robot->AuxServoEnable(1, 0);
     robot->Sleep(1000);
     robot->AuxServoSetControlMode(1, 1);
     robot->Sleep(1000);
     robot->AuxServoEnable(1, 1);
     robot->Sleep(1000);
     robot->AuxServoHoming(1, 1, 10, 10, 100);
     robot->Sleep(4000);
     robot->AuxServoSetAcc(3000, 3000);
     robot->AuxServoSetEmergencyStopAcc(5000, 5000);
     robot->Sleep(1000);
     double emagacc = 0;
     double emagdec = 0;
     robot->AuxServoGetEmergencyStopAcc(emagacc, emagdec);
     printf("emergency acc is %f  dec is %f \n", emagacc ,emagdec);

     robot->AuxServoSetTargetSpeed(1, 500, 100);

     robot->ProgramLoad("/fruser/testPTP.lua");
     robot->ProgramRun();
     int i = 0;
     while (true)
     {
         i++;
         if (i > 400)
         {
             robot->ResetAllError();
             i = 0;

             robot->AuxServoSetTargetSpeed(1, 500, 100);
         }
         ROBOT_STATE_PKG pkg;
         robot->GetRobotRealTimeState(&pkg);
         printf("%d:%d  cur velocity is %f   cur 485 axis emergency state is %d   robot collision state is %d  robot emergency state is %d\n",
             pkg.robotTime.second,pkg.robotTime.millisecond,pkg.aux_state.servoVel, ((pkg.aux_state.servoState >> 7) & 0x01), pkg.collisionState, pkg.EmergencyStop);
         robot->Sleep(5);
         //ROBOT_STATE_PKG pkg;
         robot->GetRobotRealTimeState(&pkg);
         printf("cur velocity is %f   cur emergency state is %d \n", pkg.aux_state.servoVel, ((pkg.aux_state.servoState >> 7) & 0x01));
         robot->Sleep(20);
        robot->AuxServoSetAcc(5000, 5000);
        robot->AuxServoSetTargetPos(1, 1000, 500, 100);
        robot->Sleep(2000);
        robot->AuxServoSetTargetPos(1, 0, 500, 100);
        robot->Sleep(3000);
        robot->AuxServoSetAcc(500, 500);
        robot->AuxServoSetTargetPos(1, 1000, 500, 100);
        robot->Sleep(2000);
        robot->AuxServoSetTargetPos(1, 0, 500, 100);
        robot->Sleep(3000);
        robot->AuxServoSetTargetPos(1, 1000, 500, 10);
        robot->Sleep(5000);
        robot->AuxServoSetTargetPos(1, 0, 500, 10);
        robot->Sleep(5000);

        robot->AuxServoSetTargetSpeed(1, 500, 100);
        robot->Sleep(2000);
        robot->AuxServoSetTargetSpeed(1, 0, 100);
        robot->Sleep(2000);
        robot->AuxServoSetTargetSpeed(1, 500, 10);
        robot->Sleep(2000);
        robot->AuxServoSetTargetSpeed(1, 0, 10);
        robot->Sleep(2000);
     }
 }

 void TestAxleLuaGripper(FRRobot* robot)
 {
     robot->AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua"); 

     //Restart robot     

     ROBOT_STATE_PKG pkg;
     memset(&pkg, 0, sizeof(pkg));
     AxleComParam param(7, 8, 1, 0, 5, 3, 1);
     //AxleComParam param = new AxleComParam(8,7,2,1,6,4,2);
     robot->SetAxleCommunicationParam(param);

     AxleComParam getParam;
     robot->GetAxleCommunicationParam(&getParam);
     printf("GetAxleCommunicationParam param is %d %d %d %d %d %d %d\n", getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify, getParam.timeout, getParam.timeoutTimes, getParam.period);

     robot->SetAxleLuaEnable(1);
     int luaEnableStatus = 0;
     robot->GetAxleLuaEnableStatus(&luaEnableStatus);
     robot->SetAxleLuaEnableDeviceType(0, 1, 0);
     
     int forceEnable = 0;
     int gripperEnable = 0;
     int ioEnable = 0;
     robot->GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable);
     printf("GetAxleLuaEnableDeviceType param is %d %d %d\n", forceEnable, gripperEnable, ioEnable);

     //int func[16] = {0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0};
     int func[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
     robot->SetAxleLuaGripperFunc(1, func);
     int getFunc[16] = {0};
     robot->GetAxleLuaGripperFunc(1, getFunc);
     int getforceEnable[16] = {0};
     int getgripperEnable[16] = {0};
     int getioEnable[16] = {0};
     robot->GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable);
     printf("\ngetforceEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getforceEnable[i]);
     }
     printf("\ngetgripperEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getgripperEnable[i]);
     }
     printf("\ngetioEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getioEnable[i]);
     }
     printf("\n");
     robot->ActGripper(1, 0);
     robot->Sleep(2000);
     robot->ActGripper(1, 1);
     robot->Sleep(2000);
     //robot->MoveGripper(1, 90, 10, 100, 50000, 0);
     int pos = 0;
     while (true)
     {
         robot->Mode(0);
         robot->Sleep(1000);
         robot->Mode(1);
         robot->GetRobotRealTimeState(&pkg);
         printf("gripper pos is %u\n", pkg.gripper_position);
         robot->Sleep(100);
     }
     
 }

 void TestAxleLuaForceSensor(FRRobot* robot)
 {
     robot->AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua");

     //Restart robot  

     ROBOT_STATE_PKG pkg;
     memset(&pkg, 0, sizeof(pkg));
     AxleComParam param(7, 8, 1, 0, 5, 3, 1);
     robot->SetAxleCommunicationParam(param);

     AxleComParam getParam;
     robot->GetAxleCommunicationParam(&getParam);
     printf("GetAxleCommunicationParam param is %d %d %d %d %d %d %d\n", getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify, getParam.timeout, getParam.timeoutTimes, getParam.period);

     robot->SetAxleLuaEnable(1);
     int luaEnableStatus = 0;
     robot->GetAxleLuaEnableStatus(&luaEnableStatus);
     robot->SetAxleLuaEnableDeviceType(1, 0, 0);

     int forceEnable = 0;
     int gripperEnable = 0;
     int ioEnable = 0;
     robot->GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable);
     printf("GetAxleLuaEnableDeviceType param is %d %d %d\n", forceEnable, gripperEnable, ioEnable);

     
     int getforceEnable[16] = { 0 };
     int getgripperEnable[16] = { 0 };
     int getioEnable[16] = { 0 };
     robot->GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable);
     printf("\ngetforceEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getforceEnable[i]);
     }
     printf("\ngetgripperEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getgripperEnable[i]);
     }
     printf("\ngetioEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getioEnable[i]);
     }
     printf("\n");
     
     vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
     vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
     vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
     vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
     robot->EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100);

     robot->Sleep(10 * 1000);

     robot->EndForceDragControl(0, 0, 0, 0, M, B, K, F, 50, 100);
 }

 void TestEndLuaUpload(FRRobot* robot)
 {
     //robot->AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan_WeiHang_ERR1.lua");
     //robot->AxleLuaUpload("D://zUP/AXLE_LUA_End_JunDuo_Xinjingcheng.lua");
     robot->SetAxleLuaEnable(1);
     TestAxleLuaGripper(robot);
     while (true)
     {
         ROBOT_STATE_PKG pkg;
         robot->GetRobotRealTimeState(&pkg);
         printf("end lua err code is %d\n", pkg.endLuaErrCode);
         printf("gripper pos is %u\n", pkg.gripper_position);
         robot->Sleep(100);
     }
     
 }

 void TestTractorMove(FRRobot* robot)
 {
     robot->ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 101, 1);
     robot->ExtDevLoadUDPDriver();
     robot->ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
     robot->ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
     robot->SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0);

     robot->TractorEnable(false);
     robot->Sleep(2000);
     robot->TractorEnable(true);
     robot->Sleep(2000);
     robot->TractorHoming();
     robot->Sleep(2000);
     robot->TractorMoveL(100, 2);
     robot->Sleep(5000);
     robot->TractorStop();
     robot->TractorMoveL(-100, 20);
     robot->Sleep(5000);
     robot->TractorMoveC(300, 90, 20);
     robot->Sleep(10000);
     robot->TractorMoveC(300, -90, 20);
     robot->Sleep(1);
    

 }

 void TestUDPWireSearch(FRRobot* robot)
 {
     robot->ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
     robot->ExtDevLoadUDPDriver();

     robot->SetWireSearchExtDIONum(0, 0);

     int rtn0, rtn1, rtn2 = 0;
     ExaxisPos exaxisPos = { 0.0, 0.0, 0.0, 0.0 };
     DescPose offdese = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
     
     DescPose descStart = { -158.767, -510.596, 271.709, -179.427, -0.745, -137.349 };
     JointPos jointStart = { 61.667, -79.848, 108.639, -119.682, -89.700, -70.985 };
     
     DescPose descEnd = { 0.332, -516.427, 270.688, 178.165, 0.017, -119.989 };
     JointPos jointEnd = { 79.021, -81.839, 110.752, -118.298, -91.729, -70.981 };

     robot->MoveL(&jointStart, &descStart, 1, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&jointEnd, &descEnd, 1, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     
     DescPose descREF0A = { -66.106, -560.746, 270.381, 176.479, -0.126, -126.745 };
     JointPos jointREF0A = { 73.531, -75.588, 102.941, -116.250, -93.347, -69.689 };
     
     DescPose descREF0B = { -66.109, -528.440, 270.407, 176.479, -0.129, -126.744 };
     JointPos jointREF0B = { 72.534, -79.625, 108.046, -117.379, -93.366, -70.687 };
     
     DescPose descREF1A = { 72.975, -473.242, 270.399, 176.479, -0.129, -126.744 };
     JointPos jointREF1A = { 87.169, -86.509, 115.710, -117.341, -92.993, -56.034 };
     
     DescPose descREF1B = { 31.355, -473.238, 270.405, 176.480, -0.130, -126.745 };
     JointPos jointREF1B = { 82.117, -87.146, 116.470, -117.737, -93.145, -61.090 };

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF0A, &descREF0A, 1, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF0B, &descREF0B, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("REF0");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF1A, &descREF1A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF1B, &descREF1B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("REF1");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF0A, &descREF0A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF0B, &descREF0B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("RES0");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF1A, &descREF1A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF1B, &descREF1B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("RES1");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
     vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
     int offectFlag = 0;
     DescPose offectPos = { 0, 0, 0, 0, 0, 0 };
     rtn0 = robot->GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
     robot->PointsOffsetEnable(0, &offectPos);
     robot->MoveL(&jointStart, &descStart, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&jointEnd, &descEnd, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->PointsOffsetDisable();

 }

 void TestWeldModeChange(FRRobot* robot)
 {
     robot->ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
     robot->ExtDevLoadUDPDriver();

     robot->SetWeldMachineCtrlModeExtDoNum(17);
     robot->SetWeldMachineCtrlMode(0);

     robot->WeldingSetProcessParam(1, 200, 23, 1000, 100, 23, 100, 19, 1000);

     JointPos mulitilineorigin1_joint;
     mulitilineorigin1_joint.jPos[0] = -24.090;
     mulitilineorigin1_joint.jPos[1] = -63.501;
     mulitilineorigin1_joint.jPos[2] = 84.288;
     mulitilineorigin1_joint.jPos[3] = -111.940;
     mulitilineorigin1_joint.jPos[4] = -93.426;
     mulitilineorigin1_joint.jPos[5] = 57.669;

     DescPose mulitilineorigin1_desc;
     mulitilineorigin1_desc.tran.x = -677.559;
     mulitilineorigin1_desc.tran.y = 190.951;
     mulitilineorigin1_desc.tran.z = -1.205;
     mulitilineorigin1_desc.rpy.rx = 1.144;
     mulitilineorigin1_desc.rpy.ry = -41.482;
     mulitilineorigin1_desc.rpy.rz = -82.577;

     JointPos mulitilinesafe_joint;
     mulitilinesafe_joint.jPos[0] = -25.734;
     mulitilinesafe_joint.jPos[1] = -63.778;
     mulitilinesafe_joint.jPos[2] = 81.502;
     mulitilinesafe_joint.jPos[3] = -108.975;
     mulitilinesafe_joint.jPos[4] = -93.392;
     mulitilinesafe_joint.jPos[5] = 56.021;

     DescPose mulitilinesafe_desc;
     mulitilinesafe_desc.tran.x = -677.561;
     mulitilinesafe_desc.tran.y = 211.950;
     mulitilinesafe_desc.tran.z = 19.812;
     mulitilinesafe_desc.rpy.rx = 1.144;
     mulitilinesafe_desc.rpy.ry = -41.482;
     mulitilinesafe_desc.rpy.rz = -82.577;

     JointPos mulitilineorigin2_joint;
     mulitilineorigin2_joint.jPos[0] = -29.743;
     mulitilineorigin2_joint.jPos[1] = -75.623;
     mulitilineorigin2_joint.jPos[2] = 101.241;
     mulitilineorigin2_joint.jPos[3] = -116.354;
     mulitilineorigin2_joint.jPos[4] = -94.928;
     mulitilineorigin2_joint.jPos[5] = 55.735;

     DescPose mulitilineorigin2_desc;
     mulitilineorigin2_desc.tran.x = -563.961;
     mulitilineorigin2_desc.tran.y = 215.359;
     mulitilineorigin2_desc.tran.z = -0.681;
     mulitilineorigin2_desc.rpy.rx = 2.845;
     mulitilineorigin2_desc.rpy.ry = -40.476;
     mulitilineorigin2_desc.rpy.rz = -87.443;

     ExaxisPos epos;
     epos.ePos[0] = 0;
     epos.ePos[1] = 0;
     epos.ePos[2] = 0;
     epos.ePos[3] = 0;
     DescPose offset;
     offset.tran.x = 0;
     offset.tran.y = 0;
     offset.tran.z = 0;
     offset.rpy.rx = 0;
     offset.rpy.ry = 0;
     offset.rpy.rz = 0;

     robot->Sleep(10);
     int error = 0;

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ARCStart(1, 1, 3000);
     printf("ARCStart return:  %d\n", error);

     error = robot->WeaveStart(0);
     printf("WeaveStart return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 1, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->WeaveEnd(0);
     printf("WeaveEnd return:  %d\n", error);

     error = robot->ARCEnd(1, 1, 10000);
     printf("ARCEnd return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);
 }

 void TestWeldmechineMode(FRRobot* robot)
 {
     robot->ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
     robot->ExtDevLoadUDPDriver();

     robot->SetWeldMachineCtrlModeExtDoNum(17);
     for (int i = 0; i < 5; i++)
     {
         robot->SetWeldMachineCtrlMode(0);
         robot->Sleep(1000);
         robot->SetWeldMachineCtrlMode(1);
         robot->Sleep(1000);
     }

     robot->SetWeldMachineCtrlModeExtDoNum(18);
     for (int i = 0; i < 5; i++)
     {
         robot->SetWeldMachineCtrlMode(0);
         robot->Sleep(1000);
         robot->SetWeldMachineCtrlMode(1);
         robot->Sleep(1000);
     }

     robot->SetWeldMachineCtrlModeExtDoNum(19);
     for (int i = 0; i < 5; i++)
     {
         robot->SetWeldMachineCtrlMode(0);
         robot->Sleep(1000);
         robot->SetWeldMachineCtrlMode(1);
         robot->Sleep(1000);
     }

     
 }

 void TestUpgrade(FRRobot* robot)
 {
     robot->SoftwareUpgrade("D://test/software.tar.gz", false);
     while (true)
     {
         int curState = -1;
         robot->GetSoftwareUpgradeState(curState);
         printf("upgrade state is %d\n", curState);
         robot->Sleep(300);
     }
 }

 void TestSingularAvoidEArc(FRRobot* robot)
 {
     DescPose startdescPose(-57.170, -690.147, 370.969, 176.438, -8.320, 169.881);
     JointPos startjointPos(78.017, -62.036, 69.561, -94.199, -98.416, -1.360);

     DescPose middescPose(-71.044, -743.395, 375.996, -179.499, -5.398, 168.739);
     JointPos midjointPos(77.417, -55.000, 58.732, -94.360, -95.385, -1.376);

     DescPose enddescPose(-439.979, -512.743, 396.472, 178.112, 3.625, 146.576);
     JointPos endjointPos(40.243, -65.402, 70.802, -92.565, -87.055, -16.465);


     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     robot->SingularAvoidStart(2, 10, 5, 5);
     robot->MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
     robot->SingularAvoidEnd();
 }

 void TestSingularAvoidSArc(FRRobot* robot)
 {
     int rtn = 0;
     DescPose startdescPose(299.993, -168.982, 299.998, 179.999, -0.002, -166.415);
     JointPos startjointPos(-12.160, -71.236, -131.775, -66.992, 90.000, 64.255);

     DescPose middescPose(249.985, -140.988, 299.929, 179.996, -0.013, -166.417);
     JointPos midjointPos(-8.604, -60.474, -137.494, -72.046, 89.999, 67.813);

     DescPose enddescPose(-249.991, -168.980, 299.981, 179.999, 0.004, -107.386);
     JointPos endjointPos(-126.186, -63.401, -136.126, -70.477, 89.998, -108.800);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     rtn = robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     //rtn = robot->SingularAvoidStart(2, 30, 5, 5);
     rtn = robot->MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
     //rtn = robot->SingularAvoidEnd();
     printf("robot moving rtn is %d\n", rtn);
 }

 void TestSingularAvoidWArc(FRRobot* robot)
 {
     DescPose startdescPose(-352.575, -685.604, 479.380, -15.933, -54.906, 130.699);
     JointPos startjointPos(49.630, -56.597, 60.017, -57.989, 42.725, 146.834);

     DescPose middescPose(-437.302, -372.046, 366.764, -133.489, -62.309, -94.994);
     JointPos midjointPos(21.202, -72.442, 84.164, -51.660, -29.880, 146.823);

     DescPose enddescPose(-653.649, -235.926, 434.525, -176.386, -54.515, -66.734);
     JointPos endjointPos(5.070, -58.920, 55.287, -57.937, -41.207, 146.834);

     //DescPose descPose(-402.473, -185.876, 103.985, -175.367, 59.682, 94.221);
     //JointPos jointPos(-0.095, -50.828, 109.737, -150.708, -30.225, -0.623);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     //robot->SingularAvoidStart(2, 10, 5, 4);
     robot->MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
     //robot->MoveL(&jointPos, &descPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     //robot->SingularAvoidEnd();
 }

 void TestSingularAvoidSLin(FRRobot* robot)
 {
     DescPose startdescPose(300.002, -102.991, 299.994, 180.000, -0.001, -166.416);
     JointPos startjointPos(-0.189, -66.345, -134.615, -69.042, 90.000, 76.227);

     DescPose enddescPose(-300.000, -103.001, 299.994, 179.998, 0.003, -107.384);
     JointPos endjointPos(-142.292, -66.345, -134.615, -69.042, 89.997, -124.908);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     //robot->SingularAvoidStart(2, 30, 10, 3);
     robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     //robot->SingularAvoidEnd();
 }

 void TestSingularAvoidWLin(FRRobot* robot)
 {
     DescPose startdescPose(-352.574, -685.606, 479.415, -15.926, -54.905, 130.693);
     JointPos startjointPos(49.630, -56.597, 60.013, -57.990, 42.725, 146.834);

     DescPose enddescPose(-653.655, -235.943, 434.585, -176.403, -54.513, -66.719);
     JointPos endjointPos(5.072, -58.920, 55.280, -57.939, -41.207, 146.834);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     robot->SingularAvoidStart(2, 30, 10, 3);
     robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     robot->SingularAvoidEnd();
 }

 int UploadTrajectoryJ(FRRobot* robot)
 {
     robot->TrajectoryJDelete("testA.txt");
     robot->TrajectoryJUpLoad("D://zUP/testA.txt");

     int rtn = 0;
     char traj_file_name[30] = "/fruser/traj/testA.txt";
     rtn = robot->LoadTrajectoryJ(traj_file_name, 100, 1);
     printf("LoadTrajectoryJ %s, rtn is: %d\n", traj_file_name, rtn);

     DescPose traj_start_pose;
     memset(&traj_start_pose, 0, sizeof(DescPose));
     rtn = robot->GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
     printf("GetTrajectoryStartPose is: %d\n", rtn);
     printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);

     robot->SetSpeed(20);
     robot->MoveCart(&traj_start_pose, 1, 0, 100, 100, 100, -1, -1);

     robot->Sleep(5000);

     int traj_num = 0;
     rtn = robot->GetTrajectoryPointNum(&traj_num);
     printf("GetTrajectoryStartPose rtn is: %d, traj num is: %d\n", rtn, traj_num);

     rtn = robot->MoveTrajectoryJ();
     printf("MoveTrajectoryJ rtn is: %d\n", rtn);
 }

 int UploadTrajectoryB(FRRobot* robot)
 {
     robot->TrajectoryJDelete("testB.txt");
     robot->TrajectoryJUpLoad("D://zUP/testB.txt");

     int rtn = 0;
     char traj_file_name[30] = "/fruser/traj/testB.txt";
     rtn = robot->LoadTrajectoryJ(traj_file_name, 100, 1);
     printf("LoadTrajectoryJ %s, rtn is: %d\n", traj_file_name, rtn);

     DescPose traj_start_pose;
     memset(&traj_start_pose, 0, sizeof(DescPose));
     rtn = robot->GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
     printf("GetTrajectoryStartPose is: %d\n", rtn);
     printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);

     robot->SetSpeed(20);
     robot->MoveCart(&traj_start_pose, 1, 0, 100, 100, 100, -1, -1);

     robot->Sleep(5000);

     int traj_num = 0;
     rtn = robot->GetTrajectoryPointNum(&traj_num);
     printf("GetTrajectoryStartPose rtn is: %d, traj num is: %d\n", rtn, traj_num);

     rtn = robot->MoveTrajectoryJ();
     printf("MoveTrajectoryJ rtn is: %d\n", rtn);
 }

 int MoveRotGripper(FRRobot* robot, int pos, double rotPos)
 {
     robot->ResetAllError();
     robot->ActGripper(1, 1);
     robot->Sleep(1000);
     int rtn = robot->MoveGripper(1, pos, 50, 50, 5000, 1, 1, rotPos, 50, 100);
     printf("move gripper rtn is %d\n", rtn);
     uint16_t fault = 0;
     double rotNum = 0.0;
     int rotSpeed = 0;
     int rotTorque = 0;
     robot->GetGripperRotNum(&fault, &rotNum);
     robot->GetGripperRotSpeed (&fault, &rotSpeed);
     robot->GetGripperRotTorque(&fault, &rotTorque);
     printf("gripper rot num : %lf, gripper rotSpeed : %d, gripper rotTorque : %d\n", rotNum, rotSpeed, rotTorque);

     return 0;
 }

 int SetAO(FRRobot* robot, float value)
 {
     robot->SetAO(0, value, 0);
     robot->SetAO(1, value, 0);
     robot->SetToolAO(0, value, 0);
     while (true)
     {
         ROBOT_STATE_PKG pkg = {};
         robot->GetRobotRealTimeState(&pkg);
         if (fabs(pkg.cl_analog_output[0] - value) < 0.5)
         {
             break;
         }
         else
         {
             printf("cur AO value is %f\n", pkg.cl_analog_output[0]);
             robot->Sleep(1);
         }
     }
     printf("setAO Done  %f\n", value);
     return 0;
 }

 void TrajectoryJUpload(FRRobot* robot)
 {
     int rtn = -1;
     rtn = robot->TrajectoryJUpLoad("D://zUP/testA.txt");
     printf("Upload TrajectoryJ A %d\n", rtn);
     rtn = robot->TrajectoryJUpLoad("D://zUP/testB.txt");
     printf("Upload TrajectoryJ B %d\n", rtn);

     rtn = robot->TrajectoryJDelete("testA.txt");
     printf("Delete TrajectoryJ A %d\n", rtn);
     rtn = robot->TrajectoryJDelete("testB.txt");
     printf("Delete TrajectoryJ B %d\n", rtn);
 }

 void TrajectoryJDelete(FRRobot* robot)
 {
     int rtn = -1;
     rtn = robot->TrajectoryJDelete("testA.txt");
     printf("Delete TrajectoryJ A %d\n", rtn);
     rtn = robot->TrajectoryJDelete("testB.txt");
     printf("Delete TrajectoryJ B %d\n", rtn);
 }

 void FIRArc(FRRobot* robot, bool enable)
 {
     DescPose startdescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
     JointPos startjointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

     DescPose middescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
     JointPos midjointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

     DescPose enddescPose(-608.420, 610.692, 314.930, -176.438, -1.756, 117.333);
     JointPos endjointPos(-56.153, -46.964, 68.015, -113.200, -86.661, -83.479);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);
     
     if (enable)
     {
         robot->LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
         robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         robot->MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
         robot->LinArcFIRPlanningEnd();
     }
     else
     {
         robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         robot->MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
     }
 }

 void FIRLin(FRRobot* robot, bool enable)
 {
     DescPose startdescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
     JointPos startjointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

     DescPose enddescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
     JointPos endjointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     if (enable)
     {
         robot->LinArcFIRPlanningStart(5000, 5000, 5000, 5000);
         robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         robot->LinArcFIRPlanningEnd();
     }
     else
     {
         robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     }
 }

 void FIRLinL(FRRobot* robot, bool enable)
 {
     DescPose startdescPose(-194.554, -422.428, 373.689, -173.424, 13.610, -129.644);
     JointPos startjointPos(47.705, -92.449, 104.413, -88.071, -84.054, -92.590);

     DescPose enddescPose(-454.672, -221.547, 310.489, -179.596, 11.618, -166.234);
     JointPos endjointPos(11.111, -84.325, 106.005, -100.056, -90.131, -92.600);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);
     int rtn = 0;

     if (enable)
     {
         robot->LinArcFIRPlanningStart(5000, 5000, 5000, 5000);
         robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         robot->LinArcFIRPlanningEnd();
     }
     else
     {
         rtn = robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         printf("robot moveL rtn is %d\n", rtn);
         rtn = robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
         printf("robot moveL rtn is %d\n", rtn);
     }
 }

 void FIRPTP(FRRobot* robot, bool enable)
 {
     DescPose startdescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
     JointPos startjointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

     DescPose enddescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
     JointPos endjointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     if (enable)
     {
         robot->PtpFIRPlanningStart(1000);
         robot->MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         robot->MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         robot->PtpFIRPlanningEnd();
     }
     else
     {
         robot->MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         robot->MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     }
 }

 void TestReWeld(FRRobot* robot)
 {
     int rtn = -1;
     rtn = robot->WeldingSetCheckArcInterruptionParam(1, 200);
     printf("WeldingSetCheckArcInterruptionParam    %d\n", rtn);
     rtn = robot->WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
     printf("WeldingSetReWeldAfterBreakOffParam    %d\n", rtn);
     int enable = 0;
     double length = 0;
     double velocity = 0;
     int moveType = 0;
     int checkEnable = 0;
     int arcInterruptTimeLength = 0;
     rtn = robot->WeldingGetCheckArcInterruptionParam(&checkEnable, &arcInterruptTimeLength);
     printf("WeldingGetCheckArcInterruptionParam  checkEnable  %d   arcInterruptTimeLength  %d\n", checkEnable, arcInterruptTimeLength);
     rtn = robot->WeldingGetReWeldAfterBreakOffParam(&enable, &length, &velocity, &moveType);
     printf("WeldingGetReWeldAfterBreakOffParam  enable = %d, length = %lf, velocity = %lf, moveType = %d\n", enable, length, velocity, moveType);

     robot->ProgramLoad("/fruser/test.lua");
     robot->ProgramRun();

     robot->Sleep(5000);

     while (true)
     {
         ROBOT_STATE_PKG pkg = {};
         robot->GetRobotRealTimeState(&pkg);
         /*printf("welding breakoff state is     %d\n", pkg.weldingBreakOffState.breakOffState);
         if (pkg.weldingBreakOffState.breakOffState == 1)
         {
             printf("welding breakoff ! \n");
             robot->Sleep(2000);
             rtn = robot->WeldingStartReWeldAfterBreakOff();
             printf("WeldingStartReWeldAfterBreakOff    %d\n", rtn);
             break;
         }*/
         robot->Sleep(100);
     }
 }

 void TestTCP(FRRobot* robot)
 {
     DescPose p1Desc(-394.073, -276.405, 399.451, -133.692, 7.657, -139.047);
     JointPos p1Joint(15.234, -88.178, 96.583, -68.314, -52.303, -122.926);

     DescPose p2Desc( -187.141, -444.908, 432.425, 148.662, 15.483, -90.637);
     JointPos p2Joint(61.796, -91.959, 101.693, -102.417, -124.511, -122.767);

     DescPose p3Desc(-368.695, -485.023, 426.640, -162.588, 31.433, -97.036);
     JointPos p3Joint(43.896, -64.590, 60.087, -50.269, -94.663, -122.652);

     DescPose p4Desc(-291.069, -376.976, 467.560, -179.272, -2.326, -107.757);
     JointPos p4Joint(39.559, -94.731, 96.307, -93.141, -88.131, -122.673);

     DescPose p5Desc(-284.140, -488.041, 478.579, 179.785, -1.396, -98.030);
     JointPos p5Joint(49.283, -82.423, 81.993, -90.861, -89.427, -122.678);

     DescPose p6Desc(-296.307, -385.991, 484.492, -178.637, -0.057, -107.059);
     JointPos p6Joint(40.141, -92.742, 91.410, -87.978, -88.824, -122.808);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     JointPos posJ[6] = { p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint };
     DescPose coordRtn = {};
     int rtn = robot->ComputeToolCoordWithPoints(0, posJ, coordRtn);
     printf("ComputeToolCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     
     robot->MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetTcp4RefPoint(1);
     robot->MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetTcp4RefPoint(2);
     robot->MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetTcp4RefPoint(3);
     robot->MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetTcp4RefPoint(4);
     robot->ComputeTcp4(&coordRtn);
     printf("ComputeTcp4                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
     //robot->MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     //robot->MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);

     
 }

 void TestTCP6(FRRobot* robot)
 {
     DescPose p1Desc(-394.073, -276.405, 399.451, -133.692, 7.657, -139.047);
     JointPos p1Joint(15.234, -88.178, 96.583, -68.314, -52.303, -122.926);

     DescPose p2Desc(-187.141, -444.908, 432.425, 148.662, 15.483, -90.637);
     JointPos p2Joint(61.796, -91.959, 101.693, -102.417, -124.511, -122.767);

     DescPose p3Desc(-368.695, -485.023, 426.640, -162.588, 31.433, -97.036);
     JointPos p3Joint(43.896, -64.590, 60.087, -50.269, -94.663, -122.652);

     DescPose p4Desc(-291.069, -376.976, 467.560, -179.272, -2.326, -107.757);
     JointPos p4Joint(39.559, -94.731, 96.307, -93.141, -88.131, -122.673);

     DescPose p5Desc(-284.140, -488.041, 478.579, 179.785, -1.396, -98.030);
     JointPos p5Joint(49.283, -82.423, 81.993, -90.861, -89.427, -122.678);

     DescPose p6Desc(-296.307, -385.991, 484.492, -178.637, -0.057, -107.059);
     JointPos p6Joint(40.141, -92.742, 91.410, -87.978, -88.824, -122.808);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     JointPos posJ[6] = { p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint };
     DescPose coordRtn = {};
     int rtn = robot->ComputeToolCoordWithPoints(1, posJ, coordRtn);
     printf("ComputeToolCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);


     robot->MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetToolPoint(1);
     robot->MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetToolPoint(2);
     robot->MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetToolPoint(3);
     robot->MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetToolPoint(4);
     robot->MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetToolPoint(5);
     robot->MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetToolPoint(6);
     robot->ComputeTool(&coordRtn);
     printf("ComputeTool                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

 }

 void TestWObj(FRRobot* robot)
 {
     DescPose p1Desc(-275.046, -293.122, 28.747, 174.533, -1.301, -112.101);
     JointPos p1Joint(35.207, -95.350, 133.703, -132.403, -93.897, -122.768);

     DescPose p2Desc(-280.339, -396.053, 29.762, 174.621, -3.448, -102.901);
     JointPos p2Joint(44.304, -85.020, 123.889, -134.679, -92.658, -122.768);

     DescPose p3Desc(-270.597, -290.603, 83.034, 179.314, 0.808, -114.171);
     JointPos p3Joint(32.975, -99.175, 125.966, -116.484, -91.014, -122.857);

    

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     DescPose posTCP[3] = { p1Desc , p2Desc , p3Desc };
     DescPose coordRtn = {};
     int rtn = robot->ComputeWObjCoordWithPoints(1, posTCP, 0, coordRtn);
     printf("ComputeToolCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);


     robot->MoveJ(&p1Joint, &p1Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetWObjCoordPoint(1);
     robot->MoveJ(&p2Joint, &p2Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetWObjCoordPoint(2);
     robot->MoveJ(&p3Joint, &p3Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->SetWObjCoordPoint(3);
     robot->ComputeWObjCoord(1, 0, &coordRtn);
     printf("ComputeTool                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
     //robot->MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     //robot->MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);


 }


 void ExtAxisLaserTracking(FRRobot* robot)
 {
     DescPose p1Desc(381.070, -177.767, 227.851, 20.031, -2.455, -111.479);
     JointPos p1Joint(8.383, -44.801, -111.050, -97.707, 78.144, 27.709);

     DescPose p2Desc(381.077, -177.762, 217.865, 20.014, -0.131, -110.631);
     JointPos p2Joint(1.792, -44.574, -113.176, -93.687, 82.384, 21.154);

     DescPose p3Desc(381.070, -177.767, 227.851, 20.031, -2.455, -111.479);
     JointPos p3Joint(8.383, -44.801, -111.050, -97.707, 78.144, 27.709);

     ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
     DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

     ExaxisPos exaxisPosStart(0.0, 0.0, 0.0, 0.0);
     robot->MoveJ(&p1Joint, &p1Desc, 8, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->ExtAxisMove(exaxisPosStart, 50.0);
     robot->MoveL(&p2Joint, &p2Desc, 8, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->LaserSensorRecord(4, 1, 10, 2, 35, 0.1, 100);
     ExaxisPos exaxisPosTarget(0.000, 400.015, 0.000, 0.000);
     robot->ExtAxisMove(exaxisPosTarget, 10.0);
     robot->LaserSensorRecord(0, 1, 10, 2, 35, 0.1, 100);
     robot->MoveJ(&p3Joint, &p3Desc, 8, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->ExtAxisMove(exaxisPosStart, 50.0);
 }

 void TestLaser(FRRobot* robot)
 {
     int rtn = robot->LaserTrackingLaserOn(0);
     printf("LaserTrackingLaserOn    %d\n", rtn);
     rtn = robot->LaserTrackingLaserOff();
     printf("LaserTrackingLaserOff    %d\n", rtn);
     rtn = robot->LaserTrackingTrackOn(7);
     printf("LaserTrackingTrackOn    %d\n", rtn);
     rtn = robot->LaserTrackingTrackOff();
     printf("LaserTrackingTrackOff    %d\n", rtn);
     DescTran directionPoint = {};
     rtn = robot->LaserTrackingSearchStart(0, directionPoint, 100, 100, 10000, 7);
     printf("LaserTrackingSearchStart    %d\n", rtn);
     rtn = robot->LaserTrackingSearchStop();
     printf("LaserTrackingSearchStop    %d\n", rtn);

     while (true)
     {
         robot->LaserTrackingLaserOn(0);
         printf("LaserTrackingLaserOn    %d\n", rtn);
         robot->Sleep(1000);
         rtn = robot->LaserTrackingLaserOff();
         printf("LaserTrackingLaserOff    %d\n", rtn);
         robot->Sleep(1000);
     }
 }

 int AllInterface(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     robot.SetReConnectParam(true, 30000, 500);
     DescPose p1Desc(540.067, 51.456, 534.113, -179.888, -1.727, -152.934);
     JointPos p1Joint(171.785, -84.965, 76.066, -79.756, -91.088, 54.730);
     ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
     DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
     int tool = 0, user = 0, vel = 100, acc = 100, ovl = 100;
     DescPose offdese1(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);



     while (true)
     {
         robot.GetRobotRealTimeState(&pkg);
         printf("break state is %d   %d\n", pkg.weldingBreakOffState.breakOffState, pkg.weldingBreakOffState.weldArcState);
         robot.Sleep(100);
     }
     //robot.PointsOffsetEnable(0, &offdese1);
     ////robot.MoveJ(&p1Joint, &p1Desc, tool, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     //
     //
     //robot.MoveL(&p1Joint, &p1Desc, tool, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 2, 100);
     //robot.PointsOffsetDisable();

     DescPose middescPose(409.038, -137.064, 459.107, -173.659, 0.131, -179.723);
     JointPos midjointPos(145.172, -102.082, 104.372, -96.054, -95.109, 54.735);
     DescPose enddescPose(307.496, -315.912, 413.830, -177.741, -4.332, 153.361);
     JointPos endjointPos(118.104, -98.900, 106.821, -95.688, -94.344, 54.742);
     //robot.MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
     //robot.Circle(&midjointPos, &middescPose, tool, 0, 100, 100, &exaxisPos, &endjointPos, &enddescPose, tool, 0, 100, 100, &exaxisPos, 100, -1, &offdese);

     //robot.StartJOG(0, 1, 0, 20.0, 20.0, 30.0);   //单关节运动，StartJOG为非阻塞指令，运动状态下接收其他运动指令（包含StartJOG）会被丢弃
     //robot.Sleep(500);
     //robot.StopJOG(1);  //机器人单轴点动减速停止
     //robot.ImmStopJOG();  //机器人单轴点动立即停止

     //robot.MoveCart(&p1Desc, 0, 0, 100, 100, 100, -1, -1);

     //SpiralParam sp;
     //sp.circle_num = 5;
     //sp.circle_angle = 5.0;
     //sp.rad_init = 50.0;
     //sp.rad_add = 10.0;
     //sp.rotaxis_add = 10.0;
     //sp.rot_direction = 0;
     //robot.NewSpiral(&p1Joint, &p1Desc, tool, 0, 100, 100, &exaxisPos, 100, -1, &offdese, sp);

     //JointPos j;
     //memset(&j, 0, sizeof(JointPos));

     //float cmdT = 0.008;
     //float filterT = 0.0;
     //float gain = 0.0;
     //uint8_t flag = 1;
     //int count = 1000;
     //double dt = 0.01;
     //int ret = robot.GetActualJointPosDegree(flag, &j);
     //robot.ServoJ(&j, &exaxisPos, acc, vel, cmdT, filterT, gain);

     //float pos_gain[6] = { 0.0,0.0,1.0,0.0,0.0,0.0 };
     //int mode = 2;
     //filterT = 0.0;
     //gain = 0.0;
     //flag = 0;
     //robot.SetSpeed(20);
     //robot.ServoCart(mode, &p1Desc, pos_gain, acc, vel, cmdT, filterT, gain);

     //
     //robot.SplineStart();
     //robot.SplinePTP(&p1Joint, &p1Desc, tool, user, vel, acc, ovl);
     //robot.SplinePTP(&p1Joint, &p1Desc, tool, user, vel, acc, ovl);
     //robot.SplinePTP(&p1Joint, &p1Desc, tool, user, vel, acc, ovl);
     //robot.SplinePTP(&p1Joint, &p1Desc, tool, user, vel, acc, ovl);
     //robot.SplineEnd();

     //robot.NewSplineStart(1, 2000);
     //robot.NewSplinePoint(&p1Joint, &p1Desc, tool, user, vel, acc, ovl, -1, 0);
     //robot.NewSplineEnd();

     //robot.DragTeachSwitch(0);
     //robot.RobotEnable(1);
     //robot.Mode(0);;
     //robot.SetSpeed(10);
     //robot.SetLoadWeight(0, 1.3);
     //robot.SetLoadWeight(0, 0);
     //robot.SetLoadCoord(&p1Desc.tran);
     //robot.SetToolCoord(1, &p1Desc, 0, 0, 0, 0);
     //robot.SetToolList(1, &p1Desc, 0, 0, 0);
     //robot.SetExToolCoord(1, &p1Desc, &middescPose);
     //robot.SetExToolList(1, &p1Desc, &middescPose);
     //robot.SetWObjCoord(1, &p1Desc, 0);
     //robot.SetWObjList(1, &p1Desc, 0);
     //robot.SetRobotInstallPos(1);
     //robot.SetRobotInstallPos(0);
     //robot.SetRobotInstallAngle(90, 90);
     //robot.SetRobotInstallAngle(0, 0);
     //float lavev[6] = {1, 1, 1, 1, 1, 1};
     //robot.SetAnticollision(0, lavev, 0);
     //int margn[6] = {1, 1, 1, 1,1 ,1};
     //robot.SetCollisionStrategy(0, 1000, 10, 100, margn);
     //robot.ResetAllError();

     //robot.SetDO(100, 1, 0, 0);
     //robot.SetToolDO(100, 1, 0, 0);
     //robot.SetAO(100, 12.3, 0);
     //robot.SetToolAO(100, 31.2, 0);

     //robot.ProgramLoad("/fruser/test.lua");
     //robot.ProgramRun();
     //robot.ProgramPause();
     //robot.ProgramResume();
     //robot.ProgramStop();
     //robot.PointTableSwitch("pointtable1.db");
     //robot.ActGripper(2, 1);
     while (true)
     {
         robot.MoveL(&midjointPos, &middescPose, tool, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 100);
         robot.MoveGripper(2, 100, 20, 10, 10000, 0, 0, 0, 0, 0);
         //robot.ARCStart(0, 1, 10);
         robot.MoveL(&endjointPos, &enddescPose, tool, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 100);
         robot.MoveGripper(2, 0, 20, 10, 10000, 0, 0, 0, 0, 0);
         //robot.ARCEnd(0, 1, 10);
         robot.WeldingSetCurrent(0, 100, 0, 0);
         robot.WeldingSetVoltage(0, 19, 0, 0);
         //robot.WeaveStart(0);
         //robot.WeaveEnd(0);
         robot.WeldingStartReWeldAfterBreakOff();
         robot.WeldingAbortWeldAfterBreakOff();
     }







     robot.CloseRPC();
     return 0;
 }

 void TestWeaveChange(FRRobot* robot)
 {
     DescPose p1Desc(-72.912, -587.664, 31.849, 43.283, -6.731, 15.068);
     JointPos p1Joint(74.620, -80.903, 94.608, -109.882, -90.436, -13.432);

     DescPose p2Desc(-104.915, -483.712, -25.231, 42.228, -6.572, 18.433);
     JointPos p2Joint(66.431, -92.875, 116.362, -120.516, -88.627, -24.731);

     DescPose p3Desc(-240.651, -483.840, -7.161, 46.577, -5.286, 8.318);
     JointPos p3Joint(56.457, -84.796, 104.618, -114.497, -92.422, -25.430);

     ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
     DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
     robot->WeldingSetVoltage(1, 19, 0, 0);
     robot->WeldingSetCurrent(1, 190, 0, 0);
     robot->MoveJ(&p1Joint, &p1Desc, 1, 1, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->MoveL(&p2Joint, &p2Desc, 1, 1, 100, 100, 50, -1, &exaxisPos, 0, 0, &offdese);
     robot->ARCStart(1, 0, 10000);
     robot->ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
     robot->WeaveStart(0);
     robot->WeaveChangeStart(1, 0, 1, 1);
     robot->MoveL(&p3Joint, &p3Desc, 1, 1, 100, 100, 1, -1, &exaxisPos, 0, 0, &offdese);
     robot->WeaveChangeEnd();
     robot->WeaveEnd(0);
     robot->ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
     robot->ARCEnd(1, 0, 10000);
 }

 void TestArcWeldTraceChange(FRRobot* robot)
 {
     DescPose p1Desc(-72.912, -587.664, 31.849, 43.283, -6.731, 15.068);
     JointPos p1Joint(74.620, -80.903, 94.608, -109.882, -90.436, -13.432);

     DescPose p2Desc( -104.915, -483.712, -25.231, 42.228, -6.572, 18.433);
     JointPos p2Joint(66.431, -92.875, 116.362, -120.516, -88.627, -24.731);

     DescPose p3Desc(-240.651, -483.840, -7.161, 46.577, -5.286, 8.318);
     JointPos p3Joint(56.457, -84.796, 104.618, -114.497, -92.422, -25.430);

     ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
     DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
     robot->WeldingSetVoltage(1, 19, 0, 0);
     robot->WeldingSetCurrent(1, 190, 0, 0);
     robot->MoveJ(&p1Joint, &p1Desc, 1, 1, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->MoveL(&p2Joint, &p2Desc, 1, 1, 100, 100, 50, -1, &exaxisPos, 0, 0, &offdese);
     robot->ARCStart(1, 0, 10000);
     robot->ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, -5);
     robot->WeaveStart(0);
     robot->MoveL(&p3Joint, &p3Desc, 1, 1, 100, 100, 1, -1, &exaxisPos, 0, 0, &offdese);
     robot->WeaveEnd(0);
     robot->ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, -5);
     robot->ARCEnd(1, 0, 10000);
 }

 void TestTrajectoryLA(FRRobot* robot)
 {
     int rtn = 0;
     //rtn = robot->TrajectoryJUpLoad("D://zUP/A.txt");
     //cout << "TrajectoryJUpLoad A.txt rtn is " << rtn << endl;
     //rtn = robot->TrajectoryJUpLoad("D://zUP/B.txt");
     //cout << "TrajectoryJUpLoad B.txt rtn is " << rtn << endl;

     char nameA[30] = "/fruser/traj/A.txt";
     char nameB[30] = "/fruser/traj/B.txt";
     
     //rtn = robot->LoadTrajectoryLA(nameA, 2, 0.0, 0, 1.0, 100.0, 200.0, 1000.0);    //B样条
     //cout << "LoadTrajectoryLA rtn is " << rtn << endl;
     //robot->LoadTrajectoryLA(nameB, 0, 0, 0, 1, 100, 100, 1000);//直线连接
     robot->LoadTrajectoryLA(nameA, 1, 2, 0, 2, 100, 200, 1000);    //直线拟合
     DescPose startPos(0, 0, 0, 0, 0, 0);
     robot->GetTrajectoryStartPose(nameA, &startPos);
     //robot->GetTrajectoryStartPose(nameB, &startPos);
     robot->MoveCart(&startPos, 1, 0, 100, 100, 100, -1, -1);
     rtn = robot->MoveTrajectoryLA();
     cout << "MoveTrajectoryLA rtn is " << rtn << endl;
 }

 void CustomCollisionTest(FRRobot* robot)
 {
     DescPose p1Desc(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
     JointPos p1Joint(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

     DescPose p2Desc(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
     JointPos p2Joint(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

     ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
     DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
     robot->MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, 2, &exaxisPos, 0, 0, &offdese);

     robot->ResetAllError();
     int safety[6] = { 5,5,5,5,5,5 };
     robot->SetCollisionStrategy(3, 1000, 150, 250, safety);
     double jointDetectionThreshould[6] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
     double tcpDetectionThreshould[6] = { 60,60,60,60,60,60 };
     int rtn = robot->CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0);
     cout << "CustomCollisionDetectionStart rtn is " << rtn << endl;

     robot->MoveL(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     rtn = robot->CustomCollisionDetectionEnd();
     cout << "CustomCollisionDetectionEnd rtn is " << rtn << endl;
 }

 void TestAccSmoothJ(FRRobot* robot)
 {
     DescPose startdescPose(88.739, -527.617, 514.939, -179.039, 1.494, 70.209);
     JointPos startjointPos(88.927, -85.834, 80.289, -85.561, -91.388, 108.718);

     DescPose enddescPose(-433.125, -334.428, 497.139, -179.723, -0.745, 8.437);
     JointPos endjointPos(27.036, -83.909, 80.284, -85.579, -90.027, 108.604);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);
     int rtn = robot->AccSmoothStart(0);
     cout << "AccSmoothStart rtn is " << rtn << endl;
     robot->MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot->AccSmoothEnd(0);
     cout << "AccSmoothEnd rtn is " << rtn << endl;
 }

 void TestAccSmoothL(FRRobot* robot)
 {
     DescPose startdescPose(88.739, -527.617, 514.939, -179.039, 1.494, 70.209);
     JointPos startjointPos(88.927, -85.834, 80.289, -85.561, -91.388, 108.718);

     DescPose enddescPose(-433.125, -334.428, 497.139, -179.723, -0.745, 8.437);
     JointPos endjointPos(27.036, -83.909, 80.284, -85.579, -90.027, 108.604);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);
     int rtn = robot->AccSmoothStart(0);
     cout << "AccSmoothStart rtn is " << rtn << endl;

     robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);

     rtn = robot->AccSmoothEnd(0);
     cout << "AccSmoothEnd rtn is " << rtn << endl;
 }

 void TestAccSmoothC(FRRobot* robot)
 {
     DescPose startdescPose(88.739, -527.617, 514.939, -179.039, 1.494, 70.209);
     JointPos startjointPos(88.927, -85.834, 80.289, -85.561, -91.388, 108.718);

     DescPose enddescPose(-433.125, -334.428, 497.139, -179.723, -0.745, 8.437);
     JointPos endjointPos(27.036, -83.909, 80.284, -85.579, -90.027, 108.604);

     DescPose middescPose(-112.215, -409.323, 686.497, 176.217, 2.338, 41.625);
     JointPos midjointPos(60.219, -94.324, 62.906, -62.005, -87.159, 108.598);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);

     int rtn = robot->AccSmoothStart(0);
     cout << "AccSmoothStart rtn is " << rtn << endl;

     robot->MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
     robot->MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &startjointPos, &startdescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);

     rtn = robot->AccSmoothEnd(0);
     cout << "AccSmoothEnd rtn is " << rtn << endl;

 }

 void TestAccSmoothCirCle(FRRobot* robot)
 {
     DescPose startdescPose(88.739, -527.617, 514.939, -179.039, 1.494, 70.209);
     JointPos startjointPos(88.927, -85.834, 80.289, -85.561, -91.388, 108.718);

     DescPose enddescPose(-433.125, -334.428, 497.139, -179.723, -0.745, 8.437);
     JointPos endjointPos(27.036, -83.909, 80.284, -85.579, -90.027, 108.604);

     DescPose middescPose(-112.215, -409.323, 686.497, 176.217, 2.338, 41.625);
     JointPos midjointPos(60.219, -94.324, 62.906, -62.005, -87.159, 108.598);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);

     int rtn = robot->AccSmoothStart(0);
     cout << "AccSmoothStart rtn is " << rtn << endl;

     robot->Circle(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 100, -1, &offdese);
     robot->Circle(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 100, -1, &offdese);

     rtn = robot->AccSmoothEnd(0);
     cout << "AccSmoothEnd rtn is " << rtn << endl;
 }

 void TestInverseKen(FRRobot* robot)
 {
     DescPose dcs1(32.316, -232.029, 1063.415, 90.159, 18.376, 36.575);
     DescPose dcs2(105.25, -170.914, 1076.283, 87.032, 25.94, 54.644);
     DescPose dcs3(79.164, 81.645, 1045.609, 133.691, -73.265, 162.726);
     DescPose dcs4(298.779, -104.112, 298.242, 179.631, -0.628, -166.481);
     JointPos inverseRtn = {};

     robot->GetInverseKin(0, &dcs1, -1, &inverseRtn);
     printf("dcs1 getinverse rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
     robot->GetInverseKin(0, &dcs2, -1, &inverseRtn);
     printf("dcs2 getinverse rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
     
     robot->GetInverseKin(0, &dcs3, -1, &inverseRtn);
     printf("dcs3 getinverse rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
     robot->GetInverseKin(0, &dcs4, -1, &inverseRtn);
     printf("dcs4 getinverse rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
     
     JointPos jpos1(56.999, -59.002, 56.996, -96.552, 60.392, -90.005);
     DescPose forwordResult = {};
     robot->GetForwardKin(&jpos1, &forwordResult);
     printf("jpos1 forwordResult rtn is %f %f %f %f %f %f \n", forwordResult.tran.x, forwordResult.tran.y, forwordResult.tran.z, forwordResult.rpy.rx, forwordResult.rpy.ry, forwordResult.rpy.rz);


 }

 void Trigger(FRRobot* robot)
 {
     int i;

     cout << "please input a number to trigger:" << endl;

     std::cin >> i;

     int rtn = robot->ConveyorComDetectTrigger();
     printf("ConveyorComDetectTrigger rtn is: %d\n", rtn);
 }

 int ConveyorTest(FRRobot * robot)
 {

     int rtn = 0;

     rtn = 0;
     float param[6] = { 1,10000,200,0,0,20 };
     rtn = robot->ConveyorSetParam(param, 1, 0, 0);
     printf("ConveyorSetParam rtn is: %d\n", rtn);

     int index = 1;
     int max_time = 30000;
     uint8_t block = 0;
     rtn = 0;

     /* 下面是一个传送带抓取流程 */
     DescPose startdescPose(139.176, 4.717, 9.088, -179.999, -0.004, -179.990);
     JointPos startjointPos(-34.129, -88.062, 97.839, -99.780, -90.003, -34.140);
     
     DescPose homePose(139.177, 4.717, 69.084, -180.000, -0.004, -179.989);
     JointPos homejointPos(-34.129, -88.618, 84.039, -85.423, -90.003, -34.140);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     rtn = robot->MoveL(&homejointPos, &homePose, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     printf("MoveL to safety rtn is: %d\n", rtn);


     std::thread textT(Trigger, robot);
     textT.detach();

     rtn = robot->ConveyorComDetect(1000 * 10);
     printf("ConveyorComDetect rtn is: %d\n", rtn);

     rtn = robot->ConveyorGetTrackData(2);
     printf("ConveyorGetTrackData rtn is: %d\n", rtn);


     rtn = robot->ConveyorTrackStart(2);
     printf("ConveyorTrackStart rtn is: %d\n", rtn);

     robot->MoveL(&startjointPos, &startdescPose, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     robot->MoveL(&startjointPos, &startdescPose, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);

     rtn = robot->ConveyorTrackEnd();
     printf("ConveyorTrackEnd rtn is: %d\n", rtn);
     robot->MoveL(&homejointPos, &homePose, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     
     return 0;
 }

 void TestDownload()
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     robot.SetReConnectParam(true, 30000, 500);

     cout << "DataPackageDownload start" << endl;
     rtn = robot.DataPackageDownload("D://zDOWN/");
     cout << "DataPackageDownload rtn is " << rtn << endl;
     return ;

     robot.ShutDownRobotOS();

     for (int i = 0; i < 30; i++)
     {
         cout << "DataPackageDownload start" << endl;
         rtn = robot.DataPackageDownload("D://zDOWN/");
         cout << "DataPackageDownload rtn is " << rtn << "  times  " << i << endl;
     }

     for (int i = 0; i < 30; i++)
     {
         cout << "AllDataSourceDownload start" << endl;
         rtn = robot.AllDataSourceDownload("D://zDOWN/");
         cout << "AllDataSourceDownload rtn is " << rtn << "  times  " << i << endl;
     }
     for (int i = 0; i < 30; i++)
     {
         cout << "RbLogDownload start" << endl;
         rtn = robot.RbLogDownload("D://zDOWN/");
         cout << "RbLogDownload rtn is " << rtn << "  times  " << i << endl;
     }
     for (int i = 0; i < 30; i++)
     {
         string SN = "";
         robot.GetRobotSN(SN);
         cout << "robot SN is " << SN << "  times  " << i << endl;
     }
 }

 int WeaveAngle(FRRobot* robot)
 {
     DescPose startdescPose = { 146.273, -208.110, 270.102, 177.523, -3.782, -158.101 };
     JointPos startjointPos = { 98.551, -128.309, 127.341, -87.490, -94.249, -13.208 };
     DescPose enddescPose = {146.272, -476.204, 270.102, 177.523, -3.781, -158.101};
     JointPos endjointPos = { 93.931, -89.722, 102.216, -101.300, -94.359, -17.840 };

     ExaxisPos exaxisPos = { 0, 0, 0, 0 };
     DescPose offdese = { 0, 0, 0, 0, 0, 0 };
     robot->WeaveSetPara(0, 3, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0, 0);
     robot->MoveL(&startjointPos, &startdescPose, 2, 0, 100, 100, 100, -1, 0,&exaxisPos, 0, 0, &offdese);
     robot->WeaveStart(0);
     robot->MoveL(&endjointPos, &enddescPose, 2, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     robot->WeaveEnd(0);

     robot->WeaveSetPara(0, 3, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0, 30);
     robot->MoveL(&startjointPos, &startdescPose, 2, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     robot->WeaveStart(0);
     robot->MoveL(&endjointPos, &enddescPose, 2, 0, 100, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     robot->WeaveEnd(0);
     return 0;
 }

 int WeldTraceControlWithCtrlBoxAI(FRRobot* robot)
 {
     DescPose startdescPose = { -473.86, 257.879, -20.849, -37.317, -42.021, 2.543 };
     JointPos startjointPos = { -43.487, -76.526, 95.568, -104.445, -89.356, 3.72 };

     DescPose enddescPose = { -499.844, 141.225, 7.72, -34.856, -40.17, 13.13 };
     JointPos endjointPos = { -31.305, -82.998, 99.401, -104.426, -89.35, 3.696 };

     DescPose safedescPose = { -504.043, 275.181, 40.908, -28.002, -42.025, -14.044 };
     JointPos safejointPos = { -39.078, -76.732, 87.227, -99.47, -94.301, 18.714 };

     ExaxisPos exaxisPos = { 0, 0, 0, 0 };
     DescPose offdese = { 0, 0, 0, 0, 0, 0 };

     robot->WeldingSetCurrentRelation(0, 495, 1, 10, 0);
     robot->WeldingSetVoltageRelation(10, 45, 1, 10, 1);

     robot->WeldingSetVoltage(0, 25, 1, 0);// ----设置电压
     robot->WeldingSetCurrent(0, 260, 0, 0);// ----设置电流

     int rtn = robot->ArcWeldTraceAIChannelCurrent(4);
     cout << "ArcWeldTraceAIChannelCurrent rtn is " << rtn << endl;
     rtn = robot->ArcWeldTraceAIChannelVoltage(5);
     cout << "ArcWeldTraceAIChannelVoltage rtn is " << rtn << endl;
     rtn = robot->ArcWeldTraceCurrentPara(0, 5, 0, 500);
     cout << "ArcWeldTraceCurrentPara rtn is " << rtn << endl;
     rtn = robot->ArcWeldTraceVoltagePara(1.018, 10, 0, 50);
     cout << "ArcWeldTraceVoltagePara rtn is " << rtn << endl;
     robot->MoveJ(&safejointPos, &safedescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot->ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
     cout << "ArcWeldTraceControl rtn is " << rtn << endl;
     robot->ARCStart(0, 0, 10000);
     robot->WeaveStart(0);
     robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 2, -1, &exaxisPos, 0, 0, &offdese);
     robot->ARCEnd(0, 0, 10000);
     robot->WeaveEnd(0);
     robot->ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
     return 0;
 }
 int WeldparamChange(FRRobot* robot)
 {
     DescPose startdescPose = { -484.707, 276.996, -14.013, -37.657, -40.508, -1.548 };
     JointPos startjointPos = { -45.421, -75.673, 93.627, -104.302, -87.938, 6.005 };
     
     DescPose enddescPose = { -508.767, 137.109, -13.966, -37.639, -40.508, -1.559 };
     JointPos endjointPos = { -32.768, -80.947, 100.254, -106.201, -87.201, 18.648 };

     DescPose safedescPose = { -484.709, 294.436, 13.621, -37.660, -40.508, -1.545 };
     JointPos safejointPos = { -46.604, -75.410, 89.109, -100.003, -88.012, 4.823 };

     ExaxisPos exaxisPos = { 0, 0, 0, 0 };
     DescPose offdese = { 0, 0, 0, 0, 0, 0 };

     robot->WeldingSetCurrentRelation(0, 495, 1, 10, 0);
     robot->WeldingSetVoltageRelation(10, 45, 1, 10, 1);

     //robot->WeldingSetVoltage(0, 25, 1, 0);// ----设置电压
     //robot->WeldingSetCurrent(0, 260, 0, 0);// ----设置电流

     //return 0;

     robot->MoveJ(&safejointPos, &safedescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
     int rtn = robot->WeldingSetCurrentGradualChangeStart(0, 260, 220, 0, 0);
     cout << "WeldingSetCurrentGradualChangeStart rtn is " << rtn << endl;
     rtn = robot->WeldingSetVoltageGradualChangeStart(0, 25, 22, 1, 0);
     cout << "WeldingSetVoltageGradualChangeStart rtn is " << rtn << endl;
     rtn = robot->ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
     cout << "ArcWeldTraceControl rtn is " << rtn << endl;
     robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
     
     robot->ARCStart(0, 0, 10000);
     robot->WeaveStart(0);
     robot->WeaveChangeStart(2, 1, 24, 36);
     robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 2, -1, &exaxisPos, 0, 0, &offdese);
     robot->ARCEnd(0, 0, 10000);
     robot->WeaveChangeEnd();
     robot->WeaveEnd(0);
     robot->ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
     robot->WeldingSetCurrentGradualChangeEnd();
     robot->WeldingSetVoltageGradualChangeEnd();
     return 0;
 }


 int TestBlend(FRRobot* robot)
 {
     ExaxisPos exaxisPos = { 0, 0, 0, 0 };
     DescPose offdese = { 0, 0, 0, 0, 0, 0 };

     JointPos JP1 = {55.203, -69.138, 75.617, -103.969, -83.549, -0.001};
     DescPose DP1 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP2 = {57.646, -61.846, 59.286, -69.645, -99.735, 3.824};
     DescPose DP2 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP3 = {57.304, -61.380, 58.260, -67.641, -97.447, 2.685};
     DescPose DP3 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP4 = {57.297, -61.373, 58.250, -67.637, -97.448, 2.677};
     DescPose DP4 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP5 = {23.845, -108.202, 111.300, -80.971, -106.753, -30.246};
     DescPose DP5 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP6 = {23.845, -108.202, 111.300, -80.971, -106.753, -30.246};
     DescPose DP6 = { 0, 0, 0, 0, 0, 0 };
     robot->GetForwardKin(&JP1, &DP1);
     robot->GetForwardKin(&JP2, &DP2);
     robot->GetForwardKin(&JP3, &DP3);
     robot->GetForwardKin(&JP4, &DP4);
     robot->GetForwardKin(&JP5, &DP5);
     robot->GetForwardKin(&JP6, &DP6);
     robot->MoveJ(&JP1, &DP1, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->MoveJ(&JP2, &DP2, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveJ(&JP3, &DP3, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveJ(&JP4, &DP4, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveJ(&JP5, &DP5, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveJ(&JP6, &DP6, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);


     JointPos JP7 = { -10.503, -93.654, 111.333, -84.702, -103.479, -30.179 };
     DescPose DP7 = { 0, 0, 0, 0, 0, 0 };

     JointPos JP8 = { -10.503, -93.654, 111.333, -84.702, -103.479, -30.179 };
     DescPose DP8 = { 0, 0, 0, 0, 0, 0 };
     
     JointPos JP9 = {-10.503, -93.654, 111.333, -84.702, -103.479, -30.179};
     DescPose DP9 = { 0, 0, 0, 0, 0, 0 };
     
     JointPos JP10 = { -30.623, -74.158, 89.844, -91.942, -97.060, -30.180 };
     DescPose DP10 = { 0, 0, 0, 0, 0, 0 };
     
     JointPos JP11 = { -34.797, -72.641, 93.917, -104.961, -84.449, -30.287 };
     DescPose DP11 = { 0, 0, 0, 0, 0, 0 };

     JointPos JP12 = { -17.454, -58.309, 82.054, -111.034, -109.900, -30.241 };
     DescPose DP12 = { 0, 0, 0, 0, 0, 0 };

     JointPos JP13 = { -4.930, -72.469, 100.631, -109.906, -76.760, -10.947 };
     DescPose DP13 = { 0, 0, 0, 0, 0, 0 };
     robot->GetForwardKin(&JP7, &DP7);
     robot->GetForwardKin(&JP8, &DP8);
     robot->GetForwardKin(&JP9, &DP9);
     robot->GetForwardKin(&JP10, &DP10);
     robot->GetForwardKin(&JP11, &DP11);
     robot->GetForwardKin(&JP12, &DP12);
     robot->GetForwardKin(&JP13, &DP13);
     robot->MoveJ(&JP7, &DP7, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveL(&JP8, &DP8, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveJ(&JP9, &DP9, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveL(&JP10, &DP10, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveJ(&JP11, &DP11, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveC(&JP12, &DP12, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &JP13, &DP13, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, 20);

     JointPos JP14 = { 9.586, -66.925, 85.589, -99.109, -103.403, -30.280 };
     DescPose DP14 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP15 = { 23.056, -59.187, 76.487, -102.155, -77.560, -30.250 };
     DescPose DP15 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP16 = { 28.028, -71.754, 91.463, -102.182, -102.361, -30.253 };
     DescPose DP16 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP17 = { 38.974, -62.622, 79.068, -102.543, -101.630, -30.253 };
     DescPose DP17 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP18 = { -34.797, -72.641, 93.917, -104.961, -84.449, -30.287 };
     DescPose DP18 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP19 = { -17.454, -58.309, 82.054, -111.034, -109.900, -30.241 };
     DescPose DP19 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP20 = { -4.930, -72.469, 100.631, -109.906, -76.760, -10.947 };
     DescPose DP20 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP21 = { 3.021, -76.365, 81.332, -98.130, -68.530, -30.284 };
     DescPose DP21 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP22 = { 12.532, -94.241, 106.254, -87.131, -102.719, -30.227 };
     DescPose DP22 = { 0, 0, 0, 0, 0, 0 };

     robot->GetForwardKin(&JP14, &DP14);
     robot->GetForwardKin(&JP15, &DP15);
     robot->GetForwardKin(&JP16, &DP16);
     robot->GetForwardKin(&JP17, &DP17);
     robot->GetForwardKin(&JP18, &DP18);
     robot->GetForwardKin(&JP19, &DP19);
     robot->GetForwardKin(&JP20, &DP20);
     robot->GetForwardKin(&JP21, &DP21);
     robot->GetForwardKin(&JP22, &DP22);

     robot->MoveJ(&JP14, &DP14, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->Circle(&JP15, &DP15, 0, 0, 100, 100, &exaxisPos, &JP16, &DP16, 0, 0, 100, 100, &exaxisPos, 100, 0, &offdese, 100, 20);
     robot->MoveJ(&JP17, &DP17, 0, 0, 100, 100, 100, &exaxisPos, 200, 0, &offdese);
     robot->MoveL(&JP18, &DP18, 0, 0, 100, 100, 100, 100, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveC(&JP19, &DP19, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &JP20, &DP20, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, 20);
     robot->MoveC(&JP21, &DP21, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &JP22, &DP22, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, 20);

     JointPos JP23 = { 9.586, -66.925, 85.589, -99.109, -103.403, -30.280 };
     DescPose DP23 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP24 = { 23.056, -59.187, 76.487, -102.155, -77.560, -30.250 };
     DescPose DP24 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP25 = { 28.028, -71.754, 91.463, -102.182, -102.361, -30.253 };
     DescPose DP25 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP26 = { -11.207, -81.555, 110.050, -108.983, -74.292, -30.249 };
     DescPose DP26 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP27 = { 18.930, -70.987, 100.659, -115.974, -115.465, -30.231 };
     DescPose DP27 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP28 = { 32.493, -65.561, 86.053, -109.669, -103.427, -30.267 };
     DescPose DP28 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP29 = { 21.954, -87.113, 123.299, -109.730, -72.157, -9.013 };
     DescPose DP29 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP30= { 19.084, -69.127, 104.304, -109.629, -106.997, -9.011 };
     DescPose DP30= { 0, 0, 0, 0, 0, 0 };
     JointPos JP31 = { 38.654, -60.146, 93.485, -109.637, -87.023, -8.989 };
     DescPose DP31 = { 0, 0, 0, 0, 0, 0 };

     robot->GetForwardKin(&JP23, &DP23);
     robot->GetForwardKin(&JP24, &DP24);
     robot->GetForwardKin(&JP25, &DP25);
     robot->GetForwardKin(&JP26, &DP26);
     robot->GetForwardKin(&JP27, &DP27);
     robot->GetForwardKin(&JP28, &DP28);
     robot->GetForwardKin(&JP29, &DP29);
     robot->GetForwardKin(&JP30, &DP30);
     robot->GetForwardKin(&JP31, &DP31);


     robot->MoveL(&JP23, &DP23, 0, 0, 100, 100, 100, 20, 1, &exaxisPos, 0, 0, &offdese);
     robot->Circle(&JP24, &DP24, 0, 0, 100, 100, &exaxisPos, &JP25, &DP25, 0, 0, 100, 100, &exaxisPos, 100, 0, &offdese, 100, 20);
     robot->Circle(&JP26, &DP26, 0, 0, 100, 100, &exaxisPos, &JP27, &DP27, 0, 0, 100, 100, &exaxisPos, 100, 0, &offdese, 100, 20);
     robot->MoveC(&JP28, &DP28, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &JP29, &DP29, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, 20);
     robot->Circle(&JP30, &DP30, 0, 0, 100, 100, &exaxisPos, &JP31, &DP31, 0, 0, 100, 100, &exaxisPos, 100, 0, &offdese, 100, 20);

     JointPos JP32 = { 38.654, -60.146, 93.485, -109.637, -87.023, -8.989 };
     DescPose DP32 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP33 = { 55.203, -69.138, 75.617, -103.969, -83.549, -0.001 };
     DescPose DP33 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP34 = { 57.646, -61.846, 59.286, -69.645, -99.735, 3.824 };
     DescPose DP34 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP35 = { 57.304, -61.380, 58.260, -67.641, -97.447, 2.685 };
     DescPose DP35 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP36 = { 57.297, -61.373, 58.250, -67.637, -97.448, 2.677 };
     DescPose DP36 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP37 = { 23.845, -108.202, 111.300, -80.971, -106.753, -30.246 };
     DescPose DP37 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP38 = { 23.845, -108.202, 111.300, -80.971, -106.753, -30.246 };
     DescPose DP38 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP39 = { -10.503, -93.654, 111.333, -84.702, -103.479, -30.179 };
     DescPose DP39 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP40 = { -30.623, -74.158, 89.844, -91.942, -97.060, -30.180 };
     DescPose DP40 = { 0, 0, 0, 0, 0, 0 };

     robot->GetForwardKin(&JP32, &DP32);
     robot->GetForwardKin(&JP33, &DP33);
     robot->GetForwardKin(&JP34, &DP34);
     robot->GetForwardKin(&JP35, &DP35);
     robot->GetForwardKin(&JP36, &DP36);
     robot->GetForwardKin(&JP37, &DP37);
     robot->GetForwardKin(&JP38, &DP38);
     robot->GetForwardKin(&JP39, &DP39);
     robot->GetForwardKin(&JP40, &DP40);

     robot->MoveL(&JP32, &DP32, 0, 0, 100, 100, 100, 20, 1, &exaxisPos, 0, 0, &offdese);
     robot->MoveJ(&JP33, &DP33, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot->MoveL(&JP34, &DP34, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&JP35, &DP35, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&JP36, &DP36, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&JP37, &DP37, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&JP38, &DP38, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&JP39, &DP39, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveJ(&JP40, &DP40, 0, 0, 100, 100, 100, &exaxisPos, 20, 0, &offdese);

     JointPos JP50 = { -34.797, -72.641, 93.917, -104.961, -84.449, -30.287 };
     DescPose DP50 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP41 = { -17.454, -58.309, 82.054, -111.034, -109.900, -30.241 };
     DescPose DP41 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP42 = { -4.930, -72.469, 100.631, -109.906, -76.760, -10.947 };
     DescPose DP42 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP43 = { 9.586, -66.925, 85.589, -99.109, -103.403, -30.280 };
     DescPose DP43 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP44 = { 23.056, -59.187, 76.487, -102.155, -77.560, -30.250 };
     DescPose DP44 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP45 = { 28.028, -71.754, 91.463, -102.182, -102.361, -30.253 };
     DescPose DP45 = { 0, 0, 0, 0, 0, 0 };
     JointPos JP46 = { 38.974, -62.622, 79.068, -102.543, -101.630, -30.253 };
     DescPose DP46 = { 0, 0, 0, 0, 0, 0 };

     robot->GetForwardKin(&JP50, &DP50);
     robot->GetForwardKin(&JP41, &DP41);
     robot->GetForwardKin(&JP42, &DP42);
     robot->GetForwardKin(&JP43, &DP43);
     robot->GetForwardKin(&JP44, &DP44);
     robot->GetForwardKin(&JP45, &DP45);
     robot->GetForwardKin(&JP46, &DP46);

     robot->MoveL(&JP50, &DP50, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->MoveC(&JP41, &DP41, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &JP42, &DP42, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, 20);
     robot->MoveL(&JP43, &DP43, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     robot->Circle(&JP44, &DP44, 0, 0, 100, 100, &exaxisPos, &JP45, &DP45, 0, 0, 100, 100, &exaxisPos, 100, 0, &offdese, 100, 20);
     robot->MoveL(&JP46, &DP46, 0, 0, 100, 100, 100, 20, 0, &exaxisPos, 0, 0, &offdese);
     return 0;
 }
 ///////////////////////////////////////////////////////////////////////////////////////////////////
 int TestRobotCtrl(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     robot.SetReConnectParam(true, 30000, 500);

     char ip[64] = "";
     char version[64] = "";
     uint8_t state;

     robot.GetSDKVersion(version);
     printf("SDK version:%s\n", version);
     robot.GetControllerIP(ip);
     printf("controller ip:%s\n", ip);

     robot.Mode(1);
     robot.Sleep(1000);
     robot.DragTeachSwitch(1);
     robot.Sleep(1000);
     robot.IsInDragTeach(&state);
     printf("drag state :%u\n", state);
     robot.Sleep(3000);
     robot.DragTeachSwitch(0);
     robot.Sleep(1000);
     robot.IsInDragTeach(&state);
     printf("drag state :%u\n", state);
     robot.Sleep(3000);

     robot.RobotEnable(0);
     robot.Sleep(3000);
     robot.RobotEnable(1);

     robot.Mode(0);
     robot.Sleep(1000);
     robot.Mode(1);

     robot.Sleep(3000);
     robot.ShutDownRobotOS();

     robot.CloseRPC();
     return 0;
 }

 int TestGetVersions(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     char robotModel[64] = { 0 };
     char webversion[64] = { 0 };
     char controllerVersion[64] = { 0 };

     char ctrlBoxBoardversion[128] = { 0 };
     char driver1version[128] = { 0 };
     char driver2version[128] = { 0 };
     char driver3version[128] = { 0 };
     char driver4version[128] = { 0 };
     char driver5version[128] = { 0 };
     char driver6version[128] = { 0 };
     char endBoardversion[128] = { 0 };

     rtn = robot.GetSoftwareVersion(robotModel, webversion, controllerVersion);
     printf("Getsoftwareversion rtn is: %d\n", rtn);
     printf("robotmodel is: %s, webversion is: %s, controllerVersion is: %s \n\n", robotModel, webversion, controllerVersion);

     rtn = robot.GetHardwareVersion(ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
     printf("GetHardwareversion rtn is: %d\n", rtn);
     printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);

     rtn = robot.GetFirmwareVersion(ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
     printf("GetFirmwareversion rtn is: %d\n", rtn);
     printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);

     robot.CloseRPC();
     return 0;
 }

 int TestJOG(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     for (int i = 0; i < 6; i++)
     {
         robot.StartJOG(0, i + 1, 0, 20.0, 20.0, 30.0);
         robot.Sleep(1000);
         robot.ImmStopJOG();
         robot.Sleep(1000);
     }

     for (int i = 0; i < 6; i++)
     {
         robot.StartJOG(2, i + 1, 0, 20.0, 20.0, 30.0);
         robot.Sleep(1000);
         robot.ImmStopJOG();
         robot.Sleep(1000);
     }

     for (int i = 0; i < 6; i++)
     {
         robot.StartJOG(4, i + 1, 0, 20.0, 20.0, 30.0);
         robot.Sleep(1000);
         robot.StopJOG(5);
         robot.Sleep(1000);
     }

     for (int i = 0; i < 6; i++)
     {
         robot.StartJOG(8, i + 1, 0, 20.0, 20.0, 30.0);
         robot.Sleep(1000);
         robot.StopJOG(9);
         robot.Sleep(1000);
     }

     robot.CloseRPC();
     return 0;
 }

 int TestMove(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
     JointPos j3(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);
     JointPos j4(-31.154, -95.317, 94.276, -88.079, -89.740, 74.256);
     DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
     DescPose desc_pos3(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);
     DescPose desc_pos4(-443.165, 147.881, 480.951, 179.511, -0.775, -15.409);
     DescPose offset_pos(0, 0, 0, 0, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);

     int tool = 0;
     int user = 0;
     float vel = 100.0;
     float acc = 100.0;
     float ovl = 100.0;
     float blendT = 0.0;
     float blendR = 0.0;
     uint8_t flag = 0;
     uint8_t search = 0;

     robot.SetSpeed(20);

     rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     printf("movej errcode:%d\n", rtn);

     rtn = robot.MoveL(&j2, &desc_pos2, tool, user, vel, acc, ovl, blendR, &epos, search, flag, &offset_pos);
     printf("movel errcode:%d\n", rtn);

     rtn = robot.MoveC(&j3, &desc_pos3, tool, user, vel, acc, &epos, flag, &offset_pos, &j4, &desc_pos4, tool, user, vel, acc, &epos, flag, &offset_pos, ovl, blendR);
     printf("movec errcode:%d\n", rtn);

     rtn = robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     printf("movej errcode:%d\n", rtn);

     rtn = robot.Circle(&j3, &desc_pos3, tool, user, vel, acc, &epos, &j1, &desc_pos1, tool, user, vel, acc, &epos, ovl, flag, &offset_pos, 100, -1);
     printf("circle errcode:%d\n", rtn);

     rtn = robot.MoveCart(&desc_pos4, tool, user, vel, acc, ovl, blendT, -1);
     printf("MoveCart errcode:%d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestSpiral(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);
     JointPos j(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     DescPose desc_pos(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose offset_pos1(50, 0, 0, -30, 0, 0);
     DescPose offset_pos2(50, 0, 0, -5, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);
     SpiralParam sp;
     sp.circle_num = 5;
     sp.circle_angle = 5.0;
     sp.rad_init = 50.0;
     sp.rad_add = 10.0;
     sp.rotaxis_add = 10.0;
     sp.rot_direction = 0;

     int tool = 0;
     int user = 0;
     float vel = 100.0;
     float acc = 100.0;
     float ovl = 100.0;
     float blendT = 0.0;
     uint8_t flag = 2;

     robot.SetSpeed(20);

     rtn = robot.MoveJ(&j, &desc_pos, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos1);
     printf("movej errcode:%d\n", rtn);

     rtn = robot.NewSpiral(&j, &desc_pos, tool, user, vel, acc, &epos, ovl, flag, &offset_pos2, sp);
     printf("newspiral errcode:%d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

int TestServoJ(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    JointPos j(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);

    float vel = 0.0;
    float acc = 0.0;
    float cmdT = 0.008;
    float filterT = 0.0;
    float gain = 0.0;
    uint8_t flag = 0;
    int count = 500;
    float dt = 0.1;
    int cmdID = 0;
    int ret = robot.GetActualJointPosDegree(flag, &j);
    if (ret == 0)
    {
        cmdID += 1;
        robot.ServoMoveStart();
        while (count)
        {
            robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain, cmdID);
            j.jPos[0] += dt;
            count -= 1;
            robot.WaitMs(cmdT * 1000);
        }
        robot.ServoMoveEnd();
    }
    else
    {
        printf("GetActualJointPosDegree errcode:%d\n", ret);
    }

    robot.CloseRPC();
    return 0;
}

 int TestServoCart(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     DescPose desc_pos_dt;
     memset(&desc_pos_dt, 0, sizeof(DescPose));

     desc_pos_dt.tran.z = -0.5;
     float pos_gain[6] = { 0.0,0.0,1.0,0.0,0.0,0.0 };
     int mode = 2;
     float vel = 0.0;
     float acc = 0.0;
     float cmdT = 0.008;
     float filterT = 0.0;
     float gain = 0.0;
     uint8_t flag = 0;
     int count = 100;

     robot.SetSpeed(20);

     while (count)
     {
         robot.ServoCart(mode, &desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
         count -= 1;
         robot.WaitMs(cmdT * 1000);
     }

     robot.CloseRPC();
     return 0;
 }

 int TestSpline(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
     JointPos j3(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
     JointPos j4(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
     DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
     DescPose desc_pos3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
     DescPose desc_pos4(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
     DescPose offset_pos(0, 0, 0, 0, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);


     int tool = 0;
     int user = 0;
     float vel = 100.0;
     float acc = 100.0;
     float ovl = 100.0;
     float blendT = -1.0;
     uint8_t flag = 0;

     robot.SetSpeed(20);

     int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     printf("movej errcode:%d\n", err1);
     robot.SplineStart();
     robot.SplinePTP(&j1, &desc_pos1, tool, user, vel, acc, ovl);
     robot.SplinePTP(&j2, &desc_pos2, tool, user, vel, acc, ovl);
     robot.SplinePTP(&j3, &desc_pos3, tool, user, vel, acc, ovl);
     robot.SplinePTP(&j4, &desc_pos4, tool, user, vel, acc, ovl);
     robot.SplineEnd();

     robot.CloseRPC();
     return 0;
 }

 int TestNewSpline(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
     JointPos j3(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
     JointPos j4(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
     JointPos j5(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
     DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
     DescPose desc_pos3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
     DescPose desc_pos4(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
     DescPose desc_pos5(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
     DescPose offset_pos(0, 0, 0, 0, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);


     int tool = 0;
     int user = 0;
     float vel = 100.0;
     float acc = 100.0;
     float ovl = 100.0;
     float blendT = -1.0;
     uint8_t flag = 0;

     robot.SetSpeed(20);

     int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     printf("movej errcode:%d\n", err1);
     robot.NewSplineStart(1, 2000);
     robot.NewSplinePoint(&j1, &desc_pos1, tool, user, vel, acc, ovl, -1, 0);
     robot.NewSplinePoint(&j2, &desc_pos2, tool, user, vel, acc, ovl, -1, 0);
     robot.NewSplinePoint(&j3, &desc_pos3, tool, user, vel, acc, ovl, -1, 0);
     robot.NewSplinePoint(&j4, &desc_pos4, tool, user, vel, acc, ovl, -1, 0);
     robot.NewSplinePoint(&j5, &desc_pos5, tool, user, vel, acc, ovl, -1, 0);
     robot.NewSplineEnd();

     robot.CloseRPC();
     return 0;
 }

 int TestPause(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j5(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
     DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_pos5(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
     DescPose offset_pos(0, 0, 0, 0, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);

     int tool = 0;
     int user = 0;
     float vel = 100.0;
     float acc = 100.0;
     float ovl = 100.0;
     float blendT = -1.0;
     uint8_t flag = 0;

     robot.SetSpeed(20);

     rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     rtn = robot.MoveJ(&j5, &desc_pos5, tool, user, vel, acc, ovl, &epos, 1, flag, &offset_pos);
     robot.Sleep(1000);
     robot.PauseMotion();

     robot.Sleep(1000);
     robot.ResumeMotion();

     robot.Sleep(1000);
     robot.StopMotion();

     robot.Sleep(1000);

     robot.CloseRPC();
     return 0;
 }

 int TestOffset(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

     DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     DescPose offset_pos(0, 0, 0, 0, 0, 0);
     DescPose offset_pos1(0, 0, 50, 0, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);

     int tool = 0;
     int user = 0;
     float vel = 100.0;
     float acc = 100.0;
     float ovl = 100.0;
     float blendT = -1.0;
     uint8_t flag = 0;

     robot.SetSpeed(20);

     robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.Sleep(1000);
     robot.PointsOffsetEnable(0, &offset_pos1);
     robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.PointsOffsetDisable();

     robot.CloseRPC();
     return 0;
 }

 int TestMoveAO(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

     DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     DescPose offset_pos(0, 0, 0, 0, 0, 0);
     DescPose offset_pos1(0, 0, 50, 0, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);

     int tool = 0;
     int user = 0;
     float vel = 20.0;
     float acc = 20.0;
     float ovl = 100.0;
     float blendT = -1.0;
     uint8_t flag = 0;

     robot.SetSpeed(20);

     robot.MoveAOStart(0, 100, 100, 20);
     robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.MoveAOStop();

     robot.Sleep(1000);

     robot.MoveToolAOStart(0, 100, 100, 20);
     robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
     robot.MoveToolAOStop();

     robot.CloseRPC();
     return 0;
 }

 int TestAODO(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     uint8_t status = 1;
     uint8_t smooth = 0;
     uint8_t block = 0;

     for (int i = 0; i < 16; i++)
     {
         robot.SetDO(i, status, smooth, block);
         robot.Sleep(300);
     }

     status = 0;

     for (int i = 0; i < 16; i++)
     {
         robot.SetDO(i, status, smooth, block);
         robot.Sleep(300);
     }

     status = 1;

     for (int i = 0; i < 2; i++)
     {
         robot.SetToolDO(i, status, smooth, block);
         robot.Sleep(1000);
     }

     status = 0;

     for (int i = 0; i < 2; i++)
     {
         robot.SetToolDO(i, status, smooth, block);
         robot.Sleep(1000);
     }

     for (int i = 0; i < 100; i++)
     {
         robot.SetAO(0, i * 40.96, block);
         robot.Sleep(30);
     }

     for (int i = 0; i < 100; i++)
     {
         robot.SetToolAO(0, i * 40.96, block);
         robot.Sleep(30);
     }

     robot.CloseRPC();
     return 0;
 }

 int TestGetDIAI(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     uint8_t status = 1;
     uint8_t smooth = 0;
     uint8_t block = 0;
     uint8_t di = 0, tool_di = 0;
     float ai = 0.0, tool_ai = 0.0;
     float value = 0.0;

     robot.GetDI(0, block, &di);
     printf("di0:%u\n", di);

     tool_di = robot.GetToolDI(1, block, &tool_di);
     printf("tool_di1:%u\n", tool_di);

     robot.GetAI(0, block, &ai);
     printf("ai0:%f\n", ai);

     tool_ai = robot.GetToolAI(0, block, &tool_ai);
     printf("tool_ai0:%f\n", tool_ai);

     uint8_t _button_state = 0;
     robot.GetAxlePointRecordBtnState(&_button_state);
     printf("_button_state is: %u\n", _button_state);

     uint8_t tool_do_state = 0;
     robot.GetToolDO(&tool_do_state);
     printf("tool DO state is: %u\n", tool_do_state);

     uint8_t do_state_h = 0;
     uint8_t do_state_l = 0;
     robot.GetDO(&do_state_h, &do_state_l);
     printf("DO state high is: %u \n DO state low is: %u\n", do_state_h, do_state_l);

     robot.CloseRPC();
     return 0;
 }

 int TestWaitDIAI(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     uint8_t status = 1;
     uint8_t smooth = 0;
     uint8_t block = 0;
     uint8_t di = 0, tool_di = 0;
     float ai = 0.0, tool_ai = 0.0;
     float value = 0.0;

     rtn = robot.WaitDI(0, 1, 1000, 1);
     cout << "WaitDI over; rtn is: " << rtn << endl;

     robot.WaitMultiDI(1, 3, 3, 1000, 1);
     cout << "WaitDI over; rtn is: " << rtn << endl;

     robot.WaitToolDI(1, 1, 1000, 1);
     cout << "WaitDI over; rtn is: " << rtn << endl;

     robot.WaitAI(0, 0, 50, 1000, 1);
     cout << "WaitDI over; rtn is: " << rtn << endl;

     robot.WaitToolAI(0, 0, 50, 1000, 1);
     cout << "WaitDI over; rtn is: " << rtn << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestDOReset(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     for (int i = 0; i < 16; i++)
     {
         robot.SetDO(i, 1, 0, 0);
         robot.Sleep(300);
     }

     int resetFlag = 1;
     rtn = robot.SetOutputResetCtlBoxDO(resetFlag);
     robot.SetOutputResetCtlBoxAO(resetFlag);
     robot.SetOutputResetAxleDO(resetFlag);
     robot.SetOutputResetAxleAO(resetFlag);
     robot.SetOutputResetExtDO(resetFlag);
     robot.SetOutputResetExtAO(resetFlag);
     robot.SetOutputResetSmartToolDO(resetFlag);

     robot.ProgramLoad("/fruser/test.lua");
     robot.ProgramRun();

     robot.CloseRPC();
     return 0;
 }

 int TestTCPCompute(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     DescPose p1Desc(186.331, 487.913, 209.850, 149.030, 0.688, -114.347);
     JointPos p1Joint(-127.876, -75.341, 115.417, -122.741, -59.820, 74.300);

     DescPose p2Desc(69.721, 535.073, 202.882, -144.406, -14.775, -89.012);
     JointPos p2Joint(-101.780, -69.828, 110.917, -125.740, -127.841, 74.300);

     DescPose p3Desc(146.861, 578.426, 205.598, 175.997, -36.178, -93.437);
     JointPos p3Joint(-112.851, -60.191, 86.566, -80.676, -97.463, 74.300);

     DescPose p4Desc(136.284, 509.876, 225.613, 178.987, 1.372, -100.696);
     JointPos p4Joint(-116.397, -76.281, 113.845, -128.611, -88.654, 74.299);

     DescPose p5Desc(138.395, 505.972, 298.016, 179.134, 2.147, -101.110);
     JointPos p5Joint(-116.814, -82.333, 109.162, -118.662, -88.585, 74.302);

     DescPose p6Desc(105.553, 454.325, 232.017, -179.426, 0.444, -99.952);
     JointPos p6Joint(-115.649, -84.367, 122.447, -128.663, -90.432, 74.303);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     JointPos posJ[6] = { p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint };
     DescPose coordRtn = {};
     rtn = robot.ComputeToolCoordWithPoints(1, posJ, coordRtn);
     printf("ComputeToolCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetToolPoint(1);
     robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetToolPoint(2);
     robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetToolPoint(3);
     robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetToolPoint(4);
     robot.MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetToolPoint(5);
     robot.MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetToolPoint(6);
     rtn = robot.ComputeTool(&coordRtn);
     printf("6 Point ComputeTool        %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
     robot.SetToolList(1, &coordRtn, 0, 0, 0);

     robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetTcp4RefPoint(1);
     robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetTcp4RefPoint(2);
     robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetTcp4RefPoint(3);
     robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetTcp4RefPoint(4);
     rtn = robot.ComputeTcp4(&coordRtn);
     printf("4 Point ComputeTool        %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     robot.SetToolCoord(2, &coordRtn, 0, 0, 1, 0);

     DescPose getCoord = {};
     rtn = robot.GetTCPOffset(0, &getCoord);
     printf("GetTCPOffset    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     robot.CloseRPC();
     return 0;
 }

 int TestWobjCoord(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     DescPose p1Desc(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
     JointPos p1Joint(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);

     DescPose p2Desc(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
     JointPos p2Joint(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);

     DescPose p3Desc(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
     JointPos p3Joint(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     DescPose posTCP[3] = { p1Desc , p2Desc , p3Desc };
     DescPose coordRtn = {};
     rtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0, coordRtn);
     printf("ComputeWObjCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     robot.MoveJ(&p1Joint, &p1Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetWObjCoordPoint(1);
     robot.MoveJ(&p2Joint, &p2Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetWObjCoordPoint(2);
     robot.MoveJ(&p3Joint, &p3Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetWObjCoordPoint(3);
     rtn = robot.ComputeWObjCoord(1, 0, &coordRtn);
     printf("ComputeWObjCoord                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     robot.SetWObjCoord(1, &coordRtn, 0);
     robot.SetWObjList(1, &coordRtn, 0);

     DescPose getWobjDesc = {};
     rtn = robot.GetWObjOffset(0, &getWobjDesc);
     printf("GetWObjOffset                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     robot.CloseRPC();
     return 0;
 }

 int TestExtCoord(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     DescPose p1Desc(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
     JointPos p1Joint(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);

     DescPose p2Desc(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
     JointPos p2Joint(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);

     DescPose p3Desc(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
     JointPos p3Joint(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     DescPose posTCP[3] = { p1Desc , p2Desc , p3Desc };
     DescPose coordRtn = {};

     robot.MoveJ(&p1Joint, &p1Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetExTCPPoint(1);
     robot.MoveJ(&p2Joint, &p2Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetExTCPPoint(2);
     robot.MoveJ(&p3Joint, &p3Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.SetExTCPPoint(3);
     rtn = robot.ComputeExTCF(&coordRtn);
     printf("ComputeExTCF                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);

     robot.SetExToolCoord(1, &coordRtn, &offdese);
     robot.SetExToolList(1, &coordRtn, &offdese);

     robot.CloseRPC();
     return 0;
 }

 int TestLoadInstall(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     for (int i = 1; i < 100; i++)
     {
         robot.SetSpeed(i);
         robot.SetOaccScale(i);
         robot.Sleep(30);
     }

     float defaultVel = 0.0;
     robot.GetDefaultTransVel(&defaultVel);
     printf("GetDefaultTransVel is %f\n", defaultVel);

     for (int i = 1; i < 21; i++)
     {
         robot.SetSysVarValue(i, i + 0.5);
         robot.Sleep(100);
     }

     for (int i = 1; i < 21; i++)
     {
         float value = 0;
         robot.GetSysVarValue(i, &value);
         printf("sys value  %d is :%f\n", i, value);
         robot.Sleep(100);
     }

     robot.SetLoadWeight(0, 2.5);

     DescTran loadCoord = {};
     loadCoord.x = 3.0;
     loadCoord.y = 4.0;
     loadCoord.z = 5.0;
     robot.SetLoadCoord(&loadCoord);

     robot.Sleep(1000);

     float getLoad = 0.0;
     robot.GetTargetPayload(0, &getLoad);

     DescTran getLoadTran = {};
     robot.GetTargetPayloadCog(0, &getLoadTran);
     printf("get load is %f; get load cog is %f %f %f\n", getLoad, getLoadTran.x, getLoadTran.y, getLoadTran.z);

     robot.SetRobotInstallPos(0);
     robot.SetRobotInstallAngle(15.0, 25.0);

     float anglex = 0.0;
     float angley = 0.0;
     robot.GetRobotInstallAngle(&anglex, &angley);
     printf("GetRobotInstallAngle x:  %f;  y:  %f\n", anglex, angley);

     robot.CloseRPC();
     return 0;
 }

 int TestFriction(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     float lcoeff[6] = { 0.9,0.9,0.9,0.9,0.9,0.9 };
     float wcoeff[6] = { 0.4,0.4,0.4,0.4,0.4,0.4 };
     float ccoeff[6] = { 0.6,0.6,0.6,0.6,0.6,0.6 };
     float fcoeff[6] = { 0.5,0.5,0.5,0.5,0.5,0.5 };

     rtn = robot.FrictionCompensationOnOff(1);
     printf("FrictionCompensationOnOff rtn is %d\n", rtn);

     rtn = robot.SetFrictionValue_level(lcoeff);
     printf("SetFrictionValue_level rtn is %d\n", rtn);

     rtn = robot.SetFrictionValue_wall(wcoeff);
     printf("SetFrictionValue_wall rtn is %d\n", rtn);

     rtn = robot.SetFrictionValue_ceiling(ccoeff);
     printf("SetFrictionValue_ceiling rtn is %d\n", rtn);

     rtn = robot.SetFrictionValue_freedom(fcoeff);
     printf("SetFrictionValue_freedom rtn is %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestGetError(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int maincode, subcode;
     robot.GetRobotErrorCode(&maincode, &subcode);
     printf("robot maincode is %d;  subcode is %d\n", maincode, subcode);

     robot.ResetAllError();

     robot.Sleep(1000);

     robot.GetRobotErrorCode(&maincode, &subcode);
     printf("robot maincode is %d;  subcode is %d\n", maincode, subcode);

     robot.CloseRPC();
     return 0;
 }

 int TestCollision(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int mode = 0;
     int config = 1;
     float level1[6] = { 1.0,2.0,3.0,4.0,5.0,6.0 };
     float level2[6] = { 50.0,20.0,30.0,40.0,50.0,60.0 };

     rtn = robot.SetAnticollision(mode, level1, config);
     printf("SetAnticollision mode 0 rtn is %d\n", rtn);
     mode = 1;
     rtn = robot.SetAnticollision(mode, level2, config);
     printf("SetAnticollision mode 1 rtn is %d\n", rtn);

     JointPos p1Joint(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos p2Joint(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

     DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
     DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
     robot.MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, 2, &exaxisPos, 0, 0, &offdese);
     robot.ResetAllError();
     int safety[6] = { 5,5,5,5,5,5 };
     rtn = robot.SetCollisionStrategy(3, 1000, 150, 250, safety);
     printf("SetCollisionStrategy rtn is %d\n", rtn);

     double jointDetectionThreshould[6] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
     double tcpDetectionThreshould[6] = { 60,60,60,60,60,60 };
     rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0);
     cout << "CustomCollisionDetectionStart rtn is " << rtn << endl;

     robot.MoveL(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot.MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     rtn = robot.CustomCollisionDetectionEnd();
     cout << "CustomCollisionDetectionEnd rtn is " << rtn << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestLimit(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     float plimit[6] = { 170.0,80.0,150.0,80.0,170.0,160.0 };
     robot.SetLimitPositive(plimit);
     float nlimit[6] = { -170.0,-260.0,-150.0,-260.0,-170.0,-160.0 };
     robot.SetLimitNegative(nlimit);

     float neg_deg[6] = { 0.0 }, pos_deg[6] = { 0.0 };
     robot.GetJointSoftLimitDeg(0, neg_deg, pos_deg);
     printf("neg limit deg:%f,%f,%f,%f,%f,%f\n", neg_deg[0], neg_deg[1], neg_deg[2], neg_deg[3], neg_deg[4], neg_deg[5]);
     printf("pos limit deg:%f,%f,%f,%f,%f,%f\n", pos_deg[0], pos_deg[1], pos_deg[2], pos_deg[3], pos_deg[4], pos_deg[5]);

     robot.CloseRPC();
     return 0;
 }

 int TestCollisionMethod(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     rtn = robot.SetCollisionDetectionMethod(0, 0);
     printf("SetCollisionDetectionMethod rtn is %d\n", rtn);

     rtn = robot.SetStaticCollisionOnOff(1);
     printf("SetStaticCollisionOnOff On rtn is %d\n", rtn);
     rtn = robot.Sleep(5000);
     rtn = robot.SetStaticCollisionOnOff(0);
     printf("SetStaticCollisionOnOff Off rtn is %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestPowerLimit(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.DragTeachSwitch(1);
     robot.SetPowerLimit(1, 200);
     float torques[] = { 0, 0, 0, 0, 0, 0 };
     robot.GetJointTorques(1, torques);

     int count = 100;
     robot.ServoJTStart(); 
     int error = 0;
     while (count > 0)
     {
         error = robot.ServoJT(torques, 0.001);
         count = count - 1;
         robot.Sleep(1);
     }
     error = robot.ServoJTEnd();
     robot.DragTeachSwitch(0);

     robot.CloseRPC();
     return 0;
 }

 int TestServoJT(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.DragTeachSwitch(1);
     float torques[] = { 0, 0, 0, 0, 0, 0 };
     robot.GetJointTorques(1, torques);

     int count = 100;
     robot.ServoJTStart(); 
     int error = 0;
     while (count > 0)
     {
         error = robot.ServoJT(torques, 0.001);
         count = count - 1;
         robot.Sleep(1);
     }
     error = robot.ServoJTEnd();
     robot.DragTeachSwitch(0);

     robot.CloseRPC();
     return 0;
 }


 int TestGetStatus(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     float yangle, zangle;
     robot.GetRobotInstallAngle(&yangle, &zangle);
     printf("yangle:%f,zangle:%f\n", yangle, zangle);

     JointPos j_deg = {};
     robot.GetActualJointPosDegree(0, &j_deg);
     printf("joint pos deg:%f,%f,%f,%f,%f,%f\n", j_deg.jPos[0], j_deg.jPos[1], j_deg.jPos[2], j_deg.jPos[3], j_deg.jPos[4], j_deg.jPos[5]);

     float jointSpeed[6] = { 0.0 };
     robot.GetActualJointSpeedsDegree(0, jointSpeed);
     printf("joint speeds deg:%f,%f,%f,%f,%f,%f\n", jointSpeed[0], jointSpeed[1], jointSpeed[2], jointSpeed[3], jointSpeed[4], jointSpeed[5]);

     float jointAcc[6] = { 0.0 };
     robot.GetActualJointAccDegree(0, jointAcc);
     printf("joint acc deg:%f,%f,%f,%f,%f,%f\n", jointAcc[0], jointAcc[1], jointAcc[2], jointAcc[3], jointAcc[4], jointAcc[5]);

     float tcp_speed = 0.0;
     float ori_speed = 0.0;
     robot.GetTargetTCPCompositeSpeed(0, &tcp_speed, &ori_speed);
     printf("GetTargetTCPCompositeSpeed tcp %f;  ori  %f\n", tcp_speed, ori_speed);

     robot.GetActualTCPCompositeSpeed(0, &tcp_speed, &ori_speed);
     printf("GetActualTCPCompositeSpeed tcp %f;  ori  %f\n", tcp_speed, ori_speed);
     
     float targetSpeed[6] = { 0.0 };
     robot.GetTargetTCPSpeed(0, targetSpeed);
     printf("GetTargetTCPSpeed  %f,%f,%f,%f,%f,%f\n", targetSpeed[0], targetSpeed[1], targetSpeed[2], targetSpeed[3], targetSpeed[4], targetSpeed[5]);

     float actualSpeed[6] = { 0.0 };
     robot.GetActualTCPSpeed(0, actualSpeed);
     printf("GetTargetTCPSpeed  %f,%f,%f,%f,%f,%f\n", actualSpeed[0], actualSpeed[1], actualSpeed[2], actualSpeed[3], actualSpeed[4], actualSpeed[5]);

     DescPose tcp = {};
     robot.GetActualTCPPose(0, &tcp);
     printf("tcp pose:%f,%f,%f,%f,%f,%f\n", tcp.tran.x, tcp.tran.y, tcp.tran.z, tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz);

     DescPose flange = {};
     robot.GetActualToolFlangePose(0, &flange);
     printf("flange pose:%f,%f,%f,%f,%f,%f\n", flange.tran.x, flange.tran.y, flange.tran.z, flange.rpy.rx, flange.rpy.ry, flange.rpy.rz);

     int id = 0;
     robot.GetActualTCPNum(0, &id);
     printf("tcp num:%d\n", id);

     robot.GetActualWObjNum(0, &id);
     printf("wobj num:%d\n", id);

     float jtorque[6] = { 0.0 };
     robot.GetJointTorques(0, jtorque);
     printf("torques:%f,%f,%f,%f,%f,%f\n", jtorque[0], jtorque[1], jtorque[2], jtorque[3], jtorque[4], jtorque[5]);

     float t_ms = 0.0;
     robot.GetSystemClock(&t_ms);
     printf("system clock:%f\n", t_ms);

     int config = 0;
     robot.GetRobotCurJointsConfig(&config);
     printf("joint config:%d\n", config);

     uint8_t motionDone = 0;
     robot.GetRobotMotionDone(&motionDone);
     printf("GetRobotMotionDone :%d\n", motionDone);

     int len = 0;
     robot.GetMotionQueueLength(&len);
     printf("GetMotionQueueLength :%d\n", len);

     uint8_t emergState = 0;
     robot.GetRobotEmergencyStopState(&emergState);
     printf("GetRobotEmergencyStopState :%d\n", emergState);

     int comstate = 0;
     robot.GetSDKComState(&comstate);
     printf("GetSDKComState :%d\n", comstate);

     uint8_t si0_state, si1_state;
     robot.GetSafetyStopState(&si0_state, &si1_state);
     printf("GetSafetyStopState :%d  %d\n", si0_state, si1_state);

     double temp[6] = { 0.0 };
     robot.GetJointDriverTemperature(temp);
     printf("Temperature:%f,%f,%f,%f,%f,%f\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);

     double torque[6] = { 0.0 };
     robot.GetJointDriverTorque(torque);
     printf("torque:%f,%f,%f,%f,%f,%f\n", torque[0], torque[1], torque[2], torque[3], torque[4], torque[5]);

     robot.GetRobotRealTimeState(&pkg);

     robot.CloseRPC();
     return 0;
 }

 int TestInverseKin(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);

     JointPos inverseRtn = {};

     robot.GetInverseKin(0, &desc_pos1, -1, &inverseRtn);
     printf("dcs1 GetInverseKin rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
     robot.GetInverseKinRef(0, &desc_pos1, &j1, &inverseRtn);
     printf("dcs1 GetInverseKinRef rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);

     uint8_t hasResut = 0;
     robot.GetInverseKinHasSolution(0, &desc_pos1, &j1, &hasResut);
     printf("dcs1 GetInverseKinRef result %d\n", hasResut);

     DescPose forwordResult = {};
     robot.GetForwardKin(&j1, &forwordResult);
     printf("jpos1 forwordResult rtn is %f %f %f %f %f %f \n", forwordResult.tran.x, forwordResult.tran.y, forwordResult.tran.z, forwordResult.rpy.rx, forwordResult.rpy.ry, forwordResult.rpy.rz);

     robot.CloseRPC();
     return 0;
 }

 int TestGetTeachPoint(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     char name[64] = "P1";
     float data[20] = { 0 };
     rtn = robot.GetRobotTeachingPoint(name, data);
     printf(" %d name is: %s \n", rtn, name);
     for (int i = 0; i < 20; i++)
     {
         printf("data is: %f \n", data[i]);
     }

     int que_len = 0;
     rtn = robot.GetMotionQueueLength(&que_len);
     printf("GetMotionQueueLength rtn is: %d, queue length is: %d \n", rtn, que_len);

     double dh[6] = { 0 };
     int retval = 0;
     retval = robot.GetDHCompensation(dh);
     cout << "retval is: " << retval << endl;
     cout << "dh is: " << dh[0] << " " << dh[1] << " " << dh[2] << " " << dh[3] << " " << dh[4] << " " << dh[5] << endl;

     string SN = "";
     robot.GetRobotSN(SN);
     cout << "robot SN is " << SN << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestTPD(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int type = 1;
     char name[30] = "tpd2025";
     int period_ms = 4;
     uint16_t di_choose = 0;
     uint16_t do_choose = 0;

     robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);

     robot.Mode(1);
     robot.Sleep(1000);
     robot.DragTeachSwitch(1);
     robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
     robot.Sleep(10000);
     robot.SetWebTPDStop();
     robot.DragTeachSwitch(0);

     float ovl = 100.0;
     uint8_t blend = 0;

     DescPose start_pose = {};

     rtn = robot.LoadTPD(name);
     printf("LoadTPD rtn is: %d\n", rtn);

     robot.GetTPDStartPose(name, &start_pose);
     printf("start pose, xyz is: %f %f %f. rpy is: %f %f %f \n", start_pose.tran.x, start_pose.tran.y, start_pose.tran.z, start_pose.rpy.rx, start_pose.rpy.ry, start_pose.rpy.rz);
     robot.MoveCart(&start_pose, 0, 0, 100, 100, ovl, -1, -1);
     robot.Sleep(1000);

     rtn = robot.MoveTPD(name, blend, ovl);
     printf("MoveTPD rtn is: %d\n", rtn);
     std::this_thread::sleep_for(std::chrono::milliseconds(5000));

     robot.SetTPDDelete(name);

     robot.CloseRPC();
     return 0;
 }
 int TestTraj(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     rtn = robot.TrajectoryJUpLoad("D://zUP/spray_traj1.txt");
     printf("Upload TrajectoryJ A %d\n", rtn);

     char traj_file_name[30] = "/fruser/traj/spray_traj1.txt";
     rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
     printf("LoadTrajectoryJ %s, rtn is: %d\n", traj_file_name, rtn);

     DescPose traj_start_pose;
     memset(&traj_start_pose, 0, sizeof(DescPose));
     rtn = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
     printf("GetTrajectoryStartPose is: %d\n", rtn);
     printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);

     std::this_thread::sleep_for(std::chrono::seconds(1));

     robot.SetSpeed(50);
     robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);

     int traj_num = 0;
     rtn = robot.GetTrajectoryPointNum(&traj_num);
     printf("GetTrajectoryStartPose rtn is: %d, traj num is: %d\n", rtn, traj_num);

     rtn = robot.SetTrajectoryJSpeed(50.0);
     printf("SetTrajectoryJSpeed is: %d\n", rtn);

     ForceTorque traj_force;
     memset(&traj_force, 0, sizeof(ForceTorque));
     traj_force.fx = 10;
     rtn = robot.SetTrajectoryJForceTorque(&traj_force);
     printf("SetTrajectoryJForceTorque rtn is: %d\n", rtn);

     rtn = robot.SetTrajectoryJForceFx(10.0);
     printf("SetTrajectoryJForceFx rtn is: %d\n", rtn);

     rtn = robot.SetTrajectoryJForceFy(0.0);
     printf("SetTrajectoryJForceFy rtn is: %d\n", rtn);

     rtn = robot.SetTrajectoryJForceFz(0.0);
     printf("SetTrajectoryJForceFz rtn is: %d\n", rtn);

     rtn = robot.SetTrajectoryJTorqueTx(10.0);
     printf("SetTrajectoryJTorqueTx rtn is: %d\n", rtn);

     rtn = robot.SetTrajectoryJTorqueTy(10.0);
     printf("SetTrajectoryJTorqueTy rtn is: %d\n", rtn);

     rtn = robot.SetTrajectoryJTorqueTz(10.0);
     printf("SetTrajectoryJTorqueTz rtn is: %d\n", rtn);

     rtn = robot.MoveTrajectoryJ();
     printf("MoveTrajectoryJ rtn is: %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestLuaOp(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     char program_name[64] = "/fruser/test.lua";
     char loaded_name[64] = "";
     uint8_t state;
     int line;

     robot.Mode(0);
     robot.LoadDefaultProgConfig(0, program_name);
     robot.ProgramLoad(program_name);
     robot.ProgramRun();
     robot.Sleep(1000);
     robot.ProgramPause();
     robot.GetProgramState(&state);
     printf("program state:%u\n", state);
     robot.GetCurrentLine(&line);
     printf("current line:%d\n", line);
     robot.GetLoadedProgram(loaded_name);
     printf("program name:%s\n", loaded_name);
     robot.Sleep(1000);
     robot.ProgramResume();
     robot.Sleep(1000);
     robot.ProgramStop();
     robot.Sleep(1000);

     robot.CloseRPC();
     return 0;
 }

 int TestLUAUpDownLoad(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     /* 获取lua名称 */
     list<std::string> luaNames;
     rtn = robot.GetLuaList(&luaNames);
     std::cout << "res is: " << rtn << std::endl;
     std::cout << "size is: " << luaNames.size() << std::endl;
     for (auto it = luaNames.begin(); it != luaNames.end(); it++)
     {
         std::cout << it->c_str() << std::endl;
     }

     /* 下载lua */
     rtn = robot.LuaDownLoad("test.lua", "D://zDOWN/");
     printf("LuaDownLoad rtn is %d\n", rtn);

     /* 上传lua */
     rtn = robot.LuaUpload("D://zUP/airlab.lua");
     printf("LuaUpload rtn is %d\n", rtn);

     /* 删除lua */
     rtn = robot.LuaDelete("test.lua");
     printf("LuaDelete rtn is %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestGripper(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 4;
     int device = 0;
     int softversion = 0;
     int bus = 2;
     int index = 2;
     int act = 0;
     int max_time = 30000;
     uint8_t block = 0;
     uint8_t status;
     uint16_t fault;
     uint16_t active_status = 0;
     uint8_t current_pos = 0;
     int8_t current = 0;
     int voltage = 0;
     int temp = 0;
     int8_t speed = 0;

     robot.SetGripperConfig(company, device, softversion, bus);
     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
     robot.GetGripperConfig(&company, &device, &softversion, &bus);
     printf("gripper config:%d,%d,%d,%d\n", company, device, softversion, bus);

     robot.ActGripper(index, act);
     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
     act = 1;
     robot.ActGripper(index, act);
     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

     robot.MoveGripper(index, 100, 50, 50, max_time, block, 0, 0, 0, 0);
     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
     robot.MoveGripper(index, 0, 50, 0, max_time, block, 0, 0, 0, 0);

     robot.GetGripperMotionDone(&fault, &status);
     printf("motion status:%u,%u\n", fault, status);

     robot.GetGripperActivateStatus(&fault, &active_status);
     printf("gripper active fault is: %u, status is: %u\n", fault, active_status);

     robot.GetGripperCurPosition(&fault, &current_pos);
     printf("fault is:%u, current position is: %u\n", fault, current_pos);

     robot.GetGripperCurCurrent(&fault, &current);
     printf("fault is:%u, current current is: %d\n", fault, current);

     robot.GetGripperVoltage(&fault, &voltage);
     printf("fault is:%u, current voltage is: %d \n", fault, voltage);

     robot.GetGripperTemp(&fault, &temp);
     printf("fault is:%u, current temperature is: %d\n", fault, temp);

     robot.GetGripperCurSpeed(&fault, &speed);
     printf("fault is:%u, current speed is: %d\n", fault, speed);

     int retval = 0;
     DescPose prepick_pose = {};
     DescPose postpick_pose = {};

     DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     retval = robot.ComputePrePick(&p1Desc, 10, 0, &prepick_pose);
     printf("ComputePrePick retval is: %d\n", retval);
     printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", prepick_pose.tran.x, prepick_pose.tran.y, prepick_pose.tran.z, prepick_pose.rpy.rx, prepick_pose.rpy.ry, prepick_pose.rpy.rz);

     retval = robot.ComputePostPick(&p2Desc, -10, 0, &postpick_pose);
     printf("ComputePostPick retval is: %d\n", retval);
     printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", postpick_pose.tran.x, postpick_pose.tran.y, postpick_pose.tran.z, postpick_pose.rpy.rx, postpick_pose.rpy.ry, postpick_pose.rpy.rz);

     robot.CloseRPC();
     return 0;
 }

 int TestRotGripperState(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     uint16_t fault = 0;
     double rotNum = 0.0;
     int rotSpeed = 0;
     int rotTorque = 0;
     robot.GetGripperRotNum(&fault, &rotNum);
     robot.GetGripperRotSpeed(&fault, &rotSpeed);
     robot.GetGripperRotTorque(&fault, &rotTorque);
     printf("gripper rot num : %lf, gripper rotSpeed : %d, gripper rotTorque : %d\n", rotNum, rotSpeed, rotTorque);

     robot.CloseRPC();
     return 0;
 }

 int TestConveyor(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int retval = 0;

     retval = robot.ConveyorStartEnd(1);
     printf("ConveyorStartEnd retval is: %d\n", retval);

     retval = robot.ConveyorPointIORecord();
     printf("ConveyorPointIORecord retval is: %d\n", retval);

     retval = robot.ConveyorPointARecord();
     printf("ConveyorPointARecord retval is: %d\n", retval);

     retval = robot.ConveyorRefPointRecord();
     printf("ConveyorRefPointRecord retval is: %d\n", retval);

     retval = robot.ConveyorPointBRecord();
     printf("ConveyorPointBRecord retval is: %d\n", retval);

     retval = robot.ConveyorStartEnd(0);
     printf("ConveyorStartEnd retval is: %d\n", retval);

     retval = 0;
     float param[6] = { 1,10000,200,0,0,20 };
     retval = robot.ConveyorSetParam(param);
     printf("ConveyorSetParam retval is: %d\n", retval);

     double cmp[3] = { 0.0, 0.0, 0.0 };
     retval = robot.ConveyorCatchPointComp(cmp);
     printf("ConveyorCatchPointComp retval is: %d\n", retval);

     int index = 1;
     int max_time = 30000;
     uint8_t block = 0;
     retval = 0;

     DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);


     retval = robot.MoveCart(&p1Desc, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);
     printf("MoveCart retval is: %d\n", retval);

     retval = robot.WaitMs(1);
     printf("WaitMs retval is: %d\n", retval);

     retval = robot.ConveyorIODetect(10000);
     printf("ConveyorIODetect retval is: %d\n", retval);

     retval = robot.ConveyorGetTrackData(1);
     printf("ConveyorGetTrackData retval is: %d\n", retval);

     retval = robot.ConveyorTrackStart(1);
     printf("ConveyorTrackStart retval is: %d\n", retval);

     retval = robot.TrackMoveL("cvrCatchPoint", 1, 0, 100, 100, 100, -1.0, 0, 0);
     printf("TrackMoveL retval is: %d\n", retval);

     retval = robot.MoveGripper(index, 51, 40, 30, max_time, block, 0, 0, 0, 0);
     printf("MoveGripper retval is: %d\n", retval);

     retval = robot.TrackMoveL("cvrRaisePoint", 1, 0, 100, 100, 100, -1.0, 0, 0);
     printf("TrackMoveL retval is: %d\n", retval);

     retval = robot.ConveyorTrackEnd();
     printf("ConveyorTrackEnd retval is: %d\n", retval);

     robot.MoveCart(&p2Desc, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);

     retval = robot.MoveGripper(index, 100, 40, 10, max_time, block, 0, 0, 0, 0);
     printf("MoveGripper retval is: %d\n", retval);

     robot.CloseRPC();
     return 0;
 }

 int TestAxleSensor(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.AxleSensorConfig(18, 0, 0, 1);
     int company = -1;
     int type = -1;
     robot.AxleSensorConfigGet(company, type);
     printf("company is %d, type is %d\n", company, type);

     rtn = robot.AxleSensorActivate(1);
     printf("AxleSensorActivate rtn is %d\n", rtn);

     robot.Sleep(1000);

     rtn = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
     printf("AxleSensorRegWrite rtn is %d\n", rtn);


     robot.CloseRPC();
     return 0;
 }

 int TestExDevProtocol(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int protocol = 4096;
     rtn = robot.SetExDevProtocol(protocol);
     std::cout << "SetExDevProtocol rtn " << rtn << std::endl;
     rtn = robot.GetExDevProtocol(&protocol);
     std::cout << "GetExDevProtocol rtn " << rtn << " protocol is: " << protocol << std::endl;

     robot.CloseRPC();
     return 0;
 }

 int TestAxleLua(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua");

     AxleComParam param(7, 8, 1, 0, 5, 3, 1);
     robot.SetAxleCommunicationParam(param);

     AxleComParam getParam;
     robot.GetAxleCommunicationParam(&getParam);
     printf("GetAxleCommunicationParam param is %d %d %d %d %d %d %d\n", getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify, getParam.timeout, getParam.timeoutTimes, getParam.period);

     robot.SetAxleLuaEnable(1);
     int luaEnableStatus = 0;
     robot.GetAxleLuaEnableStatus(&luaEnableStatus);
     robot.SetAxleLuaEnableDeviceType(0, 1, 0);

     int forceEnable = 0;
     int gripperEnable = 0;
     int ioEnable = 0;
     robot.GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable);
     printf("GetAxleLuaEnableDeviceType param is %d %d %d\n", forceEnable, gripperEnable, ioEnable);

     int func[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
     robot.SetAxleLuaGripperFunc(1, func);
     int getFunc[16] = { 0 };
     robot.GetAxleLuaGripperFunc(1, getFunc);
     int getforceEnable[16] = { 0 };
     int getgripperEnable[16] = { 0 };
     int getioEnable[16] = { 0 };
     robot.GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable);
     printf("\ngetforceEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getforceEnable[i]);
     }
     printf("\ngetgripperEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getgripperEnable[i]);
     }
     printf("\ngetioEnable status : ");
     for (int i = 0; i < 16; i++)
     {
         printf("%d,", getioEnable[i]);
     }
     printf("\n");
     robot.ActGripper(1, 0);
     robot.Sleep(2000);
     robot.ActGripper(1, 1);
     robot.Sleep(2000);
     robot.MoveGripper(1, 90, 10, 100, 50000, 0, 0, 0, 0, 0);
     int pos = 0;
     while (true)
     {
         robot.GetRobotRealTimeState(&pkg);
         printf("gripper pos is %u\n", pkg.gripper_position);
         robot.Sleep(100);
     }

     robot.CloseRPC();
     return 0;
 }

 int TestSetWeldParam(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.WeldingSetProcessParam(1, 177, 27, 1000, 178, 28, 176, 26, 1000);
     robot.WeldingSetProcessParam(2, 188, 28, 555, 199, 29, 133, 23, 333);

     double startCurrent = 0;
     double startVoltage = 0;
     double startTime = 0;
     double weldCurrent = 0;
     double weldVoltage = 0;
     double endCurrent = 0;
     double endVoltage = 0;
     double endTime = 0;

     robot.WeldingGetProcessParam(1, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
     cout << "the Num 1 process param is " << startCurrent << " " << startVoltage << " " << startTime << " " << weldCurrent << " " << weldVoltage << " " << endCurrent << " " << endVoltage << " " << endTime << endl;
     robot.WeldingGetProcessParam(2, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
     cout << "the Num 2 process param is " << startCurrent << " " << startVoltage << " " << startTime << " " << weldCurrent << " " << weldVoltage << " " << endCurrent << " " << endVoltage << " " << endTime << endl;

     rtn = robot.WeldingSetCurrentRelation(0, 400, 0, 10, 0);
     cout << "WeldingSetCurrentRelation rtn is: " << rtn << endl;

     rtn = robot.WeldingSetVoltageRelation(0, 40, 0, 10, 1);
     cout << "WeldingSetVoltageRelation rtn is: " << rtn << endl;

     double current_min = 0;
     double current_max = 0;
     double vol_min = 0;
     double vol_max = 0;
     double output_vmin = 0;
     double output_vmax = 0;
     int curIndex = 0;
     int volIndex = 0;
     rtn = robot.WeldingGetCurrentRelation(&current_min, &current_max, &output_vmin, &output_vmax, &curIndex);
     cout << "WeldingGetCurrentRelation rtn is: " << rtn << endl;
     cout << "current min " << current_min << " current max " << current_max << " output vol min " << output_vmin << " output vol max " << output_vmax << endl;

     rtn = robot.WeldingGetVoltageRelation(&vol_min, &vol_max, &output_vmin, &output_vmax, &volIndex);
     cout << "WeldingGetVoltageRelation rtn is: " << rtn << endl;
     cout << "vol min " << vol_min << " vol max " << vol_max << " output vol min " << output_vmin << " output vol max " << output_vmax << endl;

     rtn = robot.WeldingSetCurrent(1, 100, 0, 0);
     cout << "WeldingSetCurrent rtn is: " << rtn << endl;

     this_thread::sleep_for(chrono::seconds(3));

     rtn = robot.WeldingSetVoltage(1, 10, 0, 0);
     cout << "WeldingSetVoltage rtn is: " << rtn << endl;

     rtn = robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 60.000000);
     cout << "rtn is: " << rtn << endl;

     robot.WeaveOnlineSetPara(0, 0, 1, 0, 20, 0, 0, 0, 0);

     rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200);
     printf("WeldingSetCheckArcInterruptionParam    %d\n", rtn);
     rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
     printf("WeldingSetReWeldAfterBreakOffParam    %d\n", rtn);
     int enable = 0;
     double length = 0;
     double velocity = 0;
     int moveType = 0;
     int checkEnable = 0;
     int arcInterruptTimeLength = 0;
     rtn = robot.WeldingGetCheckArcInterruptionParam(&checkEnable, &arcInterruptTimeLength);
     printf("WeldingGetCheckArcInterruptionParam  checkEnable  %d   arcInterruptTimeLength  %d\n", checkEnable, arcInterruptTimeLength);
     rtn = robot.WeldingGetReWeldAfterBreakOffParam(&enable, &length, &velocity, &moveType);
     printf("WeldingGetReWeldAfterBreakOffParam  enable = %d, length = %lf, velocity = %lf, moveType = %d\n", enable, length, velocity, moveType);

     robot.SetWeldMachineCtrlModeExtDoNum(17);
     for (int i = 0; i < 5; i++)
     {
         robot.SetWeldMachineCtrlMode(0);
         robot.Sleep(1000);
         robot.SetWeldMachineCtrlMode(1);
         robot.Sleep(1000);
     }
     robot.CloseRPC();
     return 0;
 }

 int TestWelding(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.WeldingSetCurrent(1, 230, 0, 0);
     robot.WeldingSetVoltage(1, 24, 0, 1);

     DescPose p1Desc(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
     JointPos p1Joint(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

     DescPose p2Desc(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
     JointPos p2Joint(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot.MoveJ(&p1Joint, &p1Desc, 13, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.ARCStart(1, 0, 10000);
     robot.WeaveStart(0);
     robot.MoveL(&p2Joint, &p2Desc, 13, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     robot.ARCEnd(1, 0, 10000);
     robot.WeaveEnd(0);

     robot.CloseRPC();
     return 0;
 }

 int TestSegWeld(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.WeldingSetCurrent(1, 230, 0, 0);
     robot.WeldingSetVoltage(1, 24, 0, 1);

     DescPose p1Desc(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
     JointPos p1Joint(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

     DescPose p2Desc(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
     JointPos p2Joint(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     rtn = robot.SegmentWeldStart(&p1Desc, &p2Desc, &p1Joint, &p2Joint, 20, 20, 0, 0, 5000, 0, 0, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     printf("SegmentWeldStart rtn is %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestWeave(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     DescPose p1Desc(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
     JointPos p1Joint(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

     DescPose p2Desc(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
     JointPos p2Joint(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     robot.MoveJ(&p1Joint, &p1Desc, 13, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.WeaveStartSim(0);
     robot.MoveL(&p2Joint, &p2Desc, 13, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     robot.WeaveEndSim(0);
     robot.MoveJ(&p1Joint, &p1Desc, 13, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.WeaveInspectStart(0);
     robot.MoveL(&p2Joint, &p2Desc, 13, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
     robot.WeaveInspectEnd(0);

     robot.WeldingSetVoltage(1, 19, 0, 0);
     robot.WeldingSetCurrent(1, 190, 0, 0);
     robot.MoveL(&p1Joint, &p1Desc, 1, 1, 100, 100, 50, -1, &exaxisPos, 0, 0, &offdese);
     robot.ARCStart(1, 0, 10000);
     robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
     robot.WeaveStart(0);
     robot.WeaveChangeStart(1, 0, 50, 30);
     robot.MoveL(&p2Joint, &p2Desc, 1, 1, 100, 100, 1, -1, &exaxisPos, 0, 0, &offdese);
     robot.WeaveChangeEnd();
     robot.WeaveEnd(0);
     robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
     robot.ARCEnd(1, 0, 10000);

     robot.CloseRPC();
     return 0;
 }

 int TestSSHMd5(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     char file_path[256] = "/fruser/airlab.lua";
     char md5[256] = { 0 };
     uint8_t emerg_state = 0;
     uint8_t si0_state = 0;
     uint8_t si1_state = 0;
     int sdk_com_state = 0;

     char ssh_keygen[1024] = { 0 };
     int retval = robot.GetSSHKeygen(ssh_keygen);
     printf("GetSSHKeygen retval is: %d\n", retval);
     printf("ssh key is: %s \n", ssh_keygen);

     char ssh_name[32] = "fr";
     char ssh_ip[32] = "192.168.58.45";
     char ssh_route[128] = "/home/fr";
     char ssh_robot_url[128] = "/root/robot/dhpara.config";
     retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url);
     printf("SetSSHScpCmd retval is: %d\n", retval);
     printf("robot url is: %s\n", ssh_robot_url);

     robot.ComputeFileMD5(file_path, md5);
     printf("md5 is: %s \n", md5);

     robot.CloseRPC();
     return 0;
 }

 int TestRealtimePeriod(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.SetRobotRealtimeStateSamplePeriod(10);
     int getPeriod = 0;
     robot.GetRobotRealtimeStateSamplePeriod(getPeriod);
     cout << "period is " << getPeriod << endl;
     robot.Sleep(1000);

     robot.CloseRPC();
     return 0;
 }

 int TestUpgrade(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(3);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.SoftwareUpgrade("D://zUP/QNX382/software.tar.gz", false);
     while (true)
     {
         int curState = -1;
         robot.GetSoftwareUpgradeState(curState);
         printf("upgrade state is %d\n", curState);
         robot.Sleep(300);
     }

     robot.CloseRPC();
     return 0;
 }

 int TestPointTable(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     string save_path = "D://zDOWN/";
     string point_table_name = "point_table_FR5.db";
     rtn = robot.PointTableDownLoad(point_table_name, save_path);
     cout << "download : " << point_table_name << " fail: " << rtn << endl;

     string upload_path = "D://zUP/point_table_FR5.db";
     rtn = robot.PointTableUpLoad(upload_path);
     cout << "retval is: " << rtn << endl;

     string point_tablename = "point_table_FR5.db";
     string lua_name = "airlab.lua";
     rtn = robot.PointTableUpdateLua(point_tablename, lua_name);
     cout << "retval is: " << rtn << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestDownLoadRobotData(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     rtn = robot.RbLogDownload("D://zDOWN/");
     cout << "RbLogDownload rtn is " << rtn << endl;

     rtn = robot.AllDataSourceDownload("D://zDOWN/");
     cout << "AllDataSourceDownload rtn is " << rtn << endl;

     rtn = robot.DataPackageDownload("D://zDOWN/");
     cout << "DataPackageDownload rtn is " << rtn << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestExtDIConfig(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.SetArcStartExtDoNum(10);
     robot.SetAirControlExtDoNum(20);
     robot.SetWireForwardFeedExtDoNum(30);
     robot.SetWireReverseFeedExtDoNum(40);

     robot.SetWeldReadyExtDiNum(50);
     robot.SetArcDoneExtDiNum(60);
     robot.SetExtDIWeldBreakOffRecover(70, 80);
     robot.SetWireSearchExtDIONum(0, 1);

     robot.CloseRPC();
     return 0;
 }

 int TestArcWeldTrace(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos mulitilineorigin1_joint(-24.090, -63.501, 84.288, -111.940, -93.426, 57.669);
     DescPose mulitilineorigin1_desc(-677.559, 190.951, -1.205, 1.144, -41.482, -82.577);

     DescTran mulitilineX1_desc;
     mulitilineX1_desc.x = -677.556;
     mulitilineX1_desc.y = 211.949;
     mulitilineX1_desc.z = -1.206;

     DescTran mulitilineZ1_desc;
     mulitilineZ1_desc.x = -677.564;
     mulitilineZ1_desc.y = 190.956;
     mulitilineZ1_desc.z = 19.817;

     JointPos mulitilinesafe_joint(-25.734, -63.778, 81.502, -108.975, -93.392, 56.021);
     DescPose mulitilinesafe_desc(-677.561, 211.950, 19.812, 1.144, -41.482, -82.577);
     JointPos mulitilineorigin2_joint(-29.743, -75.623, 101.241, -116.354, -94.928, 55.735);
     DescPose mulitilineorigin2_desc(-563.961, 215.359, -0.681, 2.845, -40.476, -87.443);

     DescTran mulitilineX2_desc;
     mulitilineX2_desc.x = -563.965;
     mulitilineX2_desc.y = 220.355;
     mulitilineX2_desc.z = -0.680;

     DescTran mulitilineZ2_desc;
     mulitilineZ2_desc.x = -563.968;
     mulitilineZ2_desc.y = 215.362;
     mulitilineZ2_desc.z = 4.331;

     ExaxisPos epos(0, 0, 0, 0);
     DescPose offset(0, 0, 0, 0, 0, 0);

     robot.Sleep(10);
     int error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return: %d\n", error);

     error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return: %d\n", error);

     error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return: %d\n", error);

     error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.ARCStart(1, 0, 3000);
     printf("ARCStart return: %d\n", error);

     error = robot.WeaveStart(0);
     printf("WeaveStart return: %d\n", error);

     error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
     printf("ArcWeldTraceControl return: %d\n", error);

     error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 1, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
     printf("ArcWeldTraceControl return: %d\n", error);

     error = robot.WeaveEnd(0);
     printf("WeaveEnd return: %d\n", error);

     error = robot.ARCEnd(1, 0, 10000);
     printf("ARCEnd return: %d\n", error);

     error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return: %d\n", error);

     error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0, offset);
     printf("MultilayerOffsetTrsfToBase return: %d  offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.ARCStart(1, 0, 3000);
     printf("ARCStart return: %d\n", error);

     error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0, offset);
     printf("MultilayerOffsetTrsfToBase return: %d  offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot.ArcWeldTraceReplayStart();
     printf("ArcWeldTraceReplayStart return: %d\n", error);

     error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.ArcWeldTraceReplayEnd();
     printf("ArcWeldTraceReplayEnd return: %d\n", error);

     error = robot.ARCEnd(1, 0, 10000);
     printf("ARCEnd return: %d\n", error);

     error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return: %d\n", error);

     error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0, offset);
     printf("MultilayerOffsetTrsfToBase return: %d  offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.ARCStart(1, 0, 3000);
     printf("ARCStart return: %d\n", error);

     error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0, offset);
     printf("MultilayerOffsetTrsfToBase return: %d  offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot.ArcWeldTraceReplayStart();
     printf("MoveJ return: %d\n", error);

     error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return: %d\n", error);

     error = robot.ArcWeldTraceReplayEnd();
     printf("ArcWeldTraceReplayEnd return: %d\n", error);

     error = robot.ARCEnd(1, 0, 3000);
     printf("ARCEnd return: %d\n", error);

     error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return: %d\n", error);

     robot.CloseRPC();
     return 0;
 }

 int TestWireSearch(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     DescPose toolCoord(0, 0, 200, 0, 0, 0);
     robot.SetToolCoord(1, &toolCoord, 0, 0, 1, 0);
     DescPose wobjCoord(0, 0, 0, 0, 0, 0);
     robot.SetWObjCoord(1, &wobjCoord, 0);

     int rtn0, rtn1, rtn2 = 0;
     ExaxisPos exaxisPos = { 0, 0, 0, 0 };
     DescPose offdese = { 0, 0, 0, 0, 0, 0 };


     DescPose descStart = { 216.543, 445.175, 93.465, 179.683, 1.757, -112.641 };
     JointPos jointStart = { -128.345, -86.660, 114.679, -119.625, -89.219, 74.303 };

     DescPose descEnd = { 111.143, 523.384, 87.659, 179.703, 1.835, -97.750 };
     JointPos jointEnd = { -113.454, -81.060, 109.328, -119.954, -89.218, 74.302 };

     robot.MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot.MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);

     DescPose descREF0A = { 142.135, 367.604, 86.523, 179.728, 1.922, -111.089 };
     JointPos jointREF0A = { -126.794, -100.834, 128.922, -119.864, -89.218, 74.302 };

     DescPose descREF0B = { 254.633, 463.125, 72.604, 179.845, 2.341, -114.704 };
     JointPos jointREF0B = { -130.413, -81.093, 112.044, -123.163, -89.217, 74.303 };

     DescPose descREF1A = { 92.556, 485.259, 47.476, -179.932, 3.130, -97.512 };
     JointPos jointREF1A = { -113.231, -83.815, 119.877, -129.092, -89.217, 74.303 };

     DescPose descREF1B = { 203.103, 583.836, 63.909, 179.991, 2.854, -103.372 };
     JointPos jointREF1B = { -119.088, -69.676, 98.692, -121.761, -89.219, 74.303 };

     rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot.MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot.MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot.WireSearchWait("REF0");
     rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot.MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot.MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot.WireSearchWait("REF1");
     rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot.MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot.MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot.WireSearchWait("RES0");
     rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot.MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot.MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot.WireSearchWait("RES1");
     rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
     vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
     int offectFlag = 0;
     DescPose offectPos = { 0, 0, 0, 0, 0, 0 };
     rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
     robot.PointsOffsetEnable(0, &offectPos);
     robot.MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot.MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);
     robot.PointsOffsetDisable();

     robot.CloseRPC();
     return 0;
 }

 int TestFTInit(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 24;
     int device = 0;
     int softversion = 0;
     int bus = 1;
     int index = 1;

     robot.FT_SetConfig(company, device, softversion, bus);
     robot.Sleep(1000);
     robot.FT_GetConfig(&company, &device, &softversion, &bus);
     printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
     robot.Sleep(1000);

     robot.FT_Activate(0);
     robot.Sleep(1000);
     robot.FT_Activate(1);
     robot.Sleep(1000);

     robot.Sleep(1000);
     robot.FT_SetZero(0);
     robot.Sleep(1000);

     ForceTorque ft;
     memset(&ft, 0, sizeof(ForceTorque));
     robot.FT_GetForceTorqueOrigin(0, &ft);
     printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx, ft.fy, ft.fz, ft.tx, ft.ty, ft.tz);
     robot.FT_SetZero(1);
     robot.Sleep(1000);

     DescPose ftCoord = {};
     robot.FT_SetRCS(0, ftCoord);

     robot.SetForceSensorPayload(0.824);
     robot.SetForceSensorPayloadCog(0.778, 2.554, 48.765);
     double weight = 0;
     double x = 0, y = 0, z = 0;
     robot.GetForceSensorPayload(weight);
     robot.GetForceSensorPayloadCog(x, y, z);
     printf("the FT load is  %lf, %lf %lf %lf\n", weight, x, y, z);

     robot.SetForceSensorPayload(0);
     robot.SetForceSensorPayloadCog(0, 0, 0);

     double computeWeight = 0;
     DescTran tran = {};
     robot.ForceSensorAutoComputeLoad(weight, tran);
     cout << "the result is weight " << weight << " pos is " << tran.x << " " << tran.y << " " << tran.z << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestFTLoadCompute(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 24;
     int device = 0;
     int softversion = 0;
     int bus = 1;
     int index = 1;

     robot.FT_SetConfig(company, device, softversion, bus);
     robot.Sleep(1000);
     robot.FT_GetConfig(&company, &device, &softversion, &bus);
     printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
     robot.Sleep(1000);

     robot.FT_Activate(0);
     robot.Sleep(1000);
     robot.FT_Activate(1);
     robot.Sleep(1000);

     robot.Sleep(1000);
     robot.FT_SetZero(0);
     robot.Sleep(1000);

     ForceTorque ft;
     memset(&ft, 0, sizeof(ForceTorque));
     robot.FT_GetForceTorqueOrigin(0, &ft);
     printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx, ft.fy, ft.fz, ft.tx, ft.ty, ft.tz);
     robot.FT_SetZero(1);
     robot.Sleep(1000);

     DescPose tcoord = {};
     tcoord.tran.z = 35.0;
     robot.SetToolCoord(10, &tcoord, 1, 0, 0, 0);

     robot.FT_PdIdenRecord(10);
     robot.Sleep(1000);

     float weight = 0.0;
     robot.FT_PdIdenCompute(&weight);
     printf("payload weight:%f\n", weight);

     DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
     DescPose desc_p3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);

     robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
     robot.Sleep(1000);
     robot.FT_PdCogIdenRecord(10, 1);
     robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
     robot.Sleep(1000);
     robot.FT_PdCogIdenRecord(10, 2);
     robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
     robot.Sleep(1000);
     robot.FT_PdCogIdenRecord(10, 3);
     robot.Sleep(1000);
     DescTran cog;
     memset(&cog, 0, sizeof(DescTran));
     robot.FT_PdCogIdenCompute(&cog);
     printf("cog:%f,%f,%f\n", cog.x, cog.y, cog.z);

     robot.CloseRPC();
     return 0;
 }

 int TestFTGuard(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 24;
     int device = 0;
     int softversion = 0;
     int bus = 1;
     int index = 1;

     robot.FT_SetConfig(company, device, softversion, bus);
     robot.Sleep(1000);
     robot.FT_GetConfig(&company, &device, &softversion, &bus);
     printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
     robot.Sleep(1000);

     robot.FT_Activate(0);
     robot.Sleep(1000);
     robot.FT_Activate(1);
     robot.Sleep(1000);

     robot.Sleep(1000);
     robot.FT_SetZero(0);
     robot.Sleep(1000);

     uint8_t sensor_id = 1;
     uint8_t select[6] = { 1,1,1,1,1,1 };
     float max_threshold[6] = { 10.0,10.0,10.0,10.0,10.0,10.0 };
     float min_threshold[6] = { 5.0,5.0,5.0,5.0,5.0,5.0 };

     ForceTorque ft;
     DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
     DescPose desc_p3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);

     robot.FT_Guard(1, sensor_id, select, &ft, max_threshold, min_threshold);
     robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
     robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
     robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);

     robot.FT_Guard(0, sensor_id, select, &ft, max_threshold, min_threshold);

     robot.CloseRPC();
     return 0;
 }

 int TestFTControl(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 24;
     int device = 0;
     int softversion = 0;
     int bus = 1;
     int index = 1;

     robot.FT_SetConfig(company, device, softversion, bus);
     robot.Sleep(1000);
     robot.FT_GetConfig(&company, &device, &softversion, &bus);
     printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
     robot.Sleep(1000);

     robot.FT_Activate(0);
     robot.Sleep(1000);
     robot.FT_Activate(1);
     robot.Sleep(1000);

     robot.Sleep(1000);
     robot.FT_SetZero(0);
     robot.Sleep(1000);

     uint8_t sensor_id = 1;
     uint8_t select[6] = { 0,0,1,0,0,0 };
     float ft_pid[6] = { 0.0005,0.0,0.0,0.0,0.0,0.0 };
     uint8_t adj_sign = 0;
     uint8_t ILC_sign = 0;
     float max_dis = 100.0;
     float max_ang = 0.0;

     ForceTorque ft;
     ExaxisPos epos(0, 0, 0, 0);
     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
     DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
     DescPose offset_pos(0, 0, 0, 0, 0, 0);

     ft.fz = -10.0;

     rtn = robot.MoveJ(&j1, &desc_p1, 0, 0, 100.0, 180.0, 100.0, &epos, -1.0, 0, &offset_pos);
     rtn = robot.FT_Control(1, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
     printf("FT_Control start rtn is %d\n", rtn);
     rtn = robot.MoveL(&j2, &desc_p2, 0, 0, 100.0, 180.0, 20.0, -1.0, &epos, 0, 0, &offset_pos);
     rtn = robot.FT_Control(0, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
     printf("FT_Control end rtn is %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestFTSearch(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 24;
     int device = 0;
     int softversion = 0;
     int bus = 1;
     int index = 1;

     robot.FT_SetConfig(company, device, softversion, bus);
     robot.Sleep(1000);
     robot.FT_GetConfig(&company, &device, &softversion, &bus);
     printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
     robot.Sleep(1000);

     robot.FT_Activate(0);
     robot.Sleep(1000);
     robot.FT_Activate(1);
     robot.Sleep(1000);

     robot.Sleep(1000);
     robot.FT_SetZero(0);
     robot.Sleep(1000);

     //恒力参数
     uint8_t status = 1;  //恒力控制开启标志，0-关，1-开
     int sensor_num = 1; //力传感器编号
     float gain[6] = { 0.0001,0.0,0.0,0.0,0.0,0.0 };  //最大阈值
     uint8_t adj_sign = 0;  //自适应启停状态，0-关闭，1-开启
     uint8_t ILC_sign = 0;  //ILC控制启停状态，0-停止，1-训练，2-实操
     float max_dis = 100.0;  //最大调整距离
     float max_ang = 5.0;  //最大调整角度

     ForceTorque ft;
     memset(&ft, 0, sizeof(ForceTorque));

     //螺旋线探索参数
     int rcs = 0;  //参考坐标系，0-工具坐标系，1-基坐标系
     float dr = 0.7;  //每圈半径进给量，单位mm
     float fFinish = 1.0; //力或力矩阈值（0~100），单位N或Nm
     float t = 60000.0; //最大探索时间，单位ms
     float vmax = 3.0; //线速度最大值，单位mm/s

     //直线插入参数
     float force_goal = 20.0;  //力或力矩阈值（0~100），单位N或Nm
     float lin_v = 0.0; //直线速度，单位mm/s
     float lin_a = 0.0; //直线加速度，单位mm/s^2,暂不使用
     float disMax = 100.0; //最大插入距离，单位mm
     uint8_t linorn = 1; //插入方向，1-正方向，2-负方向

     //旋转插入参数
     float angVelRot = 2.0;  //旋转角速度，单位°/s
     float forceInsertion = 1.0; //力或力矩阈值（0~100），单位N或Nm
     int angleMax = 45; //最大旋转角度，单位°
     uint8_t orn = 1; //力的方向，1-fz,2-mz
     float angAccmax = 0.0; //最大旋转角加速度，单位°/s^2,暂不使用
     uint8_t rotorn = 1; //旋转方向，1-顺时针，2-逆时针

     uint8_t select1[6] = { 0,0,1,1,1,0 }; //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
     ft.fz = -10.0;
     robot.FT_Control(status, sensor_num, select1, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
     rtn = robot.FT_SpiralSearch(rcs, dr, fFinish, t, vmax);
     printf("FT_SpiralSearch rtn is %d\n", rtn);
     status = 0;
     robot.FT_Control(status, sensor_num, select1, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

     uint8_t select2[6] = { 1,1,1,0,0,0 };  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
     gain[0] = 0.00005;
     ft.fz = -30.0;
     status = 1;
     robot.FT_Control(status, sensor_num, select2, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
     rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn);
     printf("FT_LinInsertion rtn is %d\n", rtn);
     status = 0;
     robot.FT_Control(status, sensor_num, select2, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

     uint8_t select3[6] = { 0,0,1,1,1,0 };  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
     ft.fz = -10.0;
     gain[0] = 0.0001;
     status = 1;
     robot.FT_Control(status, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
     rtn = robot.FT_RotInsertion(rcs, angVelRot, forceInsertion, angleMax, orn, angAccmax, rotorn);
     printf("FT_RotInsertion rtn is %d\n", rtn);
     status = 0;
     robot.FT_Control(status, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

     uint8_t select4[6] = { 1,1,1,0,0,0 };  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
     ft.fz = -30.0;
     status = 1;
     robot.FT_Control(status, sensor_num, select4, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
     rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn);
     printf("FT_LinInsertion rtn is %d\n", rtn);
     status = 0;
     robot.FT_Control(status, sensor_num, select4, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

     robot.CloseRPC();
     return 0;
 }

 int TestSurface(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 24;
     int device = 0;
     int softversion = 0;
     int bus = 1;
     int index = 1;

     robot.FT_SetConfig(company, device, softversion, bus);
     robot.Sleep(1000);
     robot.FT_GetConfig(&company, &device, &softversion, &bus);
     printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
     robot.Sleep(1000);

     robot.FT_Activate(0);
     robot.Sleep(1000);
     robot.FT_Activate(1);
     robot.Sleep(1000);

     robot.Sleep(1000);
     robot.FT_SetZero(0);
     robot.Sleep(1000);

     int rcs = 0;
     uint8_t dir = 1;
     uint8_t axis = 1;
     float lin_v = 3.0;
     float lin_a = 0.0;
     float maxdis = 50.0;
     float ft_goal = 2.0;
     DescPose desc_pos(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose xcenter(0, 0, 0, 0, 0, 0);
     DescPose ycenter(0, 0, 0, 0, 0, 0);

     ForceTorque ft;
     memset(&ft, 0, sizeof(ForceTorque));

     ft.fx = -2.0;

     robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);

     robot.FT_CalCenterStart();
     robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
     robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
     robot.WaitMs(1000);

     dir = 2;
     robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
     robot.FT_CalCenterEnd(&xcenter);
     printf("xcenter:%f,%f,%f,%f,%f,%f\n", xcenter.tran.x, xcenter.tran.y, xcenter.tran.z, xcenter.rpy.rx, xcenter.rpy.ry, xcenter.rpy.rz);
     robot.MoveCart(&xcenter, 9, 0, 60.0, 50.0, 50.0, -1.0, -1);

     robot.FT_CalCenterStart();
     dir = 1;
     axis = 2;
     lin_v = 6.0;
     maxdis = 150.0;
     robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
     robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
     robot.WaitMs(1000);

     dir = 2;
     robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
     robot.FT_CalCenterEnd(&ycenter);
     printf("ycenter:%f,%f,%f,%f,%f,%f\n", ycenter.tran.x, ycenter.tran.y, ycenter.tran.z, ycenter.rpy.rx, ycenter.rpy.ry, ycenter.rpy.rz);
     robot.MoveCart(&ycenter, 9, 0, 60.0, 50.0, 50.0, 0.0, -1);

     robot.CloseRPC();
     return 0;
 }

 int TestCompliance(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int company = 24;
     int device = 0;
     int softversion = 0;
     int bus = 1;
     int index = 1;

     robot.FT_SetConfig(company, device, softversion, bus);
     robot.Sleep(1000);
     robot.FT_GetConfig(&company, &device, &softversion, &bus);
     printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
     robot.Sleep(1000);

     robot.FT_Activate(0);
     robot.Sleep(1000);
     robot.FT_Activate(1);
     robot.Sleep(1000);

     robot.Sleep(1000);
     robot.FT_SetZero(0);
     robot.Sleep(1000);

     uint8_t flag = 1;
     int sensor_id = 1;
     uint8_t select[6] = { 1,1,1,0,0,0 };
     float ft_pid[6] = { 0.0005,0.0,0.0,0.0,0.0,0.0 };
     uint8_t adj_sign = 0;
     uint8_t ILC_sign = 0;
     float max_dis = 100.0;
     float max_ang = 0.0;

     ForceTorque ft;
     DescPose  offset_pos(0, 0, 0, 0, 0, 0);
     ExaxisPos epos(0, 0, 0, 0);

     memset(&ft, 0, sizeof(ForceTorque));

     JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
     DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     ft.fx = -10.0;
     ft.fy = -10.0;
     ft.fz = -10.0;
     robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
     float p = 0.00005;
     float force = 30.0;
     rtn = robot.FT_ComplianceStart(p, force);
     printf("FT_ComplianceStart rtn is %d\n", rtn);
     int count = 15;
     while (count)
     {
         robot.MoveL(&j1, &desc_p1, 0, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 1, &offset_pos);
         robot.MoveL(&j2, &desc_p2, 0, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 0, &offset_pos);
         count -= 1;
     }
     robot.FT_ComplianceStop();
     printf("FT_ComplianceStop rtn is %d\n", rtn);
     flag = 0;
     robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

     robot.CloseRPC();
     return 0;
 }

 int TestEndForceDragCtrl(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.SetForceSensorDragAutoFlag(1);

     vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
     vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
     vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
     vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
     robot.EndForceDragControl(1, 0, 0, 0, 1, M, B, K, F, 50, 100);

     robot.Sleep(5000);

     int dragState = 0;
     int sixDimensionalDragState = 0;
     robot.GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
     printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);

     robot.EndForceDragControl(0, 0, 0, 0, 1, M, B, K, F, 50, 100);

     robot.CloseRPC();
     return 0;
 }

 int TestForceAndJointImpedance(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.DragTeachSwitch(1);
     vector <double> lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
     vector <double> KGain = { 0, 0, 0, 0, 0, 0 };
     vector <double> BGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
     rtn = robot.ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000, 180);
     printf("ForceAndJointImpedanceStartStop rtn is %d\n", rtn);

     robot.Sleep(5000);

     robot.DragTeachSwitch(0);
     rtn = robot.ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000, 180);
     printf("ForceAndJointImpedanceStartStop rtn is %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int Test485Auxservo(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45);
     std::cout << "AuxServoSetParam is: " << retval << std::endl;

     int servoCompany;
     int servoModel;
     int servoSoftVersion;
     int servoResolution;
     double axisMechTransRatio;
     retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
     std::cout << "servoCompany " << servoCompany << "\n"
         << "servoModel " << servoModel << "\n"
         << "servoSoftVersion " << servoSoftVersion << "\n"
         << "servoResolution " << servoResolution << "\n"
         << "axisMechTransRatio " << axisMechTransRatio << "\n"
         << std::endl;

     retval = robot.AuxServoSetParam(1, 10, 11, 12, 13, 14);
     std::cout << "AuxServoSetParam is: " << retval << std::endl;

     retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
     std::cout << "servoCompany " << servoCompany << "\n"
         << "servoModel " << servoModel << "\n"
         << "servoSoftVersion " << servoSoftVersion << "\n"
         << "servoResolution " << servoResolution << "\n"
         << "axisMechTransRatio " << axisMechTransRatio << "\n"
         << std::endl;

     retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36);
     std::cout << "AuxServoSetParam is: " << retval << std::endl;
     robot.Sleep(3000);

     robot.AuxServoSetAcc(3000, 3000);
     robot.AuxServoSetEmergencyStopAcc(5000, 5000);
     robot.Sleep(1000);
     double emagacc = 0, acc = 0;
     double emagdec = 0, dec = 0;
     robot.AuxServoGetEmergencyStopAcc(emagacc, emagdec);
     printf("emergency acc is %f  dec is %f \n", emagacc, emagdec);
     robot.AuxServoGetAcc(acc, dec);
     printf("acc is %f  dec is %f \n", acc, dec);

     robot.AuxServoSetControlMode(1, 0);
     robot.Sleep(2000);

     retval = robot.AuxServoEnable(1, 0);
     std::cout << "AuxServoEnable disenable " << retval << std::endl;
     robot.Sleep(1000);
     int servoerrcode = 0;
     int servoErrCode;
     int servoState;
     double servoPos;
     double servoSpeed;
     double servoTorque;
     retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
     std::cout << "AuxServoGetStatus servoState " << servoState << std::endl;
     robot.Sleep(1000);;

     retval = robot.AuxServoEnable(1, 1);
     std::cout << "AuxServoEnable enable " << retval << std::endl;
     robot.Sleep(1000);
     retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
     std::cout << "AuxServoGetStatus servoState " << servoState << std::endl;
     robot.Sleep(1000);

     retval = robot.AuxServoHoming(1, 1, 5, 1);
     std::cout << "AuxServoHoming " << retval << std::endl;
     robot.Sleep(3000);

     retval = robot.AuxServoSetTargetPos(1, 200, 30);
     std::cout << "AuxServoSetTargetPos " << retval << std::endl;
     robot.Sleep(1000);
     retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
     std::cout << "AuxServoGetStatus servoSpeed " << servoSpeed << std::endl;
     robot.Sleep(8000);


     robot.AuxServoSetControlMode(1, 1);
     robot.Sleep(2000);

     robot.AuxServoEnable(1, 0);
     robot.Sleep(1000);
     robot.AuxServoEnable(1, 1);
     robot.Sleep(1000);
     robot.AuxServoSetTargetSpeed(1, 100, 80);

     robot.Sleep(5000);
     robot.AuxServoSetTargetSpeed(1, 0, 80);


     robot.CloseRPC();
     return 0;
 }

 int TestUDPAxis(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);
     cout << "ExtDevSetUDPComParam rtn is " << rtn << endl;
     string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
     rtn = robot.ExtDevGetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum);
     string patam = "\nip " + ip + "\nport " + to_string(port) + "\nperiod  " + to_string(period) + "\nlossPkgTime " + to_string(lossPkgTime) + "\nlossPkgNum  " + to_string(lossPkgNum) + "\ndisConntime  " + to_string(disconnectTime) + "\nreconnecable  " + to_string(reconnectEnable) + "\nreconnperiod  " + to_string(reconnectPeriod) + "\nreconnnun  " + to_string(reconnectNum);
     cout << "ExtDevGetUDPComParam rtn is " << rtn << patam << endl;

     robot.ExtDevLoadUDPDriver();

     rtn = robot.ExtAxisServoOn(1, 1);
     cout << "ExtAxisServoOn axis id 1 rtn is " << rtn << endl;
     rtn = robot.ExtAxisServoOn(2, 1);
     cout << "ExtAxisServoOn axis id 2 rtn is " << rtn << endl;
     robot.Sleep(2000);

     robot.ExtAxisSetHoming(1, 0, 10, 2);
     robot.Sleep(2000);
     rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
     cout << "ExtAxisSetHoming rtnn is  " << rtn << endl;

     robot.Sleep(4000);

     rtn = robot.SetRobotPosToAxis(1);
     cout << "SetRobotPosToAxis rtn is " << rtn << endl;
     rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0);
     cout << "SetAxisDHParaConfig rtn is " << rtn << endl;
     rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
     cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
     rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);
     cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;

     robot.Sleep(1000 * 3);
     robot.ExtAxisStartJog(1, 0, 10, 10, 30);
     robot.Sleep(1000 * 1);
     robot.ExtAxisStopJog(1);
     robot.Sleep(1000 * 3);
     robot.ExtAxisServoOn(1, 0);

     robot.Sleep(1000 * 3);
     robot.ExtAxisStartJog(2, 0, 10, 10, 30);
     robot.Sleep(1000 * 1);
     robot.ExtAxisStopJog(2);
     robot.Sleep(1000 * 3);
     robot.ExtAxisServoOn(2, 0);

     robot.ExtDevUnloadUDPDriver();
     
     robot.CloseRPC();
     return 0;
 }

 int TestUDPAxisCalib(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);
     cout << "ExtDevSetUDPComParam rtn is " << rtn << endl;
     string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
     rtn = robot.ExtDevGetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum);
     string patam = "\nip " + ip + "\nport " + to_string(port) + "\nperiod  " + to_string(period) + "\nlossPkgTime " + to_string(lossPkgTime) + "\nlossPkgNum  " + to_string(lossPkgNum) + "\ndisConntime  " + to_string(disconnectTime) + "\nreconnecable  " + to_string(reconnectEnable) + "\nreconnperiod  " + to_string(reconnectPeriod) + "\nreconnnun  " + to_string(reconnectNum);
     cout << "ExtDevGetUDPComParam rtn is " << rtn << patam << endl;

     robot.ExtDevLoadUDPDriver();

     rtn = robot.ExtAxisServoOn(1, 1);
     cout << "ExtAxisServoOn axis id 1 rtn is " << rtn << endl;
     rtn = robot.ExtAxisServoOn(2, 1);
     cout << "ExtAxisServoOn axis id 2 rtn is " << rtn << endl;
     robot.Sleep(2000);

     robot.ExtAxisSetHoming(1, 0, 10, 2);
     robot.Sleep(2000);
     rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
     cout << "ExtAxisSetHoming rtnn is  " << rtn << endl;

     robot.Sleep(4000);

     rtn = robot.SetRobotPosToAxis(1);
     cout << "SetRobotPosToAxis rtn is " << rtn << endl;
     rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4,  0, 0, 0, 0, 0, 0);
     cout << "SetAxisDHParaConfig rtn is " << rtn << endl;
     rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
     cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
     rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);
     cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;

     DescPose toolCoord(0, 0, 210, 0, 0, 0);
     robot.SetToolCoord(1, &toolCoord, 0, 0, 1, 0);

     JointPos jSafe(115.193, -96.149, 92.489, -87.068, -89.15, -83.488);
     JointPos j1(117.559, -92.624, 100.329, -96.909, -94.057, -83.488);
     JointPos j2(112.239, -90.096, 99.282, -95.909, -89.824, -83.488);
     JointPos j3(110.839, -83.473, 93.166, -89.22, -90.499, -83.487);
     JointPos j4(107.935, -83.572, 95.424, -92.873, -87.933, -83.488);

     DescPose descSafe = {};
     DescPose desc1 = {};
     DescPose desc2 = {};
     DescPose desc3 = {};
     DescPose desc4 = {};
     ExaxisPos exaxisPos = { 0, 0, 0, 0 };
     DescPose offdese = { 0, 0, 0, 0, 0, 0 };

     robot.GetForwardKin(&jSafe, &descSafe);
     robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.Sleep(2000);

     robot.GetForwardKin(&j1, &desc1);
     robot.MoveJ(&j1, &desc1, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.Sleep(2000);

     DescPose actualTCPPos = {};
     robot.GetActualTCPPose(0, &actualTCPPos);
     robot.SetRefPointInExAxisEnd(actualTCPPos);
     rtn = robot.PositionorSetRefPoint(1);
     cout << "PositionorSetRefPoint 1 rtn is " << rtn << endl;
     robot.Sleep(2000);

     robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.ExtAxisStartJog(1, 0, 50, 50, 10);
     robot.Sleep(1000);
     robot.ExtAxisStartJog(2, 0, 50, 50, 10);
     robot.Sleep(1000);
     robot.GetForwardKin(&j2, &desc2);
     rtn = robot.MoveJ(&j2, &desc2, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot.PositionorSetRefPoint(2);
     cout << "PositionorSetRefPoint 2 rtn is " << rtn << endl;
     robot.Sleep(2000);

     robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.ExtAxisStartJog(1, 0, 50, 50, 10);
     robot.Sleep(1000);
     robot.ExtAxisStartJog(2, 0, 50, 50, 10);
     robot.Sleep(1000);
     robot.GetForwardKin(&j3, &desc3);
     robot.MoveJ(&j3, &desc3, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot.PositionorSetRefPoint(3);
     cout << "PositionorSetRefPoint 3 rtn is " << rtn << endl;
     robot.Sleep(2000);

     robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.ExtAxisStartJog(1, 0, 50, 50, 10);
     robot.Sleep(1000);
     robot.ExtAxisStartJog(2, 0, 50, 50, 10);
     robot.Sleep(1000);
     robot.GetForwardKin(&j4, &desc4);
     robot.MoveJ(&j4, &desc4, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot.PositionorSetRefPoint(4);
     cout << "PositionorSetRefPoint 4 rtn is " << rtn << endl;
     robot.Sleep(2000);

     DescPose axisCoord = {};
     robot.PositionorComputeECoordSys(axisCoord);
     robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     printf("PositionorComputeECoordSys rtn is %f %f %f %f %f %f\n", axisCoord.tran.x, axisCoord.tran.y, axisCoord.tran.z, axisCoord.rpy.rx, axisCoord.rpy.ry, axisCoord.rpy.rz);
     rtn = robot.ExtAxisActiveECoordSys(3, 1, axisCoord, 1);
     cout << "ExtAxisActiveECoordSys rtn is " << rtn << endl;

     DescPose getCoord(0, 0, 0, 0, 0, 0);
     robot.ExtAxisGetCoord(getCoord);
     printf("PositionorComputeECoordSys rtn is %f %f %f %f %f %f\n", getCoord.tran.x, getCoord.tran.y, getCoord.tran.z, getCoord.rpy.rx, getCoord.rpy.ry, getCoord.rpy.rz);
     robot.CloseRPC();
     return 0;
 }

 int TestAuxDOAO(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     for (int i = 0; i < 128; i++)
     {
         robot.SetAuxDO(i, true, false, true);
         robot.Sleep(100);
     }
     for (int i = 0; i < 128; i++)
     {
         robot.SetAuxDO(i, false, false, true);
         robot.Sleep(100);
     }

     for (int i = 0; i < 409; i++)
     {
         robot.SetAuxAO(0, i * 10, true);
         robot.SetAuxAO(1, 4095 - i * 10, true);
         robot.SetAuxAO(2, i * 10, true);
         robot.SetAuxAO(3, 4095 - i * 10, true);
         robot.Sleep(10);
     }

     robot.SetAuxDIFilterTime(10);
     robot.SetAuxAIFilterTime(0, 10);

     for (int i = 0; i < 20; i++)
     {
         bool curValue = false;
         int rtn = robot.GetAuxDI(i, false, curValue);
         cout << "DI" << i << "   " << curValue << endl;
     }
     int curValue = -1;
     for (int i = 0; i < 4; i++)
     {
         rtn = robot.GetAuxAI(i, true, curValue);
     }

     robot.WaitAuxDI(1, false, 1000, false);
     robot.WaitAuxAI(1, 1, 132, 1000, false);

     robot.CloseRPC();
     return 0;
 }

 int TestTractor(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10, 1);
     robot.ExtDevLoadUDPDriver();

     rtn = robot.ExtAxisServoOn(1, 1);
     rtn = robot.ExtAxisServoOn(2, 1);
     robot.Sleep(2000);

     robot.ExtAxisSetHoming(1, 0, 10, 2);
     robot.Sleep(2000);
     rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);

     robot.Sleep(4000);

     robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
     robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
     robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0);

     robot.TractorEnable(false);
     robot.Sleep(2000);
     robot.TractorEnable(true);
     robot.Sleep(2000);
     robot.TractorHoming();
     robot.Sleep(2000);
     robot.TractorMoveL(100, 2);
     robot.Sleep(5000);
     robot.TractorStop();
     robot.TractorMoveL(-100, 20);
     robot.Sleep(5000);
     robot.TractorMoveC(300, 90, 20);
     robot.Sleep(10000);
     robot.TractorMoveC(300, -90, 20);
     robot.Sleep(1);

     robot.CloseRPC();
     return 0;
 }

 int TestFIR(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos midjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
     JointPos endjointPos(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);

     DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose middescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
     DescPose enddescPose(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     rtn = robot.PtpFIRPlanningStart(1000, 1000);
     cout << "PtpFIRPlanningStart rtn is " << rtn << endl;
     robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.PtpFIRPlanningEnd();
     cout << "PtpFIRPlanningEnd rtn is " << rtn << endl;

     robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
     cout << "LinArcFIRPlanningStart rtn is " << rtn << endl;
     robot.MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
     robot.MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
     robot.LinArcFIRPlanningEnd();
     cout << "LinArcFIRPlanningEnd rtn is " << rtn << endl;
     robot.CloseRPC();
     return 0;
 }

 int TestAccSmooth(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

     DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);
     rtn = robot.AccSmoothStart(0);
     cout << "AccSmoothStart rtn is " << rtn << endl;
     robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot.AccSmoothEnd(0);
     cout << "AccSmoothEnd rtn is " << rtn << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestAngularSpeed(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

     DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);
     rtn = robot.AngularSpeedStart(50);
     cout << "AngularSpeedStart rtn is " << rtn << endl;
     robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot.AngularSpeedEnd();
     cout << "AngularSpeedEnd rtn is " << rtn << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestSingularAvoid(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
     JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

     DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
     DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     rtn = robot.SingularAvoidStart(2, 10, 5, 5);
     cout << "SingularAvoidStart rtn is " << rtn << endl;
     robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     rtn = robot.SingularAvoidEnd();
     cout << "SingularAvoidEnd rtn is " << rtn << endl;

     robot.CloseRPC();
     return 0;
 }

 int TestLoadTrajLA(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     rtn = robot.TrajectoryJUpLoad("D://zUP/traj.txt");
     printf("Upload TrajectoryJ A %d\n", rtn);

     char traj_file_name[30] = "/fruser/traj/traj.txt";
     rtn = robot.LoadTrajectoryLA(traj_file_name, 1, 2, 0, 2, 100, 200, 1000);
     printf("LoadTrajectoryLA %s, rtn is: %d\n", traj_file_name, rtn);

     DescPose traj_start_pose;
     memset(&traj_start_pose, 0, sizeof(DescPose));
     rtn = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
     printf("GetTrajectoryStartPose is: %d\n", rtn);
     printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);

     std::this_thread::sleep_for(std::chrono::seconds(1));

     robot.SetSpeed(50);
     robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);

     rtn = robot.MoveTrajectoryLA();
     printf("MoveTrajectoryLA rtn is: %d\n", rtn);

     robot.CloseRPC();
     return 0;
 }

 int TestIdentify(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     int retval = 0;

     retval = robot.LoadIdentifyDynFilterInit();
     printf("LoadIdentifyDynFilterInit retval is: %d \n", retval);

     retval = robot.LoadIdentifyDynVarInit();
     printf("LoadIdentifyDynVarInit retval is: %d \n", retval);

     JointPos posJ = {};
     DescPose posDec = {};
     float joint_toq[6] = { 0.0 };
     robot.GetActualJointPosDegree(0, &posJ);
     posJ.jPos[1] = posJ.jPos[1] + 10;
     robot.GetJointTorques(0, joint_toq);
     joint_toq[1] = joint_toq[1] + 2;

     double tmpTorque[6] = { 0.0 };
     for (int i = 0; i < 6; i++)
     {
         tmpTorque[i] = joint_toq[i];
     }

     retval = robot.LoadIdentifyMain(tmpTorque, posJ.jPos, 1);
     printf("LoadIdentifyMain retval is: %d \n", retval);

     double gain[12] = { 0,0.05,0,0,0,0,0,0.02,0,0,0,0 };
     double weight = 0;
     DescTran load_pos;
     memset(&load_pos, 0, sizeof(DescTran));
     retval = robot.LoadIdentifyGetResult(gain, &weight, &load_pos);
     printf("LoadIdentifyGetResult retval is: %d ; weight is %f  cog is %f %f %f \n", retval, weight, load_pos.x, load_pos.y, load_pos.z);

     robot.CloseRPC();
     return 0;
 }

 int TestWideVoltageCtrlBoxtemp(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     printf("robot rpc rtn is %d\n", rtn);
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);

     robot.SetWideBoxTempFanMonitorParam(1, 2);
     int enable = 0;
     int period = 0;
     robot.GetWideBoxTempFanMonitorParam(enable, period);
     printf("GetWideBoxTempFanMonitorParam enable is %d   period is %d\n", enable, period);
     for (int i = 0; i < 100; i++)
     {
         robot.GetRobotRealTimeState(&pkg);
         printf("robot ctrl box temp is %f,  fan current is %d\n", pkg.wideVoltageCtrlBoxTemp, pkg.wideVoltageCtrlBoxFanCurrent);
         robot.Sleep(100);
     }

     rtn = robot.SetWideBoxTempFanMonitorParam(0, 2);
     printf("SetWideBoxTempFanMonitorParam rtn is %d\n", rtn);
     enable = 0;
     period = 0;
     robot.GetWideBoxTempFanMonitorParam(enable, period);
     printf("GetWideBoxTempFanMonitorParam enable is %d   period is %d\n", enable, period);
     for (int i = 0; i < 100; i++)
     {
         robot.GetRobotRealTimeState(&pkg);
         printf("robot ctrl box temp is %f,  fan current is %d\n", pkg.wideVoltageCtrlBoxTemp, pkg.wideVoltageCtrlBoxFanCurrent);
         robot.Sleep(100);
     }

     robot.CloseRPC();
     robot.Sleep(2000);

     return 0;
 }

 int terty5(FRRobot * robot)
 {
     int rtn = robot->LuaUpload("D://zUP/Program5.lua");
     printf("LuaUpload rtn is %d\n", rtn);
     robot->ProgramLoad("/fruser/Program5.lua");
     
     SOCKET serSocket = socket(AF_INET, SOCK_STREAM, 0);//创建了可识别套接字
     if (serSocket != -1)
     {
         printf("成功创建套接字！%d\n", serSocket);
     }

     //需要绑定的参数，主要是本地的socket的一些信息。
     SOCKADDR_IN addr;
     addr.sin_family = AF_INET;
     addr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);//ip地址
     addr.sin_port = htons(31000);//绑定端口

     bind(serSocket, (SOCKADDR*)&addr, sizeof(SOCKADDR));//绑定完成
     listen(serSocket, 5);//其中第二个参数代表能够接收的最多的连接数
     printf("等待客户端...\n");
     SOCKADDR_IN clientsocket;
     int len = sizeof(SOCKADDR);

     robot->ProgramRun();
     SOCKET serConn = accept(serSocket, (SOCKADDR*)&clientsocket, &len);
     //如果这里不是accept而是conection的话。。就会不断的监听
     if (serConn)
     {
         printf("监听到新的客户端...\n");
     }
     while (1)
     {
         robot->Sleep(200);
         char sendBuf[4000] = "";
         sprintf(sendBuf, "welcome %s to here", inet_ntoa(clientsocket.sin_addr));//找对对应的IP并且将这行字打印到那里
         //发送信息
         send(serConn, sendBuf, strlen(sendBuf) + 1, 0);
         char receiveBuf[4000] = "";//接收
         memset(receiveBuf, 0, 4000);
         int RecvLen;
         RecvLen = recv(serConn, receiveBuf, 4000, 0);
         printf("5 recv string is  %s\n", receiveBuf);
         if (RecvLen <= 0)
             printf("5 recv length %d  err %s\n", RecvLen, WSAGetLastError());
         else
         {
             string txt = receiveBuf;
             string::size_type idx = txt.find("ProgramFinish"); //在a中查找b.
             if (idx == string::npos) //不存在。
                 cout << "not found\n";
             else
             {
                 cout << "found ProgramFinish\n";
                 break;
             }
         }

     }
     closesocket(serConn);//关闭
     //WSACleanup();//释放资源的操作

     rtn = robot->LuaDelete("Program5.lua");
     printf("LuaDelete rtn is %d\n", rtn);
     return 0;
 }

 int terty6(FRRobot* robot)
 {
     int rtn = robot->LuaUpload("D://zUP/Program6.lua");
     printf("LuaUpload rtn is %d\n", rtn);
     robot->ProgramLoad("/fruser/Program6.lua");

     SOCKET serSocket = socket(AF_INET, SOCK_STREAM, 0);//创建了可识别套接字
     if (serSocket != -1)
     {
         printf("成功创建套接字！%d\n", serSocket);
     }

     //需要绑定的参数，主要是本地的socket的一些信息。
     SOCKADDR_IN addr;
     addr.sin_family = AF_INET;
     addr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);//ip地址
     addr.sin_port = htons(31000);//绑定端口

     bind(serSocket, (SOCKADDR*)&addr, sizeof(SOCKADDR));
     listen(serSocket, 5);
     printf("等待客户端...\n");
     SOCKADDR_IN clientsocket;
     int len = sizeof(SOCKADDR);

     robot->ProgramRun();
     SOCKET serConn = accept(serSocket, (SOCKADDR*)&clientsocket, &len);
     if (serConn)
     {
         printf("监听到新的客户端...\n");
     }
     while (1)
     {
         robot->Sleep(100);
         char sendBuf[100];
         sprintf(sendBuf, "welcome %s to here", inet_ntoa(clientsocket.sin_addr));//找对对应的IP并且将这行字打印到那里
         send(serConn, sendBuf, strlen(sendBuf) + 1, 0);
         char receiveBuf[4000] = "";//接收
         memset(receiveBuf, 0, 4000);
         int RecvLen;
         RecvLen = recv(serConn, receiveBuf, 4000, 0);
         printf("6 recv string is  %s\n", receiveBuf);
         if (RecvLen <= 0)
             printf("6 recv length %d   %d\n", RecvLen, WSAGetLastError());
         else
         {
             string txt = receiveBuf;
             string::size_type idx = txt.find("ProgramFinish"); //在a中查找b.
             if (idx == string::npos) //不存在。
                 cout << "not found\n";
             else
             {
                 cout << "found ProgramFinish\n";
                 break;
             }
         }

     }
     closesocket(serConn);//关闭
    // WSACleanup();//释放资源的操作

     rtn = robot->LuaDelete("Program6.lua");
     printf("LuaDelete rtn is %d\n", rtn);
     return 0;
 }

 int main(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;
     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }

     while (true)
     {
         terty5(&robot);
         robot.Sleep(4000);
         terty6(&robot);
         robot.Sleep(4000);
     }

     robot.CloseRPC();
     return 0;
 }


 

 int testSyncMoveJ()
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;
     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     if (rtn != 0)
     {
         return -1;
     }
     robot.SetReConnectParam(true, 30000, 500);
     robot.SetForwardWireFeed(0, 1);
     robot.Sleep(1000);
     robot.SetForwardWireFeed(0, 0);
     robot.SetReverseWireFeed(0, 1);
     robot.Sleep(1000);
     robot.SetReverseWireFeed(0, 0);
     robot.SetAspirated(0, 1);
     robot.Sleep(1000);
     robot.SetAspirated(0, 0);
     robot.WeldingStartReWeldAfterBreakOff();
     robot.WeldingAbortWeldAfterBreakOff();
 }

 