#include "robot.h"
#ifdef WIN32
#include <string.h>
#include <windows.h>
#else
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#endif

#include <chrono>
#include <thread>
#include "md5.hpp"
#include "FRTcpClient.h"

using namespace std;

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
    robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);

    robot->Sleep(5000);

    robot->EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);
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
    robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);

    robot->Sleep(1000);
    robot->EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);
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
    robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);
    return 0;
}

int FTAutoOff(FRRobot* robot)
{
    robot->SetForceSensorDragAutoFlag(0);
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);
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

    robot->MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->WeaveStartSim(0);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
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

    robot->MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->WeaveInspectStart(0);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
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
    robot->MoveL(&endjointPos, &enddescPose, 13, 0, 5, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
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

    robot->MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese );
    robot->MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);

    DescPose descREF0A = { 147.139, -21.436, 60.717, -179.633, -3.051, -83.170 };
    JointPos jointREF0A = { -121.731, -106.193, -102.561, -64.734, 89.972, 96.171 };

    DescPose descREF0B = { 139.247, 43.721, 65.361, -179.634, -3.043, -83.170 };
    JointPos jointREF0B = { -122.364, -113.991, -90.860, -68.630, 89.933, 95.540 };

    DescPose descREF1A = { 289.747, 77.395, 58.390, -179.074, -2.901, -89.790 };
    JointPos jointREF1A = { -135.719, -119.588, -83.454, -70.245, 88.921, 88.819 };

    DescPose descREF1B = { 259.310, 79.998, 64.774, -179.073, -2.900, -89.790 };
    JointPos jointREF1B = { -133.133, -119.029, -83.326, -70.976, 89.069, 91.401 };

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("REF0");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("REF1");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("RES0");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("RES1");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
    vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
    int offectFlag = 0;
    DescPose offectPos = {0, 0, 0, 0, 0, 0};
    rtn0 = robot->GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
    robot->PointsOffsetEnable(0, &offectPos);
    robot->MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);
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

 void TestServoJ(FRRobot* robot)
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
     robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);

     robot->Sleep(10 * 1000);

     robot->EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);
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
     robot->ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10);
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

     robot->MoveL(&jointStart, &descStart, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&jointEnd, &descEnd, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     
     DescPose descREF0A = { -66.106, -560.746, 270.381, 176.479, -0.126, -126.745 };
     JointPos jointREF0A = { 73.531, -75.588, 102.941, -116.250, -93.347, -69.689 };
     
     DescPose descREF0B = { -66.109, -528.440, 270.407, 176.479, -0.129, -126.744 };
     JointPos jointREF0B = { 72.534, -79.625, 108.046, -117.379, -93.366, -70.687 };
     
     DescPose descREF1A = { 72.975, -473.242, 270.399, 176.479, -0.129, -126.744 };
     JointPos jointREF1A = { 87.169, -86.509, 115.710, -117.341, -92.993, -56.034 };
     
     DescPose descREF1B = { 31.355, -473.238, 270.405, 176.480, -0.130, -126.745 };
     JointPos jointREF1B = { 82.117, -87.146, 116.470, -117.737, -93.145, -61.090 };

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF0A, &descREF0A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF0B, &descREF0B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
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

     int retval = 0;
     char traj_file_name[30] = "/fruser/traj/testA.txt";
     retval = robot->LoadTrajectoryJ(traj_file_name, 100, 1);
     printf("LoadTrajectoryJ %s, retval is: %d\n", traj_file_name, retval);

     DescPose traj_start_pose;
     memset(&traj_start_pose, 0, sizeof(DescPose));
     retval = robot->GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
     printf("GetTrajectoryStartPose is: %d\n", retval);
     printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);

     robot->SetSpeed(20);
     robot->MoveCart(&traj_start_pose, 1, 0, 100, 100, 100, -1, -1);

     robot->Sleep(5000);

     int traj_num = 0;
     retval = robot->GetTrajectoryPointNum(&traj_num);
     printf("GetTrajectoryStartPose retval is: %d, traj num is: %d\n", retval, traj_num);

     retval = robot->MoveTrajectoryJ();
     printf("MoveTrajectoryJ retval is: %d\n", retval);
 }

 int UploadTrajectoryB(FRRobot* robot)
 {
     robot->TrajectoryJDelete("testB.txt");
     robot->TrajectoryJUpLoad("D://zUP/testB.txt");

     int retval = 0;
     char traj_file_name[30] = "/fruser/traj/testB.txt";
     retval = robot->LoadTrajectoryJ(traj_file_name, 100, 1);
     printf("LoadTrajectoryJ %s, retval is: %d\n", traj_file_name, retval);

     DescPose traj_start_pose;
     memset(&traj_start_pose, 0, sizeof(DescPose));
     retval = robot->GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
     printf("GetTrajectoryStartPose is: %d\n", retval);
     printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);

     robot->SetSpeed(20);
     robot->MoveCart(&traj_start_pose, 1, 0, 100, 100, 100, -1, -1);

     robot->Sleep(5000);

     int traj_num = 0;
     retval = robot->GetTrajectoryPointNum(&traj_num);
     printf("GetTrajectoryStartPose retval is: %d, traj num is: %d\n", retval, traj_num);

     retval = robot->MoveTrajectoryJ();
     printf("MoveTrajectoryJ retval is: %d\n", retval);
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
         if (abs(pkg.cl_analog_output[0] - value) < 0.5)
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
     DescPose startdescPose(-118.215, -418.107, 558.397, 176.412, 3.477, 49.873);
     JointPos startjointPos(59.942, -100.088, 88.346, -82.308, -87.075, 0.063);

     DescPose enddescPose(-229.648, -488.934, 363.843, 173.340, 3.954, 42.816);
     JointPos endjointPos(52.850, -84.328, 102.162, -112.890, -84.130, 0.063);

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
         robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         robot->MoveJ(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
     }
 }

 void TRSF(FRRobot* robot, bool enable)
 {
     int rtn = 0;
     DescPose startdescPose(-226.699, -501.969, 264.638, -174.973, 5.852, 143.301);
     JointPos startjointPos(52.850, -84.327, 102.163, -112.843, -84.131, 0.063);

     DescPose enddescPose(-226.702, -501.973, 155.833, -174.973, 5.852, 143.301);
     JointPos endjointPos(52.850, -77.596, 111.785, -129.196, -84.131, 0.062);

     ExaxisPos exaxisPos(0, 0, 0, 0);
     DescPose offdese(0, 0, 0, 0, 0, 0);

     if (enable)
     {
         robot->ToolTrsfStart(1);
         robot->MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         robot->MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         robot->ToolTrsfEnd();
     }
     else
     {
         rtn = robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         printf("rtn is %d\n", rtn);
         rtn = robot->MoveJ(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
         printf("rtn is %d\n", rtn);
     }
 }

 int main(void)
 {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     robot.SetReConnectParam(true, 30000, 500);

     /*robot.ResetAllError();
     robot.RobotEnable(1);*/
    
     robot.ProgramLoad("/fruser/ccc.lua");
     robot.ProgramRun();
     bool slag = false;
     while (true)
     {
         robot.GetRobotRealTimeState(&pkg);
         printf("main err code %d  sub err code %d\n", pkg.main_code, pkg.sub_code);
         robot.Sleep(10);
         if (pkg.rbtEnableState == 0 && !slag)
         {
             rtn = robot.ResetAllError();
             printf("ResetAllError rtn is %d\n", rtn);
             if (rtn != 0)
             {
                 robot.ResetAllError();
             }
             robot.RobotEnable(1);
             slag = true;
         }
     }



 //    while (true )
 //    {
 //        //DescPose posCprd(0, 0, 0, 0, 0, 100);
 //        //robot.SetToolCoord(1, &posCprd, 0, 0, 1, 0);
 //        JointPos pos = {};
 //        robot.GetActualJointPosDegree(1, &pos);
 //        printf("pos is %lf %lf\n", pos.jPos[0], pos.jPos[1]);
 //        robot.Sleep(100);

 //    }
 //    //FIRLinL(&robot, false);
 //    //FIRLinL(&robot, true);

 //    //FIRArc(&robot, false);
 //    //FIRArc(&robot, true);
 //    int i = 0;
 ////    while (true)
 ////    {
 ////        i++;
 ////        int rtn = robot.LuaDelete("wand888.lua");
 ////        printf("LuaDelete rtn is %d\n", i);
 ////        rtn = robot.LuaUpload("D://zUP/wand888.lua");
 //        printf("LuaUpload rtn is %d\n", rtn);
 //        robot.Sleep(300);
 ///*        if (rtn != 0)
 //        {
 //            break;
 //        }*/
 //    }

     //TrajectoryJUpload(&robot);
     //TrajectoryJDelete(&robot);

     //TestSingularAvoidSArc(&robot);
     //TestSingularAvoidEArc(&robot);
     //TestSingularAvoidSLin(&robot);
     //TestSingularAvoidWArc(&robot);
     //TestSingularAvoidWLin(&robot);

     //while (true)
     //{
     //    MoveRotGripper(&robot, 30, 0);
     //    MoveRotGripper(&robot, 90, 0);
     //    UploadTrajectoryJ(&robot);
     //    robot.Sleep(5000);
     //    MoveRotGripper(&robot, 30, 0);
     //    robot.Sleep(1000);
     //    MoveRotGripper(&robot, 90, 0);
     //    UploadTrajectoryB(&robot);
     //    robot.Sleep(5000);
     //    MoveRotGripper(&robot, 30, 0);
     //    int rtn = robot.GetRobotRealTimeState(&pkg);
     //    printf("the robot AO0 %d, AO1 %d, tool AO0  %d\n", pkg.cl_analog_output[0], pkg.cl_analog_output[1], pkg.tl_analog_output);
     //    printf("gripper pos %d - vel %d - torque %d - rotPos %f - rotvel - %d rotTor - %d\n", pkg.gripper_position, pkg.gripper_speed, pkg.gripper_current, pkg.gripperRotNum, pkg.gripperRotSpeed, pkg.gripperRotTorque);
     //}

     //while (true)
     //{
     //    rtn = robot.GetRobotRealTimeState(&pkg);
     //    //printf("the robot AO0 %d, AO1 %d, tool AO0  %d\n", pkg.cl_analog_output[0], pkg.cl_analog_output[1], pkg.tl_analog_output);
     //    //printf("gripper pos %d - vel %d - torque %d - rotPos %f - rotvel - %d rotTor - %d\n", pkg.gripper_position, pkg.gripper_speed, pkg.gripper_current, pkg.gripperRotNum, pkg.gripperRotSpeed, pkg.gripperRotTorque);
     //    printf("main error code %d;  sub error code %d\n", pkg.safety_stop0_state, pkg.safety_stop1_state);
     //    robot.Sleep(3);
     //}
     robot.CloseRPC();
     return 0;
}

