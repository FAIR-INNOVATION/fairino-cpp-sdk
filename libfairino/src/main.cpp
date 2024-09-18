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

    robot->SetAuxDO(1, true, false, true);
    robot->SetAuxDO(2, true, false, true);

    robot->SetAuxAO(0, 1024, false);
    robot->SetAuxAO(1, 2048, false);

    //robot->SetDO(1, 1, 0, 0);
    //robot->SetDO(3, 1, 0, 0);
    //robot->SetAO(0, 50, 0);
    //robot->SetAO(1, 70, 0);
    //robot->SetToolDO(1, 1, 0, 0);
    //robot->SetToolAO(0, 40, 0);
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

int ArcTrack(FRRobot* robot)
{
    DescPose startdescPose = { -583.168, 325.637, 1.176, 75.262, 0.978, -3.571 };
    JointPos startjointPos = { -49.049, -77.203, 136.826, -189.074, -79.407, -11.811 };
    
    DescPose enddescPose = { -559.439, 420.491, 32.252, 77.745, 1.460, -10.130 };
    JointPos endjointPos = { -54.986, -77.639, 131.865, -185.707, -80.916, -12.218 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->WeldingSetCurrent(1, 230, 0, 0);
    robot->WeldingSetVoltage(1, 24, 0, 1);

    //robot->GetForwardKin(&startjointPos, &startdescPose);
    //robot->GetForwardKin(&endjointPos, &enddescPose);
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

int main(void)
{
    ROBOT_STATE_PKG pkg = {};
	FRRobot robot;
    //robot.SetReConnectParam(true, 30000, 500);
	robot.LoggerInit();
	robot.SetLoggerLevel(1);
	int rtn = robot.RPC("192.168.58.2");

    //TestWeave(&robot);
    TestWelding(&robot);

    //WeldingProcessParamConfig(&robot);
    //SetExtDIOFuntion(&robot);
    //IOReset(&robot);
    //StaticCollision(&robot);
    //DragControl(&robot);
    //SixDiaDrag(&robot);
    //RobotGetFTDragState(&robot);
    //FTAutoOn(&robot);
    //FTAutoOff(&robot);
    //FTLoadSetGet(&robot);
    //FTAutoComputeLoad(&robot);
    //PowerLimitOn(&robot);
    //PowerLimitOff(&robot);
    //WeaveSim(&robot);
    //WeaveInspect(&robot);
    //ArcTrack(&robot);
    //SegmentWeld(&robot);
    //Wiresearch(&robot);

    AxleSensorConfig(&robot);




	robot.CloseRPC();
	return 0;
}

