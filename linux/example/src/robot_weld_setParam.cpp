#include "libfairino/robot.h"
#ifdef WINDOWS_OPTION
#include <string.h>
#include <windows.h>
#elif LINUX_OPTION
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#endif

#include <chrono>
#include <thread>

using namespace std;

int main(void)
{
    printf("start to debug\n");
    FRRobot robot;            
    robot.RPC("192.168.58.2"); 

    double current_min = 0;
    double current_max = 0;
    double vol_min = 0;
    double vol_max = 0;
    double output_vmin = 0;
    double output_vmax = 0;
    int retval = 0;

    retval = robot.WeldingSetCurrentRelation(0, 400, 0, 10);
    cout << "WeldingSetCurrentRelation retval is: " << retval << endl;

    retval = robot.WeldingSetVoltageRelation(0, 40, 0, 10);
    cout << "WeldingSetVoltageRelation retval is: " << retval << endl;

    retval = robot.WeldingGetCurrentRelation(&current_min, &current_max, &output_vmin, &output_vmax);
    cout << "WeldingGetCurrentRelation retval is: " << retval << endl;
    cout << "current min " << current_min << " current max " << current_max << " output vol min " << output_vmin << " output vol max "<< output_vmax<<endl;

    retval = robot.WeldingGetVoltageRelation(&vol_min, &vol_max, &output_vmin, &output_vmax);
    cout << "WeldingGetVoltageRelation retval is: " << retval << endl;
    cout << "vol min " << vol_min << " vol max " << vol_max << " output vol min " << output_vmin << " output vol max "<< output_vmax<<endl;

    retval = robot.WeldingSetCurrent(1, 100, 0);
    cout << "WeldingSetCurrent retval is: " << retval << endl;

    this_thread::sleep_for(chrono::seconds(3));

    retval = robot.WeldingSetVoltage(1, 10, 0);
    cout << "WeldingSetVoltage retval is: " << retval << endl;
    
    return 0;
}