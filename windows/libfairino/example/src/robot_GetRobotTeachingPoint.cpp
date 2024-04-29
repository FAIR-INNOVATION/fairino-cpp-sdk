#include "libfairino/robot.h"
#include <string.h>

int main(void)
{
	FRRobot robot;			   // 实例化机器人对象
	robot.RPC("192.168.58.2"); // 与机器人控制器建立通信连接

    char name[64] = "F1";
    float data[20] = {0};

    int ret = robot.GetRobotTeachingPoint(name, data);

    printf(" %d name is: %s \n", ret, name);
    for (int i = 0; i < 20; i++)
    {
            printf("data is: %f \n", data[i]);
    }

	return 0;
}