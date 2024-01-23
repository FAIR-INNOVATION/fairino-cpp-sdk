#ifndef   ROBOTERROR_H_
#define   ROBOTERROR_H_

#define     ERR_UPLOAD_FILE_NOT_FOUND              -7       //The file to be uploaded does not exist
#define     ERR_SAVE_FILE_PATH_NOT_FOUND           -6       //The file path does not exist
#define     ERR_LUA_FILE_NOT_FOUND                 -5       //The lua file does not exist
#define     ERR_XMLRPC_CMD_FAILED                  -4      //xmlrpc interface execution failed, please contact after-sales engineer
#define     ERR_XMLRPC_COM_FAILED                  -3      //xmlrpc communication failed, please check whether the network connection and server IP address are correct
#define     ERR_SOCKET_COM_FAILED                  -2      //Network communication abnormality
#define     ERR_OTHER                              -1      //other errors
#define     ERR_SUCCESS                            0       //success
#define     ERR_PARAM_NUM                          3       //The number of parameters is inconsistent
#define     ERR_PARAM_VALUE                        4       //The parameter value is not within a reasonable range
#define     ERR_TPD_FILE_OPEN_FAILED               8       //The file failed to open
#define     ERR_EXECUTION_FAILED                   14      //The command failed to be executed
#define     ERR_PROGRAM_IS_RUNNING                 18      //The program is running
#define     ERR_COMPUTE_FAILED                     25      //The calculation failed
#define     ERR_INVERSE_KINEMATICS_COMPUTE_FAILED  28      //The inverse kinematics calculation failed
#define     ERR_SERVOJ_JOINT_OVERRUN               29      //The joint value is exceeded
#define     ERR_NON_RESSETTABLE_FAULT              30      //The fault cannot be reset, please power off and restart the control box
#define     ERR_WORKPIECE_NUM                      34      //The workpiece number is incorrect
#define     ERR_FILENAME_TOO_LONG                  36      //The file name is too long
#define     ERR_STRANGE_POSE                       38      //Singular pose
#define     ERR_NOT_ADD_CMD_QUEUE                  64      //Not queued for instructions
#define     ERR_CIRCLE_SPIRAL_MIDDLE_POINT1        66      //The middle point of the circle is 1 incorrect
#define     ERR_CIRCLE_SPIRAL_MIDDLE_POINT2        67      //The middle point of the circle is 2 incorrect
#define     ERR_CIRCLE_SPIRAL_MIDDLE_POINT3        68      //The middle point of the circle is 3 incorrect
#define     ERR_MOVEC_MIDDLE_POINT                 69      //The middle point of the arc is incorrect
#define     ERR_MOVEC_TARGET_POINT                 70      //The target point of the arc is incorrect
#define     ERR_GRIPPER_MOTION                     73      //The jaw movement is wrong
#define     ERR_LINE_POINT                         74      //The straight line target point is incorrect
#define     ERR_CHANNEL_FAULT                      75      //Channel error
#define     ERR_WAIT_TIMEOUT                       76      //The wait timed out
#define     ERR_TPD_CMD_POINT                      82      //The TPD command point is incorrect
#define     ERR_TPD_CMD_TOOL                       83      //The TPD tool number is incorrect
#define     ERR_SPLINE_POINT                       94      //The spline command point is incorrect
#define     ERR_SPIRAL_START_POINT                 108     //The helix start point is incorrect
#define     ERR_TARGET_POSE_CANNOT_REACHED         112     //The target pose cannot be reached
#define     ERR_POINTTABLE_NOTFOUND                130     //The point table does not exist

#endif
