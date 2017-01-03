#ifndef MYPROCESSHANDLER_H
#define MYPROCESSHANDLER_H

#include <QProcess>
//ROS头文件
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Bool.h"
#include "recordMsg.h"

class myProcessHandler
{
public:
    myProcessHandler(ros::NodeHandle *nh);
    ~myProcessHandler();
    ros::NodeHandle n;
    bag2csv::recordMsg commMsg;
    pthread_t read_tid;
    bool newComm;
    bool toRefresh;

    ros::Subscriber record_sub;
    void reading_loop();
    void startReadingLoop();

private:
    void recordCb(const bag2csv::recordMsgConstPtr& msg);


};

#endif // MYPROCESSHANDLER_H
