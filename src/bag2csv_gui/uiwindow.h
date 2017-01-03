#ifndef UIWINDOW_H
#define UIWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QTimer>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "unistd.h"
#include "string.h"

//ROS头文件
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Vector3.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mavros_msgs/OpticalFlowRad.h>

#include <time.h>
#include <math.h>
#include <signal.h>
#include <QProcess>
#include <qimage.h>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>

#include "StereoImage.h"
#include "StereoCompressedImage.h"
#include "recordMsg.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace mavros_msgs;

//全局变量
#define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
#define IsTest true

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void showRecordList();
    ~MainWindow();
    void setNodeHandle(ros::NodeHandle nh);
    void startReadingLoop();
    void reading_loop();

    Ui::MainWindow *ui;


private slots:
    void on_openButton_clicked();

    void on_startButton_clicked();

    void on_saveButton_clicked();

    void showInfo();

signals:
    void beginROS();


private:
    ros::NodeHandle n;

    pthread_t read_tid;
    QProcess *pocBagPlay;
    QProcess *pocEndPlay;
    //QTimer *rosTimer;
    string bagName;
    string savePath;
    bool isBagName;
    bool isSavePath;
    bool record_sensor_imu;
    bool record_sensor_stereo;
    bool record_sensor_rgbd;
    bool record_sensor_lidar;
    bool record_pixhawk_imu;
    bool record_pixhawk_px4flow;
    bool record_vicon;
    bool record_rtk;
    bool isCameraInfo;
    bool isLidarInfo;
    bool closeFile;
    bool no_loop;
    double playRate;

    ros::Publisher recordPub;
    ros::Subscriber infoSub;

    string info;
    QTimer *infoTimer;

    void infoCb(const std_msgs::StringConstPtr& msg);


};

#endif // UIWINDOW_H
