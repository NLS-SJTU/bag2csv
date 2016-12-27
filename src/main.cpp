/*
 * main.cpp
 *
 *  Created on: Nov. 24, 2016
 *      Author: ZhouQiang
 */

#include <QApplication>

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

// 使用Eigen的Geometry模块处理3d运动
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "StereoImage.h"
#include "StereoCompressedImage.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace mavros_msgs;

//全局变量
#define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
#define IsTest true

static int img_no = 0;
static int depth_no = 0;
static bool record_sensor_imu = false;
static bool record_sensor_stereo = false;
static bool record_sensor_rgbd = false;
static bool record_sensor_lidar = false;
static bool record_pixhawk_imu = false;
static bool record_pixhawk_px4flow = false;
static bool record_vicon = false;
static bool record_rtk = false;
static bool isCameraInfo = false;
static bool isLidarInfo = false;

//函数声明
static void viconCb(const geometry_msgs::TransformStampedPtr& msg);
static void imgCompressedCb(const drone_sensor::StereoCompressedImagePtr& msg);
void rgbdCb(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info);
void imuCb(const FluidPressureConstPtr& pressure, const MagneticFieldConstPtr& mag, const ImuConstPtr& imu_raw);
void imuPoseCb(const ImuConstPtr& imu);
void px4flowCb(const sensor_msgs::RangeConstPtr& ground, const OpticalFlowRadConstPtr &optical);
void lidarCb(const LaserScanConstPtr& scan);
void showParam();

ofstream sensorIMUfile;
ofstream stereoStampfile;
ofstream rgbdStampfile;
ofstream rgbdCameraInfofile;
ofstream Lidarfile;
ofstream LidarInfofile;
ofstream pixhawkImufile;
ofstream px4flowfile;
ofstream viconfile;
ofstream rtkfile;
string filePath;


int main(int argc, char** argv) {

    if(argc!=2){
        cerr<<"please input yaml file "<<argc<<endl;
        return 1;
    }
    ros::init(argc, argv,"bag2csv");
    ros::NodeHandle n;

    //开始读取参数
    FileStorage fs;
    fs.open(argv[1], FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Failed to open param.yaml" << endl;
        return 1;
    }
    //load param
    fs["Record.sensor.imu"] >> record_sensor_imu;
    fs["Record.sensor.stereo"] >> record_sensor_stereo;
    fs["Record.sensor.rgbd"] >> record_sensor_rgbd;
    fs["Record.sensor.lidar"] >> record_sensor_lidar;
    fs["Record.pixhawk.px4flow"] >> record_pixhawk_px4flow;
    fs["Record.pixhawk.imu"] >> record_pixhawk_imu;
    fs["Record.vicon"] >> record_vicon;
    fs["Record.rtk"] >> record_rtk;
    fs["Record.file"] >> filePath;
    showParam();
    //init file
    if(record_sensor_imu){
        sensorIMUfile.open(filePath + "/sensorIMU.csv",ios::out);
        sensorIMUfile<<"timeStamp,linear acceleration x,linear acceleration y,linear acceleration z,"
                    <<"angular velocity x,angular velocity y,angular velocity z,"
                   <<"quaternion x,quaternion y,quaternion z,quaternion w \n";
    }
    if(record_sensor_stereo){
        stereoStampfile.open(filePath + "/stereoStamp.csv",ios::out);
        stereoStampfile<<"image ID,timeStamp \n";
    }
    if(record_sensor_rgbd){
        rgbdStampfile.open(filePath + "/rgbdStamp.csv",ios::out);
        rgbdStampfile<<"depth image ID,timeStamp \n";
        rgbdCameraInfofile.open(filePath + "/rgbd_CameraInfo.csv",ios::out);
    }
    if(record_sensor_lidar){
        Lidarfile.open(filePath + "/lidar.csv",ios::out);
        Lidarfile<<"timeStamp,ranges \n";
        LidarInfofile.open(filePath + "/lidar_info.csv",ios::out);

    }
    if(record_pixhawk_px4flow){
        px4flowfile.open(filePath + "/px4flow.csv",ios::out);
        px4flowfile<<"timeStamp,velocity x,velocity y,distance to ground \n";
    }
    if(record_pixhawk_imu){
        pixhawkImufile.open(filePath + "/pixhawkIMU.csv",ios::out);
        pixhawkImufile<<"timeStamp,linear acceleration x,linear acceleration y,linear acceleration z,"
                     <<"angular velocity x,angular velocity y,angular velocity z,"
                    <<"magnetic field x,magnetic field y,magnetic field z,"
                   <<"fluid pressure \n";
    }
    if(record_vicon){
        viconfile.open(filePath + "/vicon.csv",ios::out);
        viconfile<<"timeStamp,position x,position y,position z,"
                <<"quaternion x,quaternion y,quaternion z,quaternion w \n";
    }
    if(record_rtk){
        rtkfile.open(filePath + "/rtk.csv",ios::out);
    }
    //define subscriber
    //vicon
    ros::Subscriber vicon_sub = n.subscribe("/vicon/uav_stereo02/uav_stereo02",1, viconCb);;
    //stereo
    ros::Subscriber stereo_sub = n.subscribe("/sensor_msgs/stereoCompressed", 1, imgCompressedCb);
    //rtk
    ros::Subscriber rtk_sub; //待定
    //lidar
    ros::Subscriber lidar_sub = n.subscribe("/scan", 1, lidarCb);
    //px4flow
    message_filters::Subscriber<sensor_msgs::Range> px4flow_ground_sub(n, "/mavros/px4flow/ground_distance", 1);
    message_filters::Subscriber<OpticalFlowRad> px4flow_rad_sub(n, "/mavros/px4flow/raw/optical_flow_rad", 1);
    TimeSynchronizer<sensor_msgs::Range, OpticalFlowRad> sync_px4flow(px4flow_ground_sub, px4flow_rad_sub, 10);
    sync_px4flow.registerCallback(boost::bind(&px4flowCb, _1, _2));
    //imu
    message_filters::Subscriber<FluidPressure> imu_pressure_sub(n, "/mavros/imu/atm_pressure", 1);
    message_filters::Subscriber<MagneticField> imu_mag_sub(n, "/mavros/imu/mag", 1);
    message_filters::Subscriber<Imu> imu_raw_sub(n, "/mavros/imu/data_raw", 1);
    TimeSynchronizer<FluidPressure, MagneticField, Imu> sync_imu(imu_pressure_sub, imu_mag_sub,imu_raw_sub, 10);
    sync_imu.registerCallback(boost::bind(&imuCb, _1, _2,_3));
    ros::Subscriber imu_pose_sub = n.subscribe("/mavros/imu/data", 1, imuPoseCb);
    //rgbd
    message_filters::Subscriber<Image> rgbd_depth_sub(n, "/camera/depth/image", 1);
    message_filters::Subscriber<CameraInfo> rgbd_info_sub(n, "/camera/depth/camera_info", 1);
    TimeSynchronizer<Image, CameraInfo> sync_rgbd(rgbd_depth_sub, rgbd_info_sub, 10);
    sync_rgbd.registerCallback(boost::bind(&rgbdCb, _1, _2));

    ros::Rate r(100);
    while (ros::ok()){
        //r.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
    fs.release(); // explicit close
    sensorIMUfile.close();
    stereoStampfile.close();
    rgbdStampfile.close();
    Lidarfile.close();
    LidarInfofile.close();
    px4flowfile.close();
    pixhawkImufile.close();
    viconfile.close();
    rtkfile.close();
    rgbdCameraInfofile.close();
    return 0;
}

static void viconCb(const geometry_msgs::TransformStampedPtr& msg){
    if(!record_vicon)
        return;
    viconfile<<std::fixed<<msg->header.stamp.toSec()<<",";
    viconfile<<msg->transform.translation.x<<","<<msg->transform.translation.y<<","
            <<msg->transform.translation.z<<",";
    viconfile<<msg->transform.rotation.x<<","<<msg->transform.rotation.y<<","
            <<msg->transform.rotation.z<<","<<msg->transform.rotation.w<<"\n";
}

static void imgCompressedCb(const drone_sensor::StereoCompressedImagePtr& msg){

    if(!record_sensor_stereo)
        return;
    double nowTime=ros::Time::now().toSec();
    stereoStampfile<<img_no<<","<<std::fixed<<nowTime<<"\n";

    cv::Mat img1 = cv::imdecode(cv::Mat(msg->img_l.data),1);
    cv::Mat img2 = cv::imdecode(cv::Mat(msg->img_r.data),1);
    std::ostringstream oss;
    oss << filePath << "/stereo_img";
    mkdir(oss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    oss.str("");
    oss << filePath << "/stereo_img/left_" << img_no << ".png";
    cv::imwrite(oss.str().c_str(), img1);
    oss.str("");
    oss << filePath << "/stereo_img/right_" << img_no <<".png";
    cv::imwrite(oss.str().c_str(), img2);
    img_no++;
}
void rgbdCb(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
    if(!record_sensor_rgbd)
        return;
    //camera info
    if(!isCameraInfo){
        rgbdCameraInfofile<<"D:"<<cam_info->D[0]<<","<<cam_info->D[1]<<","<<cam_info->D[2]
                         <<","<<cam_info->D[3]<<","<<cam_info->D[4]<<"\n";
        rgbdCameraInfofile<<"K:"<<cam_info->K[0]<<","<<cam_info->K[1]<<","<<cam_info->K[2]
                         <<","<<cam_info->K[3]<<","<<cam_info->K[4]<<","<<cam_info->K[5]
                        <<","<<cam_info->K[6]<<","<<cam_info->K[7]<<","<<cam_info->K[8]<<"\n";
        rgbdCameraInfofile<<"P:"<<cam_info->P[0]<<","<<cam_info->P[1]<<","<<cam_info->P[2]
                         <<","<<cam_info->P[3]<<","<<cam_info->P[4]<<","<<cam_info->P[5]
                        <<","<<cam_info->P[6]<<","<<cam_info->P[7]<<","<<cam_info->P[8]<<"\n";
        isCameraInfo = true;
    }
    rgbdStampfile<<depth_no<<","<<std::fixed<<image->header.stamp.toSec()<<"\n";

    //depth image
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
    try
    {
        cv_ptr =  cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    }
    catch(cv_bridge::Exception& e)  //异常处理
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;
    std::ostringstream oss;
    oss << filePath << "/depth_img";
    mkdir(oss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    oss.str("");
    oss << filePath << "/depth_img/" << depth_no << ".mat";
    ofstream depthFile;
    depthFile.open(oss.str(),ios::out);
    depthFile<<img<<"\n";
    depth_no++;
}

void imuCb(const FluidPressureConstPtr& pressure, const MagneticFieldConstPtr& mag, const ImuConstPtr& imu_raw)
{
    if(!record_pixhawk_imu)
        return;
    pixhawkImufile<<std::fixed<<imu_raw->header.stamp.toSec()<<",";
    pixhawkImufile<<imu_raw->linear_acceleration.x<<","<<imu_raw->linear_acceleration.y<<","
                 <<imu_raw->linear_acceleration.z<<",";
    pixhawkImufile<<imu_raw->angular_velocity.x<<","<<imu_raw->angular_velocity.y<<","
                 <<imu_raw->angular_velocity.z<<",";
    pixhawkImufile<<mag->magnetic_field.x<<","<<mag->magnetic_field.y<<","
                 <<mag->magnetic_field.z<<",";
    pixhawkImufile<<double(pressure->fluid_pressure)<<"\n";
}
void imuPoseCb(const ImuConstPtr& imu)
{
    if(!record_sensor_imu)
        return;
    sensorIMUfile<<std::fixed<<imu->header.stamp.toSec()<<",";
    sensorIMUfile<<imu->linear_acceleration.x<<","<<imu->linear_acceleration.y<<","
                <<imu->linear_acceleration.z<<",";
    sensorIMUfile<<imu->angular_velocity.x<<","<<imu->angular_velocity.y<<","
                <<imu->angular_velocity.z<<",";
    sensorIMUfile<<imu->orientation.x<<","<<imu->orientation.y<<","
                <<imu->orientation.z<<","<<imu->orientation.w<<"\n";

}
void px4flowCb(const sensor_msgs::RangeConstPtr&ground, const OpticalFlowRadConstPtr &optical){
    if(!record_pixhawk_px4flow)
        return;
    double height = double(ground->range);
    double vx = 0.;
    double vy = 0.;
    double dt = double(optical->integration_time_us)/1000000.0;
    vx = double(optical->integrated_x)*height/dt;
    px4flowfile<<std::fixed<<optical->header.stamp.toSec()<<",";
    px4flowfile<<vx<<","<<vy<<","<<height<<"\n";
}
void lidarCb(const LaserScanConstPtr& scan){
    if(!record_sensor_lidar)
        return;
    //lidar info
    if(!isLidarInfo){
        LidarInfofile<<"angle_min:"<<std::fixed<<scan->angle_min<<"\n";
        LidarInfofile<<"angle_max:"<<std::fixed<<scan->angle_max<<"\n";
        LidarInfofile<<"angle_increment:"<<std::fixed<<scan->angle_increment<<"\n";
        LidarInfofile<<"time_increment:"<<std::fixed<<scan->time_increment<<"\n";
        LidarInfofile<<"scan_time:"<<std::fixed<<scan->scan_time<<"\n";
        LidarInfofile<<"range_min:"<<std::fixed<<scan->range_min<<"\n";
        LidarInfofile<<"range_max:"<<std::fixed<<scan->range_max<<"\n";
        isLidarInfo = true;
    }
    //lidar data
    Lidarfile<<std::fixed<<scan->header.stamp.toSec()<<"\n";
    float dis;
    for(int i=0;i<scan->ranges.size();++i){
        dis = scan->ranges[i];
        Lidarfile<<dis<<",";
    }
    Lidarfile<<"\n";
}
void showParam(){
    cout<<"File path : \t"<<filePath<<endl;
    ROS_INFO("Load params list");
    cout<<" sensor IMU :\t"<<(record_sensor_imu?"true":"false")<<endl;
    cout<<" sensor stereo :\t"<<(record_sensor_stereo?"true":"false")<<endl;
    cout<<" sensor rgbd :\t"<<(record_sensor_rgbd?"true":"false")<<endl;
    cout<<" sensor lidar :\t"<<(record_sensor_lidar?"true":"false")<<endl;
    cout<<" Pixhawk IMU :\t"<<(record_pixhawk_imu?"true":"false")<<endl;
    cout<<" Pixhawk px4flow :\t"<<(record_pixhawk_px4flow?"true":"false")<<endl;
    cout<<" Vicon :\t"<<(record_vicon?"true":"false")<<endl;
    cout<<" RTK :\t"<<(record_rtk?"true":"false")<<endl;
}
