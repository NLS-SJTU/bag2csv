#include "myprocesshandler.h"
using namespace std;
myProcessHandler::myProcessHandler(ros::NodeHandle *nh)
{
    n=*nh;
    newComm = false;
    toRefresh = true;
    record_sub = n.subscribe("/recordComm",1, &myProcessHandler::recordCb,this);
}
myProcessHandler::~myProcessHandler()
{

}
void myProcessHandler::reading_loop(){
    ros::Rate r(10);
    while(ros::ok()){
        //cout<<"one term, new command? "<<newComm<<endl;
        ros::spinOnce();
        r.sleep();
    }
}
void* startReadThread(void *args)
{
    myProcessHandler *w = (myProcessHandler*)args;
    w->reading_loop();
    // done!
    return NULL;
}
void myProcessHandler::startReadingLoop(){
    pthread_create( &read_tid, NULL, &startReadThread, this );
}
void myProcessHandler::recordCb(const bag2csv::recordMsgConstPtr& msg){
    commMsg.bagName = msg->bagName;
    commMsg.playRate = msg->playRate;
    commMsg.savePath = msg->savePath;
    commMsg.recordIMU = msg->recordIMU;
    commMsg.recordStereo = msg->recordStereo;
    commMsg.recordRgbd = msg->recordRgbd;
    commMsg.recordLidar = msg->recordLidar;
    commMsg.recordPixhawkIMU = msg->recordPixhawkIMU;
    commMsg.recordPx4flow = msg->recordPx4flow;
    commMsg.recordVicon = msg->recordVicon;
    commMsg.recordRtk = msg->recordRtk;
    newComm = true;
    toRefresh = true;
    std::cout<<"new command"<<std::endl;
}
