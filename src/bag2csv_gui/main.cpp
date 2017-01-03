#include "uiwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{ 

    ROS_INFO("[BAG2CSV]bag to csv program is starting");
    ros::init(argc, argv, "bag2csv_gui_node");
    ros::NodeHandle n;
    QApplication a(argc, argv);
    MainWindow w;
    w.setNodeHandle(n);
    w.startReadingLoop();
    w.show();
    return a.exec();
}
