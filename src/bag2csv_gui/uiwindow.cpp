#include "uiwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    isBagName = false;
    isSavePath = false;
    pocBagPlay = new QProcess;
    pocEndPlay = new QProcess;

    QDoubleValidator* playRateValidator = new QDoubleValidator;
    ui->playRateEdit->setValidator(playRateValidator);
    ui->playRateEdit->setText("1.0");
    //checkbox
    ui->checkIMU->setChecked(false);
    ui->checkLidar->setChecked(false);
    ui->checkPixhawkIMU->setChecked(false);
    ui->checkPx4flow->setChecked(false);
    ui->checkRgbd->setChecked(false);
    ui->checkRtk->setChecked(false);
    ui->checkStereo->setChecked(false);
    ui->checkVicon->setChecked(false);
    ui->finishButton->setDisabled(true);

    //record sensor list
    record_sensor_imu = false;
    record_sensor_stereo = false;
    record_sensor_rgbd = false;
    record_sensor_lidar = false;
    record_pixhawk_imu = false;
    record_pixhawk_px4flow = false;
    record_vicon = false;
    record_rtk = false;
    isCameraInfo = false;
    isLidarInfo = false;
    no_loop = true;
    closeFile = false;
    playRate = 1.0;
    info = "";
    infoTimer = new QTimer(this);
    connect( infoTimer, SIGNAL(timeout()), this, SLOT(showInfo()) );
}

MainWindow::~MainWindow()
{
    delete ui;
    if(pocBagPlay){
        pocBagPlay -> close();
    }
    pocBagPlay->kill();
    delete pocBagPlay;
    ros::shutdown();
    infoTimer->stop();
    delete infoTimer;
}

void MainWindow::setNodeHandle(ros::NodeHandle nh){
    n=nh;
    ui->infoEdit->setPlainText("[program] bag to csv!");
    recordPub = n.advertise<bag2csv::recordMsg>("/recordComm",2);
    infoSub = n.subscribe("/recordInfo",1, &MainWindow::infoCb,this);
}

void MainWindow::on_openButton_clicked()
{
    QString path = QFileDialog::getOpenFileName(this,
                                                tr("Open bag File"),
                                                ".",
                                                tr("Text Files(*.bag)"));
    if(!path.isEmpty()) {
        bagName = path.toStdString();
        isBagName = true;
    } else {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any file."));
    }
}

void MainWindow::on_saveButton_clicked()
{
    QString path = QFileDialog::getExistingDirectory(this,
                                                     tr("Choose save path"),
                                                     ".");
    if(!path.isEmpty()) {
        savePath = path.toStdString();
        isSavePath = true;
        cout<<savePath<<endl;
    } else {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any path."));
    }
}

void MainWindow::on_startButton_clicked()
{
    if(!isBagName || !isSavePath)
        return;
    playRate = ui->playRateEdit->text().toDouble();
    std::cout<<"play rate : "<<playRate<<endl;
    //check record topic
    record_sensor_imu = ui->checkIMU->isChecked();
    record_sensor_stereo = ui->checkStereo->isChecked();
    record_sensor_rgbd = ui->checkRgbd->isChecked();
    record_sensor_lidar = ui->checkLidar->isChecked();
    record_pixhawk_imu = ui->checkPixhawkIMU->isChecked();
    record_pixhawk_px4flow = ui->checkPx4flow->isChecked();
    record_vicon = ui->checkVicon->isChecked();
    record_rtk = ui->checkRtk->isChecked();
    showRecordList();
    bag2csv::recordMsg commMsg;
    commMsg.bagName = bagName;
    commMsg.savePath = savePath;
    commMsg.playRate = playRate;
    commMsg.recordIMU = record_sensor_imu;
    commMsg.recordStereo = record_sensor_stereo;
    commMsg.recordRgbd = record_sensor_rgbd;
    commMsg.recordLidar = record_sensor_lidar;
    commMsg.recordPixhawkIMU = record_pixhawk_imu;
    commMsg.recordPx4flow = record_pixhawk_px4flow;
    commMsg.recordVicon = record_vicon;
    commMsg.recordRtk = record_rtk;
    ros::Rate r(10);
    for (int i=0;i<1;++i){
        recordPub.publish(commMsg);
        r.sleep();
    }
    infoTimer->start(100);
}

void MainWindow::showRecordList(){
    std::ostringstream oss;
    oss<<"bag to convert : \t"<<bagName<<"\n";
    oss<<"save path : \t"<<savePath<<"\n";
    oss<<"Load params list"<<"\n";
    oss<<" sensor IMU :\t"<<(record_sensor_imu?"true":"false")<<"\n";
    oss<<" sensor stereo :\t"<<(record_sensor_stereo?"true":"false")<<"\n";
    oss<<" sensor rgbd :\t"<<(record_sensor_rgbd?"true":"false")<<"\n";
    oss<<" sensor lidar :\t"<<(record_sensor_lidar?"true":"false")<<"\n";
    oss<<" Pixhawk IMU :\t"<<(record_pixhawk_imu?"true":"false")<<"\n";
    oss<<" Pixhawk px4flow :\t"<<(record_pixhawk_px4flow?"true":"false")<<"\n";
    oss<<" Vicon :\t"<<(record_vicon?"true":"false")<<"\n";
    oss<<" RTK :\t"<<(record_rtk?"true":"false")<<"\n";
    ui->infoEdit->setPlainText(oss.str().c_str());
}
void MainWindow::reading_loop(){
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}
void* startReadThread(void *args)
{
    MainWindow *w = (MainWindow*)args;
    w->reading_loop();
    // done!
    return NULL;
}
void MainWindow::startReadingLoop(){
    pthread_create( &read_tid, NULL, &startReadThread, this );
}
void MainWindow::infoCb(const std_msgs::StringConstPtr& msg){
    //cout<<msg->data<<endl;
    info = msg->data;
    //ui->infoEdit->setPlainText(info.c_str());
}
void MainWindow::showInfo(){
    if(!info.empty()){
        ui->infoEdit->setPlainText(info.c_str());
    }
}
