#include <QDebug> //作为调试的库文件
#include <stdio.h>
#include "widget.h"
#include "ui_widget.h"

/*操作说明
 * 首先创建新文件夹
 * 然后打开相机，打开串口
 * 第三步采集标定图片，15-20张，然后标定内参
 * 第四步采集外参图片，1张，然后标定外参
 * 注意每一次内外参标定完后，要想重新标定，需要再次初始化目录
 */

/*
改进思路：
d单独标定，求取多个平面的，取平均，同时显示一下看看误差大不大
*/


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setFixedSize(this->width(), this->height());

    m_capture_timer=new QTimer(this);
    connect(m_capture_timer, SIGNAL(timeout()), this, SLOT(handleTimeout_capture()));
    connect(this,SIGNAL(finish_calc()),this,SLOT(finish_show_cal()));


    //初始化
    ui->doubleSpinBox->setSingleStep(1.125);
    ui->doubleSpinBox->setSuffix("°");
//    QPixmap pixmap;
//    pixmap.load(":/pic/camera.png");
//    ui->label_image->setPixmap(pixmap);

    flag_pluse=0;//初始化0为正方向
    send_pluse_num=0;
    step_pluse=0.028125;//每步的度数
    tem_angle=0;
    theta1_sys=0;
    theta2_sys=0;

    //以下是相机标志初始化设置
    m_bOpenDevice=false;
    m_pSaveImageBuf=nullptr;
    m_nSaveImageBufSize=0;

    //图片数目初始化
    image_save=0;
    inner_image=0;
    pnp_image=0;
    sys_image=0;
}

Widget::~Widget()
{
    delete ui;
}



int Widget::SetExposureTime(){
    // ch:调节这两个曝光模式，才能让曝光时间生效
    // en:Adjust these two exposure mode to allow exposure time effective
    int nRet = m_pcMyCamera->SetEnumValue("ExposureMode", MV_EXPOSURE_MODE_TIMED);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    m_pcMyCamera->SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);

    return m_pcMyCamera->SetFloatValue("ExposureTime", (float)m_dExposureEdit);

}


int Widget::SetGain(){
    // ch:设置增益前先把自动增益关闭，失败无需返回
    //en:Set Gain after Auto Gain is turned off, this failure does not need to return
    m_pcMyCamera->SetEnumValue("GainAuto", 0);

    return m_pcMyCamera->SetFloatValue("Gain", (float)m_dGainEdit);

}


// ch:设置帧率 | en:Set Frame Rate
int Widget::SetFrameRate()
{
    int nRet = m_pcMyCamera->SetBoolValue("AcquisitionFrameRateEnable", true);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    return m_pcMyCamera->SetFloatValue("AcquisitionFrameRate", (float)m_dFrameRateEdit);
}


//标定内参按钮：调试阶段，读取文件夹进行标定
void Widget::on_inner_cali_pushButton_clicked()
{
    //extern
    bool m_calibration(vector<string> FilesName, Size board_size, Size square_size,
        Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecsMat, vector<Mat> &tvecsMat, string File_Directory);


   //读取图片
   string File_Directory1 = calib_dir_image.toStdString();   //文件夹目录
   string FileType = ".bmp";    // 需要查找的文件类型
   vector<string>FilesName1;    //存放文件名的容器
   bool ack1=getFilesName(File_Directory1, FileType, FilesName1);   // 标定所用图像文件路径向量
   if(ack1){
       QMessageBox::warning(this, tr("readImage"), tr("read image success!"));
   }else{
       QMessageBox::warning(this, tr("readImage"), tr("read image fail!"));
   }
   //先关闭采集图片线程
   on_stop_capture_clicked();


   //标定处理，注意根据实际更改
   Size board_size = Size(11, 8);                         // 标定板上每行、列的角点数
   Size2d square_size = Size2d(1.5, 1.5);                       // 实际测量得到的标定板上每个棋盘格的物理尺寸，单位mm

   Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));        // 摄像机内参数矩阵
   Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));          // 摄像机的5个畸变系数：k1,k2,p1,p2,k3, 一行五列
   vector<Mat> rvecsMat;                                          // 存放所有图像的旋转向量，每一副图像的旋转向量为一个mat
   vector<Mat> tvecsMat;                                          // 存放所有图像的平移向量，每一副图像的平移向量为一个mat

   bool ack2=m_calibration(FilesName1, board_size, square_size, cameraMatrix,
                 distCoeffs, rvecsMat, tvecsMat, calib_result.toStdString());
   if(ack2){
       QMessageBox::warning(this, tr("inner calibration"), tr("calibrate success!"));
   }else{
       QMessageBox::warning(this, tr("inner calibration"), tr("calibrate fail!"));
   }
   //打开采集线程
   on_start_capture_clicked();
}


//输入参数标定外参按钮
void Widget::on_inner_matrix_pushButton_clicked()
{
    //extern
    bool read_calibra(string fileName, Mat &mat_inner, Mat &dist);
    bool cali_outer_R_T(string FileName, Size board_size, Size square_size,
        Mat &cameraMatrix, Mat &distCoeffs, Mat &rvecsMat, Mat &tvecsMat, Mat &rotation_matrix, string save_Dir);

    //选取文件，初始化为debug文件夹
    QString file_input=QFileDialog::getOpenFileName(this,"open calibration txt",QDir::currentPath(),
                                                       "txt(*.txt)");
    if(!QFile::exists(file_input)){
        QMessageBox::warning(this,tr("read inner"),tr("read inner fail!"));
        return;
    }
    Mat mat_inner = Mat::zeros(3, 3, CV_64FC1);//创建全零
    Mat dist = Mat::zeros(5, 1, CV_64FC1);//5行一列
    read_calibra(file_input.toStdString(),mat_inner,dist);

    //显示标定信息
    stringstream s_temp;
    s_temp << mat_inner << endl;
    string output_cali;
    output_cali="cameraMatrix\t\n"+s_temp.str()+"\t\ndistCoeffs\t\n";
    s_temp.str("");
    s_temp<<dist<<endl;
    output_cali=output_cali+s_temp.str();
    QMessageBox::warning(this, tr("calibration information"), QString::fromStdString(output_cali));

    //关闭采集线程
    on_stop_capture_clicked();

    //外参标定
    Size board_size = Size(11, 8);                         // 标定板上每行、列的角点数
    Size2d square_size = Size2d(1.5, 1.5);                       // 实际测量得到的标定板上每个棋盘格的物理尺寸，单位mm

    Mat rvecsMat;                                          // 存放所有图像的旋转向量，每一副图像的旋转向量为一个mat
    Mat tvecsMat;                                          // 存放所有图像的平移向量，每一副图像的平移向量为一个mat
    Mat rotation_res;
    bool ack1;
    //这次不是输入目录，只需要输入一张bmp图片即可,所以不需要string的vector数组
    QString add_1=QString::number(pnp_image);
    QString end_1=pnp_dir+"/"+add_1+".bmp";
    ack1=cali_outer_R_T(end_1.toStdString(),board_size,square_size,
                   mat_inner,dist,rvecsMat,tvecsMat,rotation_res,pnp_result.toStdString());
    if(ack1){
        //保存参数
        CAMERA_matrix=mat_inner.clone();
        DISTCoeffs=dist.clone();
        ROTATION_camera=rotation_res.clone();
        TVECSmat=tvecsMat.clone();
        QMessageBox::warning(this, tr("solve pnp"), tr("solve pnp success and already save!"));
    }else{
        QMessageBox::warning(this, tr("solve pnp"), tr("solve pnp fail"));
    }
    //打开采集线程
    on_start_capture_clicked();
}




//初始化文件夹按钮
void Widget::on_setup_clicked()
{
    //初始化输出文件夹，格式是/xx/xx/
    QString fileName = QCoreApplication::applicationDirPath();
    QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
    QString str = time.toString("yyyy_MM_dd_hh_mm_ss"); //设置显示格式
    QDir *folder = new QDir;
    QString newdir=fileName+"/"+str;
    folder->mkdir(newdir);

    calib_dir_image=fileName+"/"+str+"/"+"calib_image";
    bool ok1=folder->mkdir(calib_dir_image);

    pnp_dir=fileName+"/"+str+"/"+"pnp_image";
    bool ok2=folder->mkdir(pnp_dir);

    calib_result=fileName+"/"+str+"/"+"calib_result";
    bool ok3=folder->mkdir(calib_result);

    pnp_result=fileName+"/"+str+"/"+"pnp_result";
    bool ok4=folder->mkdir(pnp_result);

    Cail_sys_image=fileName+"/"+str+"/"+"calib_sys_image";
    bool ok5=folder->mkdir(Cail_sys_image);

    Cail_sys_result=fileName+"/"+str+"/"+"calib_sys_result";
    bool ok6=folder->mkdir(Cail_sys_result);

    //保存图片按钮的目录
    image_docu_save=fileName+"/"+str+"/"+"test_image_save";
    bool ok7=folder->mkdir(image_docu_save);


    //三维处理目录
    three_dimension_save=fileName+"/"+str+"/"+"3D_save";
    bool ok8=folder->mkdir(three_dimension_save);

    three_dimension_result=fileName+"/"+str+"/"+"3D_result";
    bool ok9=folder->mkdir(three_dimension_result);

    if(ok1&&ok2&&ok3&&ok4&&ok5&&ok6&&ok7&&ok8&&ok9)
        QMessageBox::warning(this, tr("CreateDir"), tr("Create Dir success!"));
    else
        QMessageBox::warning(this, tr("CreateDir"), tr("Create Dir fail"));

    delete folder;


}


//打开电机串口按钮：电机串口启动
void Widget::on_motor_on_pushButton_clicked()
{
    QList<QSerialPortInfo> my_serial_name;
    QSerialPortInfo info_qstring;
//    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
//    {
//        qDebug()<<"serialPortName:"<<info.portName();
//        info_qstring=info;
//    }

    my_serial_name=QSerialPortInfo::availablePorts();
    if (my_serial_name.isEmpty()){
        QMessageBox::warning(this,tr("open serial"),tr("do not find serial"));
        return;
    }
    info_qstring=my_serial_name[0];
    m_serialPort.setPortName(info_qstring.portName());
    if(m_serialPort.isOpen())//如果串口已经打开了 先给他关闭了
    {
        m_serialPort.clear();
        m_serialPort.close();
    }
    if(!m_serialPort.open(QIODevice::ReadWrite))
    {
        QMessageBox::warning(this, tr("open serial"), tr("open serial fail"));
        return;
    }
    //打开成功
    m_serialPort.clear();
    m_serialPort.setBaudRate(QSerialPort::Baud9600,QSerialPort::AllDirections);//波特率9600和读取方向
    m_serialPort.setDataBits(QSerialPort::Data8);//数据位8位
    m_serialPort.setFlowControl(QSerialPort::NoFlowControl);//无流方向
    m_serialPort.setParity(QSerialPort::NoParity);//没有校验位
    m_serialPort.setStopBits(QSerialPort::OneStop);//1位停止位

    QMessageBox::warning(this, tr("open serial"), tr("open serial successfully"));
    ui->close_serial->setEnabled(true);
    ui->motor_on_pushButton->setEnabled(false);
    ui->motor_Zero->setEnabled(false);
    ui->motor_condition->setEnabled(true);
//    QThread::msleep(500);
//    //以下代码只是为了串口握手初始化
//    char send_flag_tem=0xDD;
//    m_serialPort.write(&send_flag_tem);
//    QByteArray temp = m_serialPort.readAll();
//    long tem1=temp.toLong();
//    qDebug()<<temp<<endl;
//    qDebug()<<tem1<<endl;
//    QThread::msleep(50);
//    m_serialPort.write(&send_flag_tem);
//    temp = m_serialPort.readAll();
//    tem1=temp.toLong();
//    qDebug()<<temp<<endl;
//    qDebug()<<tem1<<endl;

}



//输入角度转动按钮
void Widget::on_mov_motor_clicked()
{
//按照字节来使用char数组,截断常量值警告没问题
//电机第一次启动，num不变化，所以最好先读一下状态
    char send_zero=0xDD;
    m_serialPort.write(&send_zero);
    QByteArray temp = m_serialPort.readAll();
    if(temp.size()==0){
        QMessageBox::warning(this,"turn zero","fail get location!");
        return;
    }
    int length=temp.size();
    if(temp[0]!='A' && temp[length-1]!='C'){
        QMessageBox::warning(this,"turn zero","fail get location2!");
        return;
    }
    int pred=1;
    QByteArray loca;
    QByteArray dir;
    for(int i=pred;i<=length;++i){
        if(temp[i]!='B'){
            loca.append(temp[i]);
        }else {
            pred=i;
            break;
        }
    }
    for(int i=pred+1;i<=length;++i){
        if(temp[i]!='C'){
            dir.append(temp[i]);
        }else {
            pred=i;
            break;
        }
    }
    long location=loca.toLong();
    int direction=dir.toInt();
    //动起来
    double tem_num=ui->doubleSpinBox->value();
    long long tem_num1=tem_num/step_pluse;
    tem_angle=tem_num;//保存角度
    if(location>tem_num1){
        if(direction==0){
            send_zero=0xAD;
            m_serialPort.write(&send_zero);
        }
        QThread::msleep(50);
        send_zero=0xBB;
        for(int i=0;i<(location-tem_num1);++i){
            m_serialPort.write(&send_zero);
            QThread::msleep(10);
        }
        QThread::msleep(50);
        //调回正向
        send_zero=0xAD;
        m_serialPort.write(&send_zero);
    }else if(location<tem_num1){
        if(direction==1){
            send_zero=0xAD;
            m_serialPort.write(&send_zero);
        }
        QThread::msleep(50);
        send_zero=0xBB;
        for(int i=0;i<abs(location-tem_num1);++i){
            m_serialPort.write(&send_zero);
            QThread::msleep(10);
        }

    }
    QThread::msleep(100);
    QMessageBox::warning(this,"move motor","finish move!");
    ui->mov_motor->setEnabled(false);
    ui->motor_Zero->setEnabled(false);
//    char str1;
//    double tem_num=ui->doubleSpinBox->value();
//    long long tem_num1;
//    //判断方向和计算次数
//    if (tem_num>tem_angle)
//    {
//        //0才是正向
//        qDebug()<<flag_pluse<<endl;
//        if(1==flag_pluse){
//            str1=0xAD;
//            m_serialPort.write(&str1);
//            QThread::msleep(5);
//            flag_pluse=0;
//            qDebug()<<flag_pluse<<endl;
//        }
//        tem_num1=(tem_num-tem_angle)/step_pluse;
//        qDebug()<<tem_num1<<endl;
//        str1=0xBB;
//        for(int i=0;i<tem_num1;++i){
//            m_serialPort.write(&str1);
//            QThread::msleep(10);
//        }
//        tem_angle=tem_num;
//    }

//    else if(tem_num<tem_angle){
//        //反转
//        qDebug()<<flag_pluse<<endl;
//        if(0==flag_pluse){
//            str1=0xAD;
//            m_serialPort.write(&str1);
//            QThread::msleep(5);
//            flag_pluse=1;
//            qDebug()<<flag_pluse<<endl;
//        }
//        tem_num1=(tem_angle-tem_num)/step_pluse;
//        qDebug()<<tem_num1<<endl;
//        str1=0xBB;
//        for(int i=0;i<tem_num1;++i){
//            m_serialPort.write(&str1);
//            QThread::msleep(10);
//        }
//        tem_angle=tem_num;
//    }
//    QThread::msleep(20);
//    QMessageBox::warning(this, tr("move"), tr("ok"));


}


//打开相机按钮
void Widget::on_find_openCamera_clicked()
{
//    //自动初始化文件夹
//    on_setup_clicked();
    //寻找设备， 只是网口相机
    memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);
    if (MV_OK != nRet)
    {
        return;
    }
    if(m_stDevList.nDeviceNum!=1){
        QMessageBox::warning(this, tr("check camera"), tr("do not find camera"));
        return;
    }

    //打开相机
    int i=0;
    m_pcMyCamera = new CMvCamera;
    if (NULL == m_pcMyCamera)
    {
        return;
    }
    nRet = m_pcMyCamera->Open(m_stDevList.pDeviceInfo[i]);
    if (MV_OK != nRet)
    {
        delete m_pcMyCamera;
        m_pcMyCamera = NULL;
        QMessageBox::warning(this,tr("open camera"),tr("open camera fail"));
        return;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效)
    if (m_stDevList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
    {
        unsigned int nPacketSize = 0;
        nRet = m_pcMyCamera->GetOptimalPacketSize(&nPacketSize);
        if (nRet == MV_OK)
        {
            nRet = m_pcMyCamera->SetIntValue("GevSCPSPacketSize",nPacketSize);
            if(nRet != MV_OK)
            {
                QMessageBox::warning(this,tr("Get Packet"),tr("Get Packet fail"));
            }
        }
        else
        {
            QMessageBox::warning(this,tr("Get Packet"),tr("Get Packet fail"));
        }
    }

    //获取曝光时间
    MVCC_FLOATVALUE stFloatValue = {0};
    nRet = m_pcMyCamera->GetFloatValue("ExposureTime", &stFloatValue);
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("Get exposure"),tr("Get exposure fail"));
        return;
    }
    m_dExposureEdit = stFloatValue.fCurValue;
    QString str = QString::number(m_dExposureEdit, 'f', 8);
    ui->exposure->setText(str);

    //获取增益
    stFloatValue = {0};
    nRet = m_pcMyCamera->GetFloatValue("Gain", &stFloatValue);
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("Get Gain"),tr("Get Gain fail"));
        return;
    }
    m_dGainEdit = stFloatValue.fCurValue;
    str = QString::number(m_dGainEdit, 'f', 8);
    ui->Gain_edit->setText(str);

    //获取帧率
    stFloatValue = {0};
    nRet = m_pcMyCamera->GetFloatValue("ResultingFrameRate", &stFloatValue);
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("Get FrameRate"),tr("Get FrameRate fail"));
        return;
    }
    m_dFrameRateEdit = stFloatValue.fCurValue;
    str = QString::number(m_dFrameRateEdit, 'f', 8);
    ui->FrameRate->setText(str);


    QMessageBox::warning(this, tr("Open Camera"), tr("Open Camera Successfully"));
    ui->statusbar->showMessage("open camera",3000);
    m_bOpenDevice=true;

    //自动初始化文件夹
    on_setup_clicked();

    //设置按钮状态
    ui->param_set->setEnabled(true);
    ui->start_capture->setEnabled(true);
    ui->stop_capture->setEnabled(true);
    ui->save_bmp->setEnabled(true);

    ui->inner_cali_pushButton->setEnabled(true);
    ui->inner_pushButton->setEnabled(true);
    ui->inner_matrix_pushButton->setEnabled(true);
    ui->outer_pushButton->setEnabled(true);
    ui->save_for_sys->setEnabled(true);
    ui->extract_line->setEnabled(true);
    ui->cali_sys->setEnabled(true);
    ui->src_capture->setEnabled(true);
    ui->cali_D->setEnabled(true);
    ui->cap_image_D->setEnabled(true);

    //三维处理
    ui->grab_image->setEnabled(true);
    ui->line_cap->setEnabled(true);
    ui->three_point->setEnabled(true);
    ui->auto_end->setEnabled(true);
    ui->auto_three->setEnabled(true);
    return;
//    qDebug()<<m_stDevList.nDeviceNum<<endl;
//    qDebug()<<m_stDevList.pDeviceInfo[0]<<endl;

}


//参数设置按钮
void Widget::on_param_set_clicked()
{
//    double i=15.3485;
//    QString str = QString::number(i, 'f', 8);
//    ui->exposure->setText(str);
    m_dExposureEdit=ui->exposure->text().toDouble();
    m_dGainEdit=ui->Gain_edit->text().toDouble();
    m_dFrameRateEdit=ui->FrameRate->text().toDouble();
    bool bIsSetSucceed = true;
    int nRet = SetExposureTime();
    if (nRet != MV_OK)
    {
     bIsSetSucceed = false;
     QMessageBox::warning(this, tr("set param"), tr("set param fail"));
     return;
    }
    nRet = SetGain();
    if (nRet != MV_OK)
    {
     bIsSetSucceed = false;
     QMessageBox::warning(this, tr("set param"), tr("set param fail"));
     return;
    }
    nRet = SetFrameRate();
    if (nRet != MV_OK)
    {
     bIsSetSucceed = false;
     QMessageBox::warning(this, tr("set param"), tr("set param fail"));
     return;
    }

    if (true == bIsSetSucceed)
    {
     QMessageBox::warning(this, tr("set param"), tr("set param successfully"));
    }

}


//关闭相机按钮
void Widget::on_close_camera_clicked()
{
    if(m_bOpenDevice){
        if (m_pcMyCamera)
        {
            m_pcMyCamera->Close();
            delete m_pcMyCamera;
            m_pcMyCamera = NULL;
        }
        //以下是相机标志初始化设置
        m_bOpenDevice=false;
        m_pSaveImageBuf=nullptr;
        m_nSaveImageBufSize=0;

        //图片数目初始化
        image_save=0;
        inner_image=0;
        pnp_image=0;
        sys_image=0;

        //按钮状态初始化
        ui->param_set->setEnabled(false);
        ui->start_capture->setEnabled(false);
        ui->stop_capture->setEnabled(false);
        ui->save_bmp->setEnabled(false);

        ui->inner_cali_pushButton->setEnabled(false);
        ui->inner_pushButton->setEnabled(false);
        ui->inner_matrix_pushButton->setEnabled(false);
        ui->outer_pushButton->setEnabled(false);
        ui->save_for_sys->setEnabled(false);
        ui->src_capture->setEnabled(false);
        ui->extract_line->setEnabled(false);
        ui->cali_sys->setEnabled(false);
        ui->cali_D->setEnabled(false);
        ui->cap_image_D->setEnabled(false);

        //三维处理初始化
        ui->grab_image->setEnabled(false);
        ui->line_cap->setEnabled(false);
        ui->three_point->setEnabled(false);
        ui->auto_end->setEnabled(false);
        ui->auto_three->setEnabled(false);

        QPixmap pixmap;
        pixmap.load(":/pic/camera.png");
        ui->label_image->setPixmap(pixmap);
        QMessageBox::warning(this, tr("Close Camera"), tr("Close Camera Successfully"));
    }else{
        QMessageBox::warning(this, tr("Close Camera"), tr("please open the camera!"));
    }
}



//关闭串口按钮
void Widget::on_close_serial_clicked()
{

    if(m_serialPort.isOpen())//如果串口已经打开了 先给他关闭了
    {
        m_serialPort.clear();
        m_serialPort.close();
    }
    QMessageBox::warning(this,tr("close serial"),tr("close serial successful"));
    ui->close_serial->setEnabled(false);
    ui->motor_on_pushButton->setEnabled(true);
    ui->motor_Zero->setEnabled(false);
    ui->motor_condition->setEnabled(false);
    ui->mov_motor->setEnabled(false);
}



//采集并显示图像：子线程函数
void Widget::GrabThreadProcess()
{
    MV_FRAME_OUT stImageInfo = {0};
    while(m_bThreadState){
//        qDebug() << __FUNCTION__<< QThread::currentThreadId();//输出函数名+线程ID
        int nRet = m_pcMyCamera->GetImageBuffer(&stImageInfo,1000);//1000是超时时间，这个函数获得图片数据
        //显示
        if(MV_OK==nRet){
            //保存的数据，注意加锁-------------------------------
            m_mutex.lock();
            if (NULL == m_pSaveImageBuf || stImageInfo.stFrameInfo.nFrameLen > m_nSaveImageBufSize)
                {
                    if (m_pSaveImageBuf)
                    {
                        free(m_pSaveImageBuf);
                        m_pSaveImageBuf = NULL;
                    }

                    m_pSaveImageBuf = (unsigned char *)malloc(sizeof(unsigned char) * stImageInfo.stFrameInfo.nFrameLen);
                    m_nSaveImageBufSize = stImageInfo.stFrameInfo.nFrameLen;
                }
            memcpy(m_pSaveImageBuf, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nFrameLen);//取数据
            memcpy(&m_stImageInfo, &(stImageInfo.stFrameInfo), sizeof(MV_FRAME_OUT_INFO_EX));
            m_mutex.unlock();
            //--------------------------------------------------
//            qDebug()<<stImageInfo.pBufAddr<<endl;
            QImage image_show(stImageInfo.pBufAddr,
                             stImageInfo.stFrameInfo.nWidth,
                             stImageInfo.stFrameInfo.nHeight,
                             QImage::Format_Grayscale8);
            image_show = image_show.scaled(ui->label_image->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            QPixmap pixmap_show=QPixmap::fromImage(image_show);
            ui->label_image->setPixmap(pixmap_show);
        }
        m_pcMyCamera->FreeImageBuffer(&stImageInfo);//释放对象
        QThread::msleep(50);

    }
}



//开始采集按钮
void Widget::on_start_capture_clicked()
{
//    qDebug() << __FUNCTION__<< QThread::currentThreadId();//输出函数名+线程ID
    //==============================================================================
    if (false == m_bOpenDevice || true == m_bStartGrabbing || NULL == m_pcMyCamera)
    {
        return;
    }
    memset(&m_stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    //开启线程
    int nRet = m_pcMyCamera->StartGrabbing();
    m_bThreadState = true;
    QFuture<void> f_camera =QtConcurrent::run(this, &Widget::GrabThreadProcess);

    if (MV_OK != nRet)
    {
        m_bThreadState = false;
        QMessageBox::warning(this, tr("Start grabbing"), tr("Start grabbing fail!"));
        return;
    }
//    QMessageBox::warning(this, tr("Start grabbing"), tr("Start grabbing success!"));
    m_bStartGrabbing = true;
    ui->statusbar->showMessage("Start grabbing success!",1000);
    return;
}


//关闭采集按钮，使得图像静止，观察是否适合，然后保存
void Widget::on_stop_capture_clicked()
{
    if (false == m_bOpenDevice || false == m_bStartGrabbing || NULL == m_pcMyCamera)
    {
        return;
    }
    m_bThreadState=false;
    int nRet = m_pcMyCamera->StopGrabbing();
    if (MV_OK != nRet)
    {
        m_bStartGrabbing = true;
        QMessageBox::warning(this,tr("stop grabing"),tr("stop grabing fail!"));
        return;
    }
    m_bStartGrabbing = false;
//    QMessageBox::warning(this,tr("stop grabing"),tr("stop grabing successful!"));
    ui->statusbar->showMessage("stop grabing successful!",1000);
    return;
}


//保存图像为BMP格式,注意先初始化目录，后期可以设置一个全部初始化的函数。打开目录才可以。
void Widget::on_save_bmp_clicked()
{
    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
           //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    ++image_save;
    QString add_str=QString::number(image_save);
    QString str_temp=image_docu_save+"/"+add_str+".bmp";
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
//    qDebug()<<strlen(c_temp)<<endl;
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();
    //解锁------------------------------------------
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_2"));
        return;
    }
    QString show_str="save test BMP "+add_str+" successful";
    QMessageBox::warning(this,tr("save BMP"),show_str);

}



//提取中心线按钮
void Widget::on_extract_line_clicked()
{
//    //测试
//    //寻找矩形边框
//    Rect roi_board;
//    QString cail_sys_result="E:/pro_test/QT/build-calibration_v1-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug/debug/2021_12_17_12_53_20/calib_sys_result";
//    QString srcpath="E:/pro_test/QT/build-calibration_v1-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug/debug/2021_12_17_12_53_20/calib_sys_image/src.bmp";
//    Mat src=imread(srcpath.toStdString());
//    Mat out_with_roi;
//    FindRect(src, roi_board ,out_with_roi);
//    if(roi_board.x==0&&roi_board.y==0&&roi_board.height==0&&
//            roi_board.width==0){
//        QMessageBox::warning(this,"Find rect","Find rect fail!");
//        return;
//    }
//    //保存结果
//    QString save_o=cail_sys_result+"/"+"src_roi.bmp";
//    imwrite(save_o.toStdString(),out_with_roi);

//    //提取中心线1
//    QString path_sys1="E:/pro_test/QT/build-calibration_v1-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug/debug/2021_12_17_12_53_20/calib_sys_image/1.bmp";
//    Mat image_first=imread(path_sys1.toStdString());
//    Mat outline;
//    Mat out;
//    vector<KeyPoint> pt_0;
//    pt_0.clear();
//    vector<KeyPoint> ptcorr;
//    ptcorr.clear();
//    extern void ExtraLine(Mat src_input, Mat &out_line,Mat &output,
//                          vector<KeyPoint> &Pt_out, vector<KeyPoint> &Pt_corr_out);
//    ExtraLine(image_first,outline,out,pt_0,ptcorr);
//    qDebug()<<"pt1 size="<<ptcorr.size()<<endl;
//    save_o=cail_sys_result+"/"+"1_1.bmp";
//    imwrite(save_o.toStdString(),outline);
//    save_o=cail_sys_result+"/"+"1_2.bmp";
//    imwrite(save_o.toStdString(),out);
//    //插入类成员变量
////    for(auto &i:pt_0){
////        if(i.pt.x>roi_board.x && i.pt.x<roi_board.x+roi_board.width &&
////           i.pt.y>roi_board.y && i.pt.y<roi_board.y+roi_board.height){
////            image1_.push_back(i);
////        }
////    }
//    qDebug()<<"pt1 capture finish"<<endl;

//    //提取中心线2
//    QString path_sys2="E:/pro_test/QT/build-calibration_v1-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug/debug/2021_12_17_12_53_20/calib_sys_image/2.bmp";
//    Mat image_second=imread(path_sys2.toStdString());
//    Mat outline2;
//    Mat out2;
//    vector<KeyPoint> pt_1;
//    pt_1.clear();
//    vector<KeyPoint> ptcorr1;
//    ptcorr1.clear();
//    ExtraLine(image_second,outline2,out2,pt_1,ptcorr1);
//    qDebug()<<"pt2 size="<<ptcorr1.size()<<endl;
//    save_o=cail_sys_result+"/"+"2_1.bmp";
//    imwrite(save_o.toStdString(),outline2);
//    save_o=cail_sys_result+"/"+"2_2.bmp";
//    imwrite(save_o.toStdString(),out2);
////    //插入类成员变量
////    int num=0;
////    int max_n=image1_.size();
////    qDebug()<<"pred imageL1=: "<<max_n<<endl;
////    image2_.clear();
////    for(auto &i:pt_1){
////        if(i.pt.x>roi_board.x && i.pt.x<roi_board.x+roi_board.width &&
////           i.pt.y>roi_board.y && i.pt.y<roi_board.y+roi_board.height){
////            image2_.push_back(i);
////        }
////    }
//    image1_.clear();
//    image2_.clear();
//    int n1=pt_0.size();
//    int n2=pt_1.size();
//    for(int i=roi_board.y;i<roi_board.y+roi_board.height;++i){
//        for(int j=0;j<n1;++j){
//            if(pt_0[j].pt.y==i){
//                for(int k=0;k<n2;++k){
//                    if(pt_1[k].pt.y==i){
//                        image1_.push_back(pt_0[j]);
//                        image2_.push_back(pt_1[k]);
//                        break;
//                    }
//                }
//                break;
//            }
//        }
//    }
////    for(auto &i:pt_1){
////        if(i.pt.x>roi_board.x && i.pt.x<roi_board.x+roi_board.width &&
////           i.pt.y>roi_board.y && i.pt.y<roi_board.y+roi_board.height){
////            if(i.pt.y<=image1_[num].pt.y){
////                if(i.pt.y==image1_[num].pt.y){
////                   image2_.push_back(i);
////                   ++num;
////                   if(num>=max_n){
////                       break;
////                   }
////                }
////            }else if(i.pt.y>image1_[num].pt.y){
////                image1_.erase(image1_.begin()+num);
////                --max_n;
////            }

////        }
////    }
////    if(num<max_n){
////        int new_num=image2_.size();
////        image1_.assign(image1_.begin(),image1_.begin()+new_num);
////    }

//    qDebug()<<"last pt1 size="<<image1_.size()<<endl;
//    qDebug()<<"last pt2 size="<<image2_.size()<<endl;
//    QMessageBox::warning(this,"extra line","finish extra line");


    //src----------------12.17
    if(flag_srcChess!=1 && sys_image!=2){
        QMessageBox::warning(this,"Extra Line","please capture images!");
        return;
    }
    //寻找矩形边框
    Rect roi_board;
    QString srcpath=Cail_sys_image+"/"+"src.bmp";
    Mat src=imread(srcpath.toStdString());
    Mat out_with_roi;
    FindRect(src, roi_board ,out_with_roi);
    if(roi_board.x==0&&roi_board.y==0&&roi_board.height==0&&
            roi_board.width==0){
        QMessageBox::warning(this,"Find rect","Find rect fail!");
        return;
    }
    //保存结果
    QString save_o=Cail_sys_result+"/"+"src_roi.bmp";
    imwrite(save_o.toStdString(),out_with_roi);

    //提取中心线1
    QString path_sys1=Cail_sys_image+"/"+"1.bmp";
    Mat image_first=imread(path_sys1.toStdString());
    Mat outline;
    Mat out;
    vector<KeyPoint> pt_0;
    pt_0.clear();
    vector<KeyPoint> ptcorr;
    ptcorr.clear();
    extern void ExtraLine(Mat src_input, Mat &out_line,Mat &output,
                          vector<KeyPoint> &Pt_out, vector<KeyPoint> &Pt_corr_out);
    ExtraLine(image_first,outline,out,pt_0,ptcorr);
    qDebug()<<"pt1 size="<<ptcorr.size()<<endl;
    save_o=Cail_sys_result+"/"+"1_1.bmp";
    imwrite(save_o.toStdString(),outline);
    save_o=Cail_sys_result+"/"+"1_2.bmp";
    imwrite(save_o.toStdString(),out);
    qDebug()<<"pt1 capture finish"<<endl;

    //提取中心线2
    QString path_sys2=Cail_sys_image+"/"+"2.bmp";
    Mat image_second=imread(path_sys2.toStdString());
    Mat outline2;
    Mat out2;
    vector<KeyPoint> pt_1;
    pt_1.clear();
    vector<KeyPoint> ptcorr1;
    ptcorr1.clear();
    ExtraLine(image_second,outline2,out2,pt_1,ptcorr1);
    qDebug()<<"pt2 size="<<ptcorr1.size()<<endl;
    //保存结果
    save_o=Cail_sys_result+"/"+"2_1.bmp";
    imwrite(save_o.toStdString(),outline2);
    save_o=Cail_sys_result+"/"+"2_2.bmp";
    imwrite(save_o.toStdString(),out2);

    //分离点
    image1_.clear();
    image2_.clear();
    int n1=pt_0.size();
    int n2=pt_1.size();
    for(int i=roi_board.y;i<roi_board.y+roi_board.height;++i){
        for(int j=0;j<n1;++j){
            if(pt_0[j].pt.y==i){
                for(int k=0;k<n2;++k){
                    if(pt_1[k].pt.y==i){
                        image1_.push_back(pt_0[j]);
                        image2_.push_back(pt_1[k]);
                        break;
                    }
                }
                break;
            }
        }
    }

    qDebug()<<"last imageL1 size="<<image1_.size()<<endl;
    qDebug()<<"last imageL2 size="<<image2_.size()<<endl;
    QMessageBox::warning(this,"extra line","finish extra line");
    //src_end-----------------------------------------------------
}




//标定系统参数按钮
void Widget::on_cali_sys_clicked()
{
    //读取内参
    //extern
    bool read_calibra(string fileName, Mat &mat_inner, Mat &dist);

    //选取文件，初始化为debug文件夹
    QString file_input=QFileDialog::getOpenFileName(this,"open calibration txt",QDir::currentPath(),
                                                       "txt(*.txt)");
    if(!QFile::exists(file_input)){
        QMessageBox::warning(this,tr("read inner"),tr("read inner fail!"));
        return;
    }
    Mat mat_inner = Mat::zeros(3, 3, CV_64FC1);//创建全零
    Mat dist = Mat::zeros(5, 1, CV_64FC1);//5行一列，然而创建的是一行五列，存疑
    read_calibra(file_input.toStdString(),mat_inner,dist);

    //显示标定信息
    stringstream s_temp;
    s_temp << mat_inner << endl;
    string output_cali;
    output_cali="cameraMatrix\t\n"+s_temp.str()+"\t\ndistCoeffs\t\n";
    s_temp.str("");//清空输入流
    s_temp<<dist<<endl;
    output_cali=output_cali+s_temp.str();
    QMessageBox::warning(this, tr("calibration information"), QString::fromStdString(output_cali));


    //读取外参
    //选取文件，初始化为debug文件夹
    QString file_input2=QFileDialog::getOpenFileName(this,"open outer calibration txt",QDir::currentPath(),
                                                       "txt(*.txt)");
    if(!QFile::exists(file_input2)){
        QMessageBox::warning(this,tr("read outer"),tr("read outer calibration fail!"));
        return;
    }
    Mat rotation_matrix = Mat::zeros(3, 3, CV_64FC1);//创建全零
    Mat tvecsMat = Mat::zeros(3,1, CV_64FC1);
    read_calibra_outer(file_input2.toStdString(),tvecsMat,rotation_matrix);
    //显示外参
    string outer_calibrate;
    s_temp.str("");
    s_temp << tvecsMat << endl;
    outer_calibrate="tvecsMat=\t\n"+s_temp.str()+"\t\nrotation_matrix\t\n";
    s_temp.str("");
    s_temp<<rotation_matrix<<endl;
    outer_calibrate=outer_calibrate+s_temp.str();
    QMessageBox::warning(this, tr("outer calibration information"), QString::fromStdString(outer_calibrate));


    //计算逆矩阵
    Mat inv_rotation;
    Mat inv_inner;
//    double di=calculate_distance(ROTATION_camera, TVECSmat, CAMERA_matrix, inv_rotation, inv_inner);
    double di=calculate_distance(rotation_matrix, tvecsMat, mat_inner, inv_rotation, inv_inner);
    d_end=di;
    if(image1_.size()==0){
        QMessageBox::warning(this,"system calibration","please capture line point!");
        return;
    }
//    //计算系统参数
//    //测试-----
//    theta1_sys=22.5;
//    theta2_sys=23.625;
//    //---------
    int n=image1_.size();
    int n1=image2_.size();
    if(n!=n1){
        QMessageBox::warning(this,tr("system calibration"),tr("points of two images are not equal!"));
        return;
    }
    for(int i=0;i<n;++i){
           double h;
           double l1;
           double l2;
           double u=image1_[i].pt.x;
           double v=image1_[i].pt.y;
           double u1=image2_[i].pt.x;
           double v1=image2_[i].pt.y;
           calculate_deltaH_L(inv_inner, inv_rotation, di, u, v, u1, v1, theta1_sys, theta2_sys, h, l1, l2);
           all_deltaH.push_back(h);
           all_L.push_back(l1);
           all_L.push_back(l2);
    }
    //计算h和l
    double temp_h=std::accumulate(all_deltaH.begin(),all_deltaH.end(),0);
    deltaH_end=temp_h/n;
    double temp_l=std::accumulate(all_L.begin(),all_L.end(),0);
    L_endl=temp_l/2/n;
    qDebug()<<"d = "<<d_end<<endl<<"delta h= "<<deltaH_end<<endl<<"l= "<<L_endl<<endl;

    //保存系统参数
    string save_dir = Cail_sys_result.toStdString() + "/system_calibration_result.txt";
    ofstream fout(save_dir,ios::out|ios::trunc);                       // 保存标定结果的文件
    //保存定标结果
    fout << "d" << endl;
    fout << d_end << endl << endl;
    fout << "deltaH"<<endl;
    fout << deltaH_end << endl << endl;
    fout << "L" << endl;
    fout << L_endl << endl << endl;
    fout.close();

    QMessageBox::warning(this,tr("systam calibration"),tr("system calibrate successful"));

}



//采集内参标定图像
void Widget::on_inner_pushButton_clicked()
{
    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
           //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    ++inner_image;
    QString add_str=QString::number(inner_image);
    QString str_temp=calib_dir_image+"/"+add_str+".bmp";
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
//    qDebug()<<strlen(c_temp)<<endl;
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();
    //解锁------------------------------------------
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_2"));
        return;
    }
    QString show_str="save inner BMP "+add_str+" successful";
    QMessageBox::warning(this,tr("save BMP"),show_str);

}


//采集外参标定图像
void Widget::on_outer_pushButton_clicked()
{
    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
        //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    ++pnp_image;
    QString add_str=QString::number(pnp_image);
    QString str_temp=pnp_dir+"/"+add_str+".bmp";
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
//    qDebug()<<strlen(c_temp)<<endl;
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();
    //解锁------------------------------------------
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_2"));
        return;
    }
    QString show_str="save outer BMP "+add_str+" successful";
    QMessageBox::warning(this,tr("save BMP"),show_str);

}



//采集系统标定图像,只需要采集两张即可
void Widget::on_save_for_sys_clicked()
{
    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
        //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    ++sys_image;
    QString add_str=QString::number(sys_image);
    QString str_temp=Cail_sys_image+"/"+add_str+".bmp";
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
//    qDebug()<<strlen(c_temp)<<endl;
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();
    //解锁------------------------------------------
    //确定theta1和theta2
    if(sys_image==1){
        double tem_num=ui->doubleSpinBox->value();
        theta1_sys=tem_num;
        qDebug()<<"theat1="<<theta1_sys<<endl;
    }
    if(sys_image==2){
        double tem_num=ui->doubleSpinBox->value();
        theta2_sys=tem_num;
        qDebug()<<"theat2="<<theta2_sys<<endl;
        ui->save_for_sys->setEnabled(false);//关闭采集按钮
    }
    //---------------------------------
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_2"));
        return;
    }
    QString show_str="save sys BMP "+add_str+" successful";
    QMessageBox::warning(this,tr("save BMP"),show_str);

}




//三维处理的采集图像按钮
void Widget::on_grab_image_clicked()
{
    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
           //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    ++test3d_num;
    QString add_str=QString::number(test3d_num);
    QString str_temp=image_docu_save+"/"+add_str+"_3d"+".bmp";
    testfile_name=str_temp;
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();
    //解锁------------------------------------------
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("save 3d test images"),tr("save BMP fail_2"));
        return;
    }
    QString show_str="save 3d test images"+add_str+" successful";
    QMessageBox::warning(this,tr("save 3d test images"),show_str);
}


//三维处理的提取中心线按钮
void Widget::on_line_cap_clicked()
{
    if(test3d_num==0){
        QMessageBox::warning(this,"get line points","please capture images!");
        return;
    }
    Mat image_first=imread(testfile_name.toStdString());
    Mat outline;
    Mat out;
    vector<KeyPoint> pt_0;
    pt_0.clear();
    vector<KeyPoint> ptcorr;
    ptcorr.clear();
    extern void ExtraLine(Mat src_input, Mat &out_line,Mat &output,
                          vector<KeyPoint> &Pt_out, vector<KeyPoint> &Pt_corr_out);
    ExtraLine(image_first,outline,out,pt_0,ptcorr);
    test_3D_point=ptcorr;
    qDebug()<<"pt1 size="<<ptcorr.size()<<endl;
    QString add_str=QString::number(test3d_num);
    QString str_temp=image_docu_save+"/"+add_str+"_3d_line"+".bmp";
    imwrite(str_temp.toStdString(),out);
    QMessageBox::warning(this,"extra line","finish get line points");
//    ui->statusbar->showMessage("finish get 3d points",3000);

}


//三维处理按钮
void Widget::on_three_point_clicked()
{
    void calculate_3dPoints(Mat &camera_matrix, Mat &rotation_inverse, double distance1, double deltaH,
        double systemL, double theta, cv::Point2d src_point, cv::Point3d &out_point);
    if(flag_input_inner&&flag_input_outer&&flag_input_system){
        if(test_3D_point.size()==0){
            QMessageBox::warning(this,"calculate 3d points","please get laser points firstly!");
            return;
        }
        //计算三维点
        theta_first=ui->doubleSpinBox->value();//获取角度
        string save_dir = image_docu_save.toStdString() + "/Calculate3D_Points.txt";
        ofstream fout(save_dir,ios::out|ios::trunc);                       // 保存标定结果的文件
        //保存定标结果
        int n=test_3D_point.size();
        for(int i=0; i<n;++i){
            KeyPoint temp=test_3D_point[i];
            Point2d temp1=temp.pt;
            Point3d ret;
            calculate_3dPoints(CAMERA_matrix,ROTATION_camera,d_end,deltaH_end,L_endl,theta_first,
                               temp1,ret);
            //一般txt点云数据是ascii，以空格或逗号隔开
            fout<<ret.x<<","<<ret.y<<","<<ret.z<<endl;
        }
        fout.close();
        QMessageBox::warning(this,"calculate 3d points","finish calculate points!");
    }else{
        QMessageBox::warning(this,"3D reconstruction","please input params!");
    }

}

//定时器函数
void Widget::handleTimeout_capture(){
    //采集图像
    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
           //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    QString add_str=QString::number(cal_num);//图片个数
    QString str_temp=three_dimension_save+"/3D_"+add_str+".bmp";
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();
    //解锁------------------------------------------
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_2"));
        return;
    }else{
        //发送运动指令
        char send_step=0xBB;
        m_serialPort.write(&send_step);
        //成功了就个数+1，个数从0开始算起
        ++cal_num;
        //加入队列
        //加锁
        m_file_push_que.lock();
        capture_file.push(str_temp);
        //解锁
        m_file_push_que.unlock();
    }
}


//结束处理信息提示
void Widget::finish_show_cal(){
//    qDebug() << __FUNCTION__<< QThread::currentThreadId();//输出函数名+线程ID
    QMessageBox::warning(this,"Calculate 3D Points","finish calculate 3D points!");
    ui->statusbar->showMessage("finish calculate!",3000);
}

//计算尺寸子线程
void Widget::calculate_thread(){
    //子线程不能调用qmessagebox，准确来说是不能传入this
    string save_dir = three_dimension_result.toStdString() + "/Calculate3D_Points.txt";
    ofstream fout(save_dir,ios::out|ios::trunc);                       // 保存标定结果的文件
    while(start_cal_thread){
        //非空的话就一直算
        while(!capture_file.empty()){
            //取一张图
            m_file_push_que.lock();
            QString input_laser=capture_file.front();
            capture_file.pop();
            m_file_push_que.unlock();
            //计算激光点
            Mat image_first=imread(input_laser.toStdString());
            Mat outline;
            Mat out;
            vector<KeyPoint> pt_0;
            pt_0.clear();
            vector<KeyPoint> ptcorr;
            ptcorr.clear();
            extern void ExtraLine(Mat src_input, Mat &out_line,Mat &output,
                                  vector<KeyPoint> &Pt_out, vector<KeyPoint> &Pt_corr_out);
            ExtraLine(image_first,outline,out,pt_0,ptcorr);
            QString add_str=QString::number(cal_save_num);//图片个数
            QString save_laser = three_dimension_result + "/"+add_str+".bmp";
            imwrite(save_laser.toStdString(),out);
            //计算三维坐标
            double theta_now=theta_for3d+step_pluse*cal_save_num;
            int n=ptcorr.size();
            for(int i=0; i<n;++i){
                KeyPoint temp=ptcorr[i];
                Point2d temp1=temp.pt;
                Point3d ret;
                calculate_3dPoints(CAMERA_matrix,ROTATION_camera,d_end,deltaH_end,L_endl,
                                   theta_now,temp1,ret);
                //一般txt点云数据是ascii，以空格或逗号隔开
                fout<<ret.x<<","<<ret.y<<","<<ret.z<<endl;
            }
            ++cal_save_num;
        }
        QThread::msleep(10);
    }
    fout.close();
    qDebug()<<"thread finish"<<endl;
    //发出完成的信号
    emit finish_calc();

//    //测试代码-------------
//    qDebug() << __FUNCTION__<< QThread::currentThreadId();//输出函数名+线程ID
//    while(start_cal_thread){
//        //非空的话就一直算
//        qDebug()<<"child thread"<<endl;
//        QThread::msleep(250);
//        qDebug() << __FUNCTION__<< QThread::currentThreadId();//输出函数名+线程ID
//    }
//    qDebug()<<"thread finish"<<endl;
//    //发出完成的信号
//    emit finish_calc();
//    //--------------------
}



//自动处理按钮
void Widget::on_auto_three_clicked()
{
    QMessageBox::warning(this,"auto deal",tr("请确认激光角度和电机方向已调好confirm!!!"));
    //首先确定输入参数
    if(!(flag_input_inner&&flag_input_outer&&flag_input_system)){
        QMessageBox::warning(this,"auto deal","please input params!");
        return;
    }
    //获取角度
    theta_for3d=ui->doubleSpinBox->value();
    qDebug()<<"theta now="<<theta_for3d<<endl;
    //清空队列
    while(!capture_file.empty()){
        capture_file.pop();
    }
    cal_num=0;
    cal_save_num=0;
    //打开定时器
    start_timer=true;
    m_capture_timer->start(1800);//1s
    //打开子线程
    start_cal_thread=true;
    QFuture<void> f_calc =QtConcurrent::run(this, &Widget::calculate_thread);
    //标识状态
    ui->statusbar->showMessage("Being calclulating...");
}


//结束处理按钮
void Widget::on_auto_end_clicked()
{
    //关闭定时器
    start_timer=false;
    m_capture_timer->stop();
    //关闭子线程
    start_cal_thread=false;
}




// 电机归零按钮
void Widget::on_motor_Zero_clicked()
{
    char send_zero=0xDD;
    m_serialPort.write(&send_zero);
    QByteArray temp = m_serialPort.readAll();
    if(temp.size()==0){
        QMessageBox::warning(this,"turn zero","fail get location!");
        return;
    }
    int length=temp.size();
    if(temp[0]!='A' && temp[length-1]!='C'){
        QMessageBox::warning(this,"turn zero","fail get location2!");
        return;
    }
    int pred=1;
    QByteArray loca;
    QByteArray dir;
    for(int i=pred;i<=length;++i){
        if(temp[i]!='B'){
            loca.append(temp[i]);
        }else {
            pred=i;
            break;
        }
    }
    for(int i=pred+1;i<=length;++i){
        if(temp[i]!='C'){
            dir.append(temp[i]);
        }else {
            pred=i;
            break;
        }
    }
    long location=loca.toLong();
    int direction=dir.toInt();
    if(location==0){
        qDebug()<<"already zero"<<endl;
        QMessageBox::warning(this,"move zero","already zero!");
        return;
    }
    if(location>0){
        if(direction==0){
            send_zero=0xAD;
            m_serialPort.write(&send_zero);
        }
        QThread::msleep(50);
        send_zero=0xBB;
        for(int i=0;i<location;++i){
            m_serialPort.write(&send_zero);
            QThread::msleep(10);
        }
        QThread::msleep(50);
        send_zero=0xAD;
        m_serialPort.write(&send_zero);
    }else if(location<0){
        if(direction==1){
            send_zero=0xAD;
            m_serialPort.write(&send_zero);
        }
        QThread::msleep(50);
        send_zero=0xBB;
        for(int i=0;i<abs(location);++i){
            m_serialPort.write(&send_zero);
            QThread::msleep(10);
        }

    }
    QThread::msleep(100);
    QMessageBox::warning(this,"move zero","finish move zero!");
    ui->doubleSpinBox->setValue(0);
    ui->mov_motor->setEnabled(false);
    ui->motor_Zero->setEnabled(false);

}




//电机状态按钮
void Widget::on_motor_condition_clicked()
{
    char send_zero=0xDD;
    m_serialPort.write(&send_zero);
    QByteArray temp = m_serialPort.readAll();
    if(temp.size()==0){
        QMessageBox::warning(this,"turn zero","fail get location!");
        return;
    }
    int length=temp.size();
    if(temp[0]!='A' && temp[length-1]!='C'){
        QMessageBox::warning(this,"turn zero","fail get location2!");
        return;
    }
    int pred=1;
    QByteArray loca;
    QByteArray dir;
    for(int i=pred;i<=length;++i){
        if(temp[i]!='B'){
            loca.append(temp[i]);
        }else {
            pred=i;
            break;
        }
    }
    for(int i=pred+1;i<=length;++i){
        if(temp[i]!='C'){
            dir.append(temp[i]);
        }else {
            pred=i;
            break;
        }
    }
    long location=loca.toLong();
    int direction=dir.toInt();
    QString show="steps="+loca+"; diarction="+dir;
    QMessageBox::warning(this,"motor condition",show);
    flag_pluse=direction;
    send_pluse_num=location;
    ui->mov_motor->setEnabled(true);
    ui->motor_Zero->setEnabled(true);
}



//采集棋盘格原图
void Widget::on_src_capture_clicked()
{
    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
        //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    QString add_str="src";
    QString str_temp=Cail_sys_image+"/"+add_str+".bmp";
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();

    QMessageBox::warning(this,"save src chess","save chessboard successfully!");
    ui->src_capture->setEnabled(false);
    flag_srcChess=1;

}


//输入内参按钮
void Widget::on_input_inner_cali_clicked()
{
    //extern
    bool read_calibra(string fileName, Mat &mat_inner, Mat &dist);
    //选取文件，初始化为debug文件夹
    QString file_input=QFileDialog::getOpenFileName(this,"open calibration txt",QDir::currentPath(),
                                                       "txt(*.txt)");
    if(!QFile::exists(file_input)){
        QMessageBox::warning(this,tr("read inner"),tr("read inner fail!"));
        return;
    }
    Mat mat_inner = Mat::zeros(3, 3, CV_64FC1);//创建全零
    Mat dist = Mat::zeros(5, 1, CV_64FC1);//5行一列
    read_calibra(file_input.toStdString(),mat_inner,dist);
    Mat inver_inner;
    cv::invert(mat_inner, inver_inner, DECOMP_LU);
    CAMERA_matrix=inver_inner.clone();
    DISTCoeffs=dist.clone();
    flag_input_inner=true;
    //显示标定信息
    stringstream s_temp;
    s_temp << mat_inner << endl;
    string output_cali;
    output_cali="cameraMatrix\t\n"+s_temp.str()+"\t\ndistCoeffs\t\n";
    s_temp.str("");
    s_temp<<dist<<endl;
    output_cali=output_cali+s_temp.str();
    QMessageBox::warning(this, tr("calibration information"), QString::fromStdString(output_cali));

}

//输入外参按钮
void Widget::on_input_outter_cali_clicked()
{
    //读取外参
    //选取文件，初始化为debug文件夹
    QString file_input2=QFileDialog::getOpenFileName(this,"open outer calibration txt",QDir::currentPath(),
                                                       "txt(*.txt)");
    if(!QFile::exists(file_input2)){
        QMessageBox::warning(this,tr("read outer"),tr("read outer calibration fail!"));
        return;
    }
    Mat rotation_matrix = Mat::zeros(3, 3, CV_64FC1);//创建全零
    Mat tvecsMat = Mat::zeros(3,1, CV_64FC1);
    read_calibra_outer(file_input2.toStdString(),tvecsMat,rotation_matrix);
    Mat inv_rota;
    cv::invert(rotation_matrix, inv_rota, DECOMP_LU);
    ROTATION_camera=inv_rota.clone();
    TVECSmat=tvecsMat.clone();
    flag_input_outer=true;

    //显示外参
    stringstream s_temp;
    string outer_calibrate;
    s_temp.str("");
    s_temp << tvecsMat << endl;
    outer_calibrate="tvecsMat=\t\n"+s_temp.str()+"\t\nrotation_matrix\t\n";
    s_temp.str("");
    s_temp<<rotation_matrix<<endl;
    outer_calibrate=outer_calibrate+s_temp.str();
    QMessageBox::warning(this, tr("outer calibration information"), QString::fromStdString(outer_calibrate));

}


//输入系统参数按钮
void Widget::on_input_sys_clicked()
{
    //选取文件，初始化为debug文件夹
    bool read_system_fun(string fileName, double &d, double &deltaH, double &L);
    QString file_input2=QFileDialog::getOpenFileName(this,"open outer calibration txt",QDir::currentPath(),
                                                       "txt(*.txt)");
    if(!QFile::exists(file_input2)){
        QMessageBox::warning(this,tr("read system"),tr("read system calibration fail!"));
        return;
    }

    double d1,d2,d3;
    read_system_fun(file_input2.toStdString(),d1,d2,d3);
    d_end=d1;
    deltaH_end=d2;
    L_endl=d3;
    qDebug()<<d1<<endl<<d2<<endl<<d3<<endl;
    flag_input_system=true;

    //显示外参
    stringstream s_temp;
    string outer_calibrate;
    s_temp.str("");
    s_temp << d1 << endl;
    outer_calibrate="d=\t\n"+s_temp.str()+"\t\ndeltaH=\t\n";
    s_temp.str("");
    s_temp<<d2<<endl;
    outer_calibrate=outer_calibrate+s_temp.str()+"\t\nL=\t\n";
    s_temp.str("");
    s_temp<<d3<<endl;
    outer_calibrate=outer_calibrate+s_temp.str();
    QMessageBox::warning(this, tr("system calibration information"), QString::fromStdString(outer_calibrate));

}



//标定d参数 -------------------------------
void Widget::on_cap_image_D_clicked()
{

    MV_SAVE_IMG_TO_FILE_PARAM stSaveFileParam;
    memset(&stSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
    //加锁-----------------------------------------
    m_mutex.lock();
    if (m_pSaveImageBuf == NULL || m_stImageInfo.enPixelType == 0)
    {
        //保存失败
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_1"));
        return;
    }
    stSaveFileParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save
    stSaveFileParam.enPixelType = m_stImageInfo.enPixelType;  // ch:相机对应的像素格式 | en:Camera pixel type
    stSaveFileParam.nWidth      = m_stImageInfo.nWidth;         // ch:相机对应的宽 | en:Width
    stSaveFileParam.nHeight     = m_stImageInfo.nHeight;          // ch:相机对应的高 | en:Height
    stSaveFileParam.nDataLen    = m_stImageInfo.nFrameLen;
    stSaveFileParam.pData       = m_pSaveImageBuf;
    stSaveFileParam.iMethodValue = 0;
    //保存路径
    ++cal_D_num;
    QString add_str=QString::number(cal_D_num);
    QString str_temp=image_docu_save+"/"+add_str+"_calD.bmp";
    caliD_image_path.push_back(str_temp);
    QByteArray str_2=str_temp.toLatin1();
    char *  c_temp = str_2.data();
//    qDebug()<<strlen(c_temp)<<endl;
    snprintf(stSaveFileParam.pImagePath, 256, "%s" ,c_temp);
    int nRet = m_pcMyCamera->SaveImageToFile(&stSaveFileParam);
    m_mutex.unlock();
    //解锁------------------------------------------
    if (MV_OK != nRet)
    {
        QMessageBox::warning(this,tr("save BMP"),tr("save BMP fail_2"));
        return;
    }
    QString show_str="save calD BMP "+add_str+" successful";
    QMessageBox::warning(this,tr("save BMP"),show_str);
}

void Widget::on_cali_D_clicked()
{
    bool cali_outer_R_T_for_D(string FileName, Size board_size, Size square_size,
        Mat &cameraMatrix, Mat &distCoeffs, Mat &rvecsMat, Mat &tvecsMat, Mat &rotation_matrix);
    int n_image=caliD_image_path.size();
    if(n_image<=0){
        ui->statusbar->showMessage("please capture images!",3000);
        caliD_image_path.empty();
        return;
    }
    //读取参数
    //选取文件，初始化为debug文件夹
    QString file_input=QFileDialog::getOpenFileName(this,"open calibration txt",QDir::currentPath(),
                                                       "txt(*.txt)");
    if(!QFile::exists(file_input)){
        QMessageBox::warning(this,tr("read inner"),tr("read inner fail!"));
        return;
    }
    Mat mat_inner = Mat::zeros(3, 3, CV_64FC1);//创建全零
    Mat dist = Mat::zeros(5, 1, CV_64FC1);//5行一列
    read_calibra(file_input.toStdString(),mat_inner,dist);

    //显示标定信息
    stringstream s_temp;
    s_temp << mat_inner << endl;
    string output_cali;
    output_cali="cameraMatrix\t\n"+s_temp.str()+"\t\ndistCoeffs\t\n";
    s_temp.str("");
    s_temp<<dist<<endl;
    output_cali=output_cali+s_temp.str();
    QMessageBox::warning(this, tr("calibration information"), QString::fromStdString(output_cali));
    Size board_size = Size(11, 8);                         // 标定板上每行、列的角点数
    Size2d square_size = Size2d(1.5, 1.5);                       // 实际测量得到的标定板上每个棋盘格的物理尺寸，单位mm
    Mat rvecsMat;                                          // 存放所有图像的旋转向量，每一副图像的旋转向量为一个mat
    Mat tvecsMat;                                          // 存放所有图像的平移向量，每一副图像的平移向量为一个mat
    Mat rotation_res;
    bool ack1;
    int ok_num=0;
    double sum=0;
    for(int i=0;i<n_image;++i){
        QString temp_path=caliD_image_path[i];
        ack1=cali_outer_R_T_for_D(temp_path.toStdString(),board_size,square_size,mat_inner,
                                  dist,rvecsMat,tvecsMat,rotation_res);
        if(ack1){
            ++ok_num;
            Mat inv_rotation;
            cv::invert(rotation_res, inv_rotation, DECOMP_LU);
            Mat out_inv = inv_rotation * tvecsMat;
            double d = out_inv.at<double>(2, 0);
            qDebug()<<d<<" "<<ok_num<<endl;
            sum+=d;
        }
    }
    //保存
    double d_last=sum/ok_num;
    qDebug()<<d_last<<"last"<<endl;
    ui->statusbar->showMessage("finish calc distance",3000);
    caliD_image_path.clear();
}

//标定d参数 end-----------------------------




























