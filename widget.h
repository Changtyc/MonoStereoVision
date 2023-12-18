#ifndef WIDGET_H
#define WIDGET_H

#include <QStatusBar>
#include <QMutex>
#include <QtConcurrent>
#include <QFuture>
#include <QWidget>
#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QDir>
#include <QSerialPort> //要在pro文件添加QT += serialport才有
#include <QSerialPortInfo>
#include <QThread>
#include <vector>
#include <opencv2/opencv.hpp>
#include <QTimer>
//#include "mycamerathread.h"
#include "MvCamera.h"
#include "calibrate_method.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();


signals:
    void finish_calc(); // 信号

private slots:
    void on_inner_cali_pushButton_clicked();

    void on_inner_matrix_pushButton_clicked();

    void on_setup_clicked();

    void on_motor_on_pushButton_clicked();

    void on_mov_motor_clicked();

    void on_find_openCamera_clicked();

    void on_param_set_clicked();

    void on_close_camera_clicked();

    void on_close_serial_clicked();

    void on_start_capture_clicked();

    void on_stop_capture_clicked();


    void on_save_bmp_clicked();

    void on_extract_line_clicked();

    void on_cali_sys_clicked();

    void on_inner_pushButton_clicked();

    void on_outer_pushButton_clicked();

    void on_save_for_sys_clicked();

    void on_grab_image_clicked();

    void on_line_cap_clicked();

    void on_three_point_clicked();

    void on_auto_three_clicked();

    void on_auto_end_clicked();

    void on_motor_Zero_clicked();

    void on_motor_condition_clicked();

    void on_src_capture_clicked();

    void on_input_inner_cali_clicked();

    void on_input_outter_cali_clicked();

    void on_input_sys_clicked();

    void handleTimeout_capture();//采集图像子线程
    void finish_show_cal();//结束处理提示函数

    void on_cap_image_D_clicked();

    void on_cali_D_clicked();

private:
    QSerialPort m_serialPort;//串口通信
    double send_pluse_num;//发送脉冲次数，分频为12800
    double tem_angle;//上一次的角度值
    int flag_pluse;//判断脉冲方向标志
    double step_pluse;//步长
    QMutex m_mutex;//保存图片的互斥锁

    //系统参数标定
    int flag_srcChess=0;//采集系统图标志
    vector<KeyPoint> image1_;//激光点组1
    vector<KeyPoint> image2_;//激光点组2
    vector<double> all_deltaH;//h的vector组
    vector<double> all_L;//L的vector组
    double theta1_sys;
    double theta2_sys;
    double d_end;
    double deltaH_end;
    double L_endl;


public:
    //文件目录
    QString calib_dir_image;//内参标定图片
    QString pnp_dir;//外参标定图片
    QString calib_result;//内参标定结果
    QString pnp_result;//外参标定结果
    QString Cail_sys_image;//系统参数图片
    QString Cail_sys_result;//系统参数标定结果
    QString image_docu_save;//保存图片按钮的路径,只是测试
    QString three_dimension_save;//三维处理图片保存目录
    QString three_dimension_result;//三维处理结果目录

    //图片数目
    uint image_save;
    uint inner_image;
    uint pnp_image;
    uint sys_image;

    //相机标定系统参数
    cv::Mat CAMERA_matrix;//内参矩阵的逆矩阵
    cv::Mat DISTCoeffs;
    cv::Mat ROTATION_camera;//旋转矩阵的逆矩阵
    cv::Mat TVECSmat;

    //三维测量输入标志
    bool flag_input_inner=false;
    bool flag_input_outer=false;
    bool flag_input_system=false;


    //三维重建调试参数
    vector<KeyPoint> test_3D_point;
    uint test3d_num=0;
    QString testfile_name;
    double theta_first=0;

    //自动处理标志
    double theta_for3d=0;
    int cal_num=0;//计算次数，用来计算角度以计算三维点云
    int cal_save_num=0;//保存激光线图
    bool start_timer=false;
    bool start_cal_thread=false;//开启线程处理函数
    void calculate_thread();//计算子线程函数
    queue<QString> capture_file;//文件路径队列
    QMutex m_file_push_que;//加入文件队列的互斥锁
//    queue<vector<KeyPoint>> laser_Points;//激光点云队列
//    QMutex m_point_push_que;//加入队列的互斥锁
    QTimer *m_capture_timer;

    //标定d参数标志
    int cal_D_num=0;
    vector<QString> caliD_image_path;



    //相机sdk变量
public:
    // ch:设置曝光时间 | en:Set Exposure Time
    int SetExposureTime();
    //设置增益
    int SetGain();
    // ch:设置帧率 | en:Set Frame Rate
    int SetFrameRate();

    //采集图像的线程和标志
    bool  m_bThreadState;//采集图像的线程标志
    void  GrabThreadProcess();//图像采集显示线程



private:
    //相机设备接口
    CMvCamera*              m_pcMyCamera; // ch:CMyCamera封装了常用接口
    MV_CC_DEVICE_INFO_LIST  m_stDevList;
    //参数设置：曝光、增益、帧率
    double                  m_dExposureEdit;
    double                  m_dGainEdit;
    double                  m_dFrameRateEdit;

    //标志
    bool                    m_bOpenDevice;//打开相机标志
    bool                    m_bStartGrabbing;//采集按钮按下标志

    //采集图像
    unsigned char*          m_pSaveImageBuf; //保存的图像数据
    unsigned int            m_nSaveImageBufSize;//图像长度
    MV_FRAME_OUT_INFO_EX    m_stImageInfo;//图像的拓展信息

private:
    Ui::Widget *ui;
};
#endif // WIDGET_H





















