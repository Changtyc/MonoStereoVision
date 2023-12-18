#ifndef CALIBRATE_METHOD_H
#define CALIBRATE_METHOD_H

#include <fstream>
#include <io.h>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


/*
功能：从文件夹读取特定类型所有文件
@param File_Directory 为文件夹目录
@param FileType 为需要查找的文件类型
@param FilesName 为存放文件名的容器
*/
bool getFilesName(string &File_Directory, string &FileType, vector<string>&FilesName);



/*功能：标定内参并输出标定结果
@param FilesName：图像文件路径向量，输入量
@param board_size：棋盘格的行列数，先列后行，如Size(9,6)，输入量
@param square_size：棋盘格的实际边长，以mm为单位，输入量
@param cameraMatrix：相机内参矩阵，输出量
@param distCoeffs：畸变系数矩阵，输出量
@param rvecsMat：旋转向量，输出量
@param tvecsMat：平移向量，输出量
@param File_Directory：保存目录，角点图片和输出的标定结果
*/
bool m_calibration(vector<string> FilesName, Size board_size, Size square_size,
    Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecsMat, vector<Mat> &tvecsMat, string File_Directory);



/*功能：读取标定文件
@param fileName：输入的文件路径
@param mat_inner：输出的内参矩阵
@param dist：输出的畸变系数矩阵
*/
bool read_calibra(string fileName, Mat &mat_inner, Mat &dist);

//读外参
bool read_calibra_outer(string fileName, Mat &tvecsMat, Mat &rotation_matrix);

//读系统参数
bool read_system_fun(string fileName, double &d, double &deltaH, double &L);



/*功能：通过内参和畸变求出外参，同时保存在指定目录
@param FileName：输入图片路径
@param board_size：棋盘格的行列数，先列后行，如Size(9,6)，输入量
@param square_size：棋盘格的实际边长，以mm为单位，输入量
@param cameraMatrix：相机内参矩阵，输入量
@param distCoeffs：畸变系数矩阵，输入量
@param revcsMat：输出旋转向量
@param tvecsMat：输出位移向量
@param rotation_matrix：输出旋转矩阵
@param save_Dir：输入的输出目录，注意用 / 分隔表示
*/
bool cali_outer_R_T(string FileName, Size board_size, Size square_size,
    Mat &cameraMatrix, Mat &distCoeffs, Mat &rvecsMat, Mat &tvecsMat, Mat &rotation_matrix, string save_Dir);





/*功能：标定系统参数d，以及输出旋转矩阵和内参矩阵的逆矩阵
@param rotation：输入旋转矩阵
@param tvecsMat：输入平移向量
@param inner_Matrix：输入相机内参矩阵
@param inv_rotation：输出旋转矩阵的逆矩阵
@param inv_inner_Matrix：输出内参矩阵的逆矩阵
*/
double calculate_distance(Mat &rotation, Mat &tvecsMat, Mat &inner_Matrix,
    Mat &inv_rotation, Mat &inv_inner_Matrix);



/*功能：标定系统参数h和L
@param inner_Matrix：输入相机内参矩阵的逆矩阵
@param rotation：输入旋转矩阵的逆矩阵
@param Zc：输入Zc，即基准面的d
@param u：输入像素的x，即列数
@param v：输入像素的y，即行数
@param u1：输入像素的x1，即列数
@param v1：输入像素的y1，即行数
@param theta：输入角度1，输入度数要在函数里面转换为弧度制
@param theta1：输入角度2，输入度数要在函数里面转换为弧度制
@param delta_h：输出h，引用
@param systemL：输出L，引用
@param systemL1：输出L1，引用
*/
void calculate_deltaH_L(Mat &inv_inner_Matrix, Mat &inv_rotation, double Zc, double u, double v, double u1, double v1,
    double theta, double theta1, double &delta_h, double &systemL, double &systemL1);





/*功能：计算三维坐标点，相对于理想相机坐标系
@param camera_matrix：输入相机内参的逆矩阵
@param rotation_inverse：输入旋转矩阵的逆矩阵
@param distance：输入系统参数d
@param deltaH：输入系统参数h
@param systemL：输入系统参数L
@param theta：输入电机角度theta
@parma src_point：输入二维图像点,x,y
@parma out_point：输出三维坐标点
*/
void calculate_3dPoints(Mat &camera_matrix, Mat &rotation_inverse, double distance1, double deltaH,
    double systemL, double theta, cv::Point2d src_point, cv::Point3d &out_point);



//中心线提取函数-------------
Mat Differentiation_x(float sigma, int med_wid);
Mat Differentiation_y(float sigma, int med_wid);
Mat Differentiation_xy(float sigma, int med_wid);
Mat Differentiation_xx(float sigma, int med_wid);
Mat Differentiation_yy(float sigma, int med_wid);
/*
@param src_input: 输入原图
@param out_line: 输出只有中心线的图，255灰度
@param output：输出绘制中心线的图，255灰度
@param Pt_out: 输出像素级中心点坐标
@param Pt_corr_out: 输出亚像素级中心点坐标
*/
void ExtraLine(Mat src_input, Mat &out_line,Mat &output,
               vector<KeyPoint> &Pt_out, vector<KeyPoint> &Pt_corr_out);

//end中心线提取函数-------------



//矩形区域提取函数
//面积函数
bool Contour_Area(vector<Point> contour1, vector<Point> contour2);
/*
@param src: 输入棋盘格原图
@param slice_out: 输出矩形边框坐标
@param ret_image: 输出的原图剪辑图像
*/
void FindRect(Mat &src, Rect &slice_out ,Mat &ret_image);

//end_矩形区域提取函数


//标定系统参数d
bool cali_outer_R_T_for_D(string FileName, Size board_size, Size square_size,
    Mat &cameraMatrix, Mat &distCoeffs, Mat &rvecsMat, Mat &tvecsMat, Mat &rotation_matrix);




#endif // CALIBRATE_METHOD_H







