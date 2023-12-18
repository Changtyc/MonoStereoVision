#include "calibrate_method.h"
#include <QtDebug>
using namespace std;
using namespace cv;

//和相机驱动结合后需要吧\\改为/
bool getFilesName(string &File_Directory, string &FileType, vector<string>&FilesName)
{
    string buffer = File_Directory + "/*" + FileType;

	_finddata_t c_file;   // 存放文件名的结构体

	intptr_t hFile;
	hFile = _findfirst(buffer.c_str(), &c_file);   //找第一个文件名字

	if (hFile == -1L)   // 检查文件夹目录下存在需要查找的文件
        return false;
	else
	{
		string fullFilePath;
		do
		{
			fullFilePath.clear();

			//名字
            fullFilePath = File_Directory + "/" + c_file.name;

			FilesName.push_back(fullFilePath);

		} while (_findnext(hFile, &c_file) == 0);  //如果找到下个文件的名字成功的话就返回0,否则返回-1  
		_findclose(hFile);
        return true;
	}
}



bool m_calibration(vector<string> FilesName, Size board_size, Size square_size,
                   Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecsMat,
                   vector<Mat> &tvecsMat, string File_Directory)
{
    // 写文件,每次写文件前无则创建，有则删除再创建
    string save_dir = File_Directory + "/caliberation_result.txt";
    ofstream fout(save_dir,ios::out|ios::trunc);                       // 保存标定结果的文件

    int image_count = 0;                                            // 失误图像数量
    int success_image = 0;                                          // 提取成功的图像数量
    Size image_size;                                                // 图像的尺寸

    vector<Point2f> image_points;                                   // 缓存每幅图像上检测到的角点
    vector<vector<Point2f>> image_points_seq;                       // 保存检测到的所有角点

    for (size_t i = 0; i < FilesName.size(); i++)
    {
        Mat imageInput = imread(FilesName[i]);
        if (0==i)  //读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;//列数即宽度
            image_size.height = imageInput.rows;
        }

        /* 提取角点 ，自适应阈值且归一化灰度*/
        bool ok = findChessboardCorners(imageInput, board_size, image_points,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
        //提取失败ok返回false
        if (!ok)
        {
            image_count++;
             continue;
        }
        else
        {
            //使用灰度图进行精细化
            success_image++;
            Mat view_gray;
            if (imageInput.channels() == 3) {
                cvtColor(imageInput, view_gray, COLOR_RGB2GRAY);
            }
            else if(imageInput.channels() == 1) {
                view_gray = imageInput.clone();
            }

            /* 亚像素精确化 */
            cv::cornerSubPix(view_gray, image_points, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01));

            image_points_seq.push_back(image_points);  //保存亚像素角点

            /* 在图像上显示角点位置 */
            Mat output_image = imageInput.clone();
            drawChessboardCorners(output_image, board_size, image_points, true);
            //保存，可通过查看返回值判断是否所有均提取成功
            string name_image;
            stringstream fmt;                       /* 或者使用 ostringstream */
            fmt << i+1 << ".bmp" ;
            // 获取最终需要的字符串
            name_image = fmt.str();
            name_image =  File_Directory+"/"+ name_image;
            imwrite(name_image, output_image);
        }
    }

    /*棋盘三维信息*/
    vector<vector<Point3f>> object_points_seq;                     // 保存标定板上角点的三维坐标
    vector<Point3f> object_points;
    //注意size是先列后行，坐标按opencv坐标系，而角点是从左至右，从上到下的一行一行排列的
    for (int i = 0; i < board_size.height; i++)
    {
        for (int j = 0; j < board_size.width; j++)
        {
            Point3f realPoint;
            /* 假设标定板放在世界坐标系中z=0的平面上 */
            realPoint.x = (float)j * square_size.width;
            realPoint.y = (float)i * square_size.height;
            realPoint.z = 0;
            object_points.push_back(realPoint);
        }
    }

    for (int t = 0; t < success_image; t++)
    {
        object_points_seq.push_back(object_points);
    }

    /* 运行标定函数 */
    double err_first = calibrateCamera(object_points_seq, image_points_seq, image_size,
        cameraMatrix, distCoeffs, rvecsMat, tvecsMat);

    //保存定标结果
    fout << "cameraMatrix" << endl;
    fout << cameraMatrix << endl << endl;
    fout << "distCoeffs"<<endl;
    fout << distCoeffs << endl << endl;
    fout << "error" << endl;
    fout << err_first << "像素 " << endl << endl;
    fout.close();
    if (0==image_count){
        return true;
    }else{
        return false;
    }
}



bool read_calibra(string fileName, Mat &mat_inner, Mat &dist)
{
    ifstream readFile(fileName);
    if (!readFile.is_open()) {
        return false;
    }
    string temp;
    int matrix = 0;
    int vec_dist = 0;
    while (!readFile.eof())
    {
        readFile >> temp; //遇到空格输出停止，空格后的内容无法输出，'\0'是截止符
        if (matrix != 0) {
            int row = (matrix-1) / 3;
            int col = (matrix-1) % 3;
            if (matrix == 1) {
                temp.erase(temp.begin());
                temp.erase(temp.end()-1,temp.end());
                stringstream tostd_str(temp);
                double temp1=0;
                tostd_str >>temp1;
                mat_inner.at<double>(row, col) = temp1;
                matrix++;
            }
            else if (matrix == 9) {
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1=0;
                tostd_str >> temp1;
                mat_inner.at<double>(row, col) = temp1;
                matrix=0;
            }
            else {
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1=0;
                tostd_str >> temp1;
                mat_inner.at<double>(row, col) = temp1;
                matrix++;
            }
        }
        if (temp == "cameraMatrix") {
            matrix ++;
        }
        if (vec_dist != 0) {
            if (vec_dist == 1) {
                temp.erase(temp.begin());
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1 = 0;
                tostd_str >> temp1;
                dist.at<double>(vec_dist - 1, 0) = temp1;
                vec_dist++;
            }
            else if (vec_dist == 5) {
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1 = 0;
                tostd_str >> temp1;
                dist.at<double>(vec_dist - 1, 0) = temp1;
                vec_dist = 0;
            }
            else {
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1 = 0;
                tostd_str >> temp1;
                dist.at<double>(vec_dist - 1, 0) = temp1;
                vec_dist++;
            }
        }
        if (temp == "distCoeffs") {
            vec_dist++;
        }

    }
    readFile.close();
    return true;
}


//读系统参数
bool read_system_fun(string fileName, double &d, double &deltaH, double &L){
    ifstream readFile(fileName);
    if (!readFile.is_open()) {
        return false;
    }
    string temp;
    int d_num=0;
    int delta_num=0;
    int l_num=0;
    while (!readFile.eof())
    {
        readFile >> temp; //遇到空格输出停止，空格后的内容无法输出，'\0'是截止符
        if (temp == "d") {
            d_num++;
            continue;
        }
        if (d_num!= 0) {
            d_num=0;
            stringstream tostd_str(temp);
            double temp1 = 0;
            tostd_str >> temp1;
            d=temp1;
        }
        if(temp=="deltaH"){
            delta_num++;
            continue;
        }
        if(delta_num!=0){
            delta_num=0;
            stringstream tostd_str(temp);
            double temp1 = 0;
            tostd_str >> temp1;
            deltaH=temp1;
        }
        if(temp=="L"){
            l_num++;
            continue;
        }
        if(l_num!=0){
            l_num=0;
            stringstream tostd_str(temp);
            double temp1 = 0;
            tostd_str >> temp1;
            L=temp1;
        }
    }
    readFile.close();
    return true;
}


//读外参
bool read_calibra_outer(string fileName, Mat &tvecsMat, Mat &rotation_matrix)
{
    ifstream readFile(fileName);
    if (!readFile.is_open()) {
        return false;
    }
    string temp;
    int matrix = 0;//旋转矩阵的计数
    int vec_dist = 0;//平移向量的计数
    while (!readFile.eof())
    {
        readFile >> temp; //遇到空格输出停止，空格后的内容无法输出，'\0'是截止符
        if (matrix != 0) {
            int row = (matrix-1) / 3;
            int col = (matrix-1) % 3;
            if (matrix == 1) {
                temp.erase(temp.begin());
                temp.erase(temp.end()-1,temp.end());
                stringstream tostd_str(temp);
                double temp1=0;
                tostd_str >>temp1;
                rotation_matrix.at<double>(row, col) = temp1;
                matrix++;
            }
            else if (matrix == 9) {
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1=0;
                tostd_str >> temp1;
                rotation_matrix.at<double>(row, col) = temp1;
                matrix=0;
            }
            else {
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1=0;
                tostd_str >> temp1;
                rotation_matrix.at<double>(row, col) = temp1;
                matrix++;
            }
        }
        if (temp == "rotation_matrix") {
            matrix ++;
        }
        if (vec_dist != 0) {
            if (vec_dist == 1) {
                temp.erase(temp.begin());
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1 = 0;
                tostd_str >> temp1;
                tvecsMat.at<double>(vec_dist - 1, 0) = temp1;
                vec_dist++;
            }
            else if (vec_dist == 3) {
                //要退出了
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1 = 0;
                tostd_str >> temp1;
                tvecsMat.at<double>(vec_dist - 1, 0) = temp1;
                vec_dist = 0;
            }
            else {
                temp.erase(temp.end() - 1, temp.end());
                stringstream tostd_str(temp);
                double temp1 = 0;
                tostd_str >> temp1;
                tvecsMat.at<double>(vec_dist - 1, 0) = temp1;
                vec_dist++;
            }
        }
        if (temp == "tvecsMat") {
            vec_dist++;
        }

    }
    readFile.close();
    return true;
}


bool cali_outer_R_T(string FileName, Size board_size, Size square_size,
    Mat &cameraMatrix, Mat &distCoeffs, Mat &rvecsMat, Mat &tvecsMat, Mat &rotation_matrix, string save_Dir)
{
    // 写文件,每次写文件前无则创建，有则删除再创建
    string save_doc = save_Dir + "/solvePnp_result.txt";
    ofstream fout(save_doc, ios::out | ios::trunc);                       // 保存标定结果的文件


    //先得到棋盘角点
    Size image_size;                                                // 图像的尺寸
    vector<Point2f> image_points;
    Mat imageInput = imread(FileName);
    image_size.width = imageInput.cols;//列数即宽度
    image_size.height = imageInput.rows;

    /* 提取角点 ，自适应阈值且归一化灰度*/
    bool ok = findChessboardCorners(imageInput, board_size, image_points,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
    if (!ok)
    {
        return false;
    }
    else
    {
        //使用灰度图进行精细化
        Mat view_gray;
        if (imageInput.channels() == 3) {
            cvtColor(imageInput, view_gray, COLOR_RGB2GRAY);
        }
        else if (imageInput.channels() == 1) {
            view_gray = imageInput.clone();
        }

        /* 亚像素精确化并保存 */
        cv::cornerSubPix(view_gray, image_points, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01));
        Mat output_image = imageInput.clone();
        drawChessboardCorners(output_image, board_size, image_points, true);
        string name_image;
        name_image = save_Dir + "/pnp_result.jpg";
        imwrite(name_image, output_image);

        //实际的世界坐标
        vector<Point3f> object_points;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = (float)j * square_size.width;
                realPoint.y = (float)i * square_size.height;
                realPoint.z = 0;
                object_points.push_back(realPoint);
            }
        }

        //求出外参
        bool pnp_Ack=cv::solvePnPRansac(object_points, image_points, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
        if (pnp_Ack) {
            /* 将旋转向量转换为相对应的旋转矩阵 */
            Rodrigues(rvecsMat, rotation_matrix);
            fout << "revecsMat" << endl;
            fout << rvecsMat << endl << endl;
            fout << "tvecsMat" << endl;
            fout << tvecsMat << endl << endl;
            fout << "rotation_matrix" << endl;
            fout << rotation_matrix << endl << endl;
            fout.close();
            return true;
        }
        else {
            return false;
        }
    }
}



bool cali_outer_R_T_for_D(string FileName, Size board_size, Size square_size,
    Mat &cameraMatrix, Mat &distCoeffs, Mat &rvecsMat, Mat &tvecsMat, Mat &rotation_matrix)
{
    //先得到棋盘角点
    Size image_size;                                                // 图像的尺寸
    vector<Point2f> image_points;
    Mat imageInput = imread(FileName);
    image_size.width = imageInput.cols;//列数即宽度
    image_size.height = imageInput.rows;

    /* 提取角点 ，自适应阈值且归一化灰度*/
    bool ok = findChessboardCorners(imageInput, board_size, image_points,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
    if (!ok)
    {
        return false;
    }
    else
    {
        //使用灰度图进行精细化
        Mat view_gray;
        if (imageInput.channels() == 3) {
            cvtColor(imageInput, view_gray, COLOR_RGB2GRAY);
        }
        else if (imageInput.channels() == 1) {
            view_gray = imageInput.clone();
        }

        /* 亚像素精确化并保存 */
        cv::cornerSubPix(view_gray, image_points, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01));

        //实际的世界坐标
        vector<Point3f> object_points;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = (float)j * square_size.width;
                realPoint.y = (float)i * square_size.height;
                realPoint.z = 0;
                object_points.push_back(realPoint);
            }
        }

        //求出外参
        bool pnp_Ack=cv::solvePnPRansac(object_points, image_points, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
        if (pnp_Ack) {
            /* 将旋转向量转换为相对应的旋转矩阵 */
            Rodrigues(rvecsMat, rotation_matrix);
            return true;
        }
        else {
            return false;
        }
    }
}






double calculate_distance(Mat &rotation, Mat &tvecsMat, Mat &inner_Matrix,
    Mat &inv_rotation, Mat &inv_inner_Matrix)
{
    cv::invert(rotation, inv_rotation, DECOMP_LU);
    cv::invert(inner_Matrix, inv_inner_Matrix, DECOMP_LU);
    Mat out_inv = inv_rotation * tvecsMat;
    double d = out_inv.at<double>(2, 0);
    return d;
}




void calculate_deltaH_L(Mat &inv_inner_Matrix, Mat &inv_rotation, double Zc, double u, double v, double u1, double v1,
    double theta, double theta1, double &delta_h, double &systemL, double &systemL1)
{
    //第一个点
    Mat image_m = Mat::ones(3, 1, CV_64FC1);//创建全1像素矩阵
    image_m.at<double>(0, 0) = u;
    image_m.at<double>(1, 0) = v;
    Mat out_xyz = inv_rotation * inv_inner_Matrix*image_m;
    double scale_m = Zc / out_xyz.at<double>(2, 0);
    Mat out_real;
    out_xyz.convertTo(out_real, CV_64FC1, scale_m, 0);
    //第二个点
    Mat image_m_1 = Mat::ones(3, 1, CV_64FC1);//创建全1像素矩阵
    image_m_1.at<double>(0, 0) = u1;
    image_m_1.at<double>(1, 0) = v1;
    Mat out_xyz_1 = inv_rotation * inv_inner_Matrix*image_m_1;
    double scale_m_1 = Zc / out_xyz_1.at<double>(2, 0);
    Mat out_real_1;
    out_xyz_1.convertTo(out_real_1, CV_64FC1, scale_m_1, 0);

    //求delta_h
    double temp1 = out_real.at<double>(0, 0);
    double temp2 = out_real_1.at<double>(0, 0);
    double temp1y = out_real.at<double>(1, 0);
    double temp2y = out_real_1.at<double>(1, 0);
    double temp1z = out_real.at<double>(2, 0);
    double temp2z = out_real_1.at<double>(2, 0);
    qDebug()<<"x1="<<temp1<<",x2="<<temp2<<endl;
    qDebug()<<"y1="<<temp1y<<",y2="<<temp2y<<endl;
    qDebug()<<"z1="<<temp1z<<",z2="<<temp2z<<endl;
    double t_theta1 = tan(CV_PI*theta / 180);
    double t_theta2 = tan(CV_PI*theta1 / 180);
    qDebug()<<"tan22.5="<<t_theta1<<endl;
    qDebug()<<"tan23.65="<<t_theta2<<endl;
    //21.12.17更改
    delta_h = (temp1 - temp2) / (t_theta2 - t_theta1) - Zc;

    //求L
    systemL = (Zc + delta_h)*t_theta1 + temp1;
    systemL1 = (Zc + delta_h)*t_theta2 + temp2;

    return;
}



void calculate_3dPoints(Mat &camera_matrix, Mat &rotation_inverse, double distance1, double deltaH,
    double systemL, double theta, cv::Point2d src_point, cv::Point3d &out_point)
{
    //计算实际三维空间归一化坐标
    Mat image_m = Mat::ones(3, 1, CV_64FC1);//创建全1像素矩阵
    image_m.at<double>(0, 0) = src_point.x;
    image_m.at<double>(1, 0) = src_point.y;
    Mat out_xyz = rotation_inverse * camera_matrix*image_m;
    double scale_m = out_xyz.at<double>(2, 0);
    Mat out_real;
    out_xyz.convertTo(out_real, CV_64FC1, scale_m, 0);

    //计算h
    double temp = tan(CV_PI*theta / 180);
    double temp1 = out_real.at<double>(2, 0);
    double h;
    h = distance1 - (systemL - deltaH * temp) / (temp + temp1);

    //计算实际坐标
    double Zc;
    Zc = distance1 - h;
    Mat out_real_point;
    out_real.convertTo(out_real_point, CV_64FC1, Zc, 0);
    out_point.x = out_real_point.at<double>(0, 0);
    out_point.y = out_real_point.at<double>(1, 0);
    out_point.z = out_real_point.at<double>(2, 0);
    return;
}



Mat Differentiation_x(float sigma, int med_wid) {
    Mat output(2 * med_wid + 1, 2 * med_wid + 1, CV_32FC1);
    //i为行y，j为列x
    for (int i = -med_wid; i <= med_wid; i++) {
        for (int j = -med_wid; j <= med_wid; j++) {
            float tem1 = powf(sigma, 4);//
            float tem_sigma = powf(sigma, 2);
            float tem2 = exp(-(i*i + j * j) / (2 * tem_sigma));
            output.at<float>(i + med_wid, j + med_wid) = (float)(-j / (2 * CV_PI*tem1)*tem2);
        }
    }
    return output;
}

Mat Differentiation_y(float sigma, int med_wid) {
    Mat output(2 * med_wid + 1, 2 * med_wid + 1, CV_32FC1);
    //i为行y，j为列x
    for (int i = -med_wid; i <= med_wid; i++) {
        for (int j = -med_wid; j <= med_wid; j++) {
            float tem1 = powf(sigma, 4);
            float tem_sigma = powf(sigma, 2);
            float tem2 = exp(-(i*i + j * j) / (2 * tem_sigma));
            output.at<float>(i + med_wid, j + med_wid) = (float)(-i / (2 * CV_PI*tem1)*tem2);
        }
    }
    return output;
}

Mat Differentiation_xy(float sigma, int med_wid) {
    Mat output(2 * med_wid + 1, 2 * med_wid + 1, CV_32FC1);
    //i为行y，j为列x
    for (int i = -med_wid; i <= med_wid; i++) {
        for (int j = -med_wid; j <= med_wid; j++) {
            float tem1 = powf(sigma, 6);
            float tem_sigma = powf(sigma, 2);
            float tem2 = exp(-(i*i + j * j) / (2 * tem_sigma));
            output.at<float>(i + med_wid, j + med_wid) = (float)(-i * j / (2 * CV_PI*tem1)*tem2);
        }
    }
    return output;
}

Mat Differentiation_xx(float sigma, int med_wid) {
    Mat output(2 * med_wid + 1, 2 * med_wid + 1, CV_32FC1);
    //i为行y，j为列x
    for (int i = -med_wid; i <= med_wid; i++) {
        for (int j = -med_wid; j <= med_wid; j++) {
            float tem1 = powf(sigma, 4);
            float tem_sigma = powf(sigma, 2);
            float tem2 = exp(-(i*i + j * j) / (2 * tem_sigma));
            float tem3 = 1 - j * j / tem_sigma;
            output.at<float>(i + med_wid, j + med_wid) = (float)(-1 / (2 * CV_PI*tem1)*tem2*tem3);
        }
    }
    return output;
}


Mat Differentiation_yy(float sigma, int med_wid) {
    Mat output(2 * med_wid + 1, 2 * med_wid + 1, CV_32FC1);
    //i为行y，j为列x
    for (int i = -med_wid; i <= med_wid; i++) {
        for (int j = -med_wid; j <= med_wid; j++) {
            float tem1 = powf(sigma, 4);
            float tem_sigma = powf(sigma, 2);
            float tem2 = exp(-(i*i + j * j) / (2 * tem_sigma));
            float tem3 = 1 - i * i / tem_sigma;
            output.at<float>(i + med_wid, j + med_wid) = (float)(-1 / (2 * CV_PI*tem1)*tem2*tem3);
        }
    }
    return output;
}


/*
@param src_input: 输入原图
@param out_line: 输出只有中心线的图，255灰度
@param output：输出绘制中心线的图，255灰度
@param Pt_out: 输出像素级中心点坐标
@param Pt_corr_out: 输出亚像素级中心点坐标
*/
void ExtraLine(Mat src_input, Mat &out_line,Mat &output, vector<KeyPoint> &Pt_out, vector<KeyPoint> &Pt_corr_out) {
    Mat src = src_input.clone();
    Mat src32;//以上两幅图都是RGB三通道

    Mat gray_32F;//32FC1,0-1
    Mat gray_8U;//8UC1,0-255
    cvtColor(src, gray_8U, COLOR_RGB2GRAY);
    src.convertTo(src32, CV_32F, 1.0 / 255);//将CV_8UC3类型转换成CV_32FC3类型,且归一化，第三个参数为比例因子
    cvtColor(src32, gray_32F, COLOR_RGB2GRAY);//归一化转换为灰度图

    //高斯滤波去噪
    Mat result_Gauss;
    GaussianBlur(gray_32F, result_Gauss, Size(5, 5), 3, 3);//xy方向标准差为3，边界不倒推
    Mat tem_gauss;
    result_Gauss.convertTo(tem_gauss, CV_8UC1, 255);//单通道路8U才能用大津法/三角法


    //大津法进行二值化
    Mat dst_otsu;
    double threshold_1;
    threshold_1 = threshold(tem_gauss, dst_otsu, 25, 255, 8);//大津法自动找阈值，25这个参数没什么用

    Mat ROI = tem_gauss.clone();
    Mat ROI_src = gray_8U.clone();


    //Steger算法求取中心线******************************
    float light_wid = 50;//假定宽度为8
    float sigma = float(light_wid / sqrt(3));
    int gauss_size = int(4 * ceil(sigma) + 1);//模板大小
    int gauss_med = int((gauss_size - 1) / 2); //高斯模板取值数
    Mat ROI_white = Mat::zeros(ROI.size(), CV_8UC1);//展示的图片，同时后期噪点的处理

    //一阶偏导数
    Mat m1, m2;
    //m1 = (Mat_<float>(1, 2) << -1, 1);  //x偏导
    //m2 = (Mat_<float>(2, 1) << -1, 1);  //y偏导
    m1 = Differentiation_x(sigma, gauss_med);//x偏导
    m2 = Differentiation_y(sigma, gauss_med);//y偏导

    Mat dx, dy;
    filter2D(ROI, dx, CV_32FC1, m1);//图像卷积
    filter2D(ROI, dy, CV_32FC1, m2);

    //二阶偏导数
    Mat m3, m4, m5;
    //m3 = (Mat_<float>(1, 3) << 1, -2, 1);   //二阶x偏导
    //m4 = (Mat_<float>(3, 1) << 1, -2, 1);   //二阶y偏导
    //m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);   //二阶xy偏导
    m3 = Differentiation_xx(sigma, gauss_med);   //二阶x偏导
    m4 = Differentiation_yy(sigma, gauss_med);   //二阶y偏导
    m5 = Differentiation_xy(sigma, gauss_med);   //二阶xy偏导

    Mat dxx, dyy, dxy;
    filter2D(ROI, dxx, CV_32FC1, m3);//图像卷积
    filter2D(ROI, dyy, CV_32FC1, m4);
    filter2D(ROI, dxy, CV_32FC1, m5);

    //hessian矩阵,向量Pt即为中心点集合
    float maxD = 0;//自定义负极大值,有关系
    float fmaxD;
    int imgcol = ROI.cols;
    int imgrow = ROI.rows;
    vector<KeyPoint> keyValues;//特征值/法向
    vector<KeyPoint> Pt;//中心点
    vector<KeyPoint> Pt_corr;//亚像素
    //先行后列，这个按列来扫描，j为行数y，i为列数x
    for (int i = gauss_med; i < (imgcol - gauss_med); i++)
    {
        for (int j = gauss_med; j < (imgrow - gauss_med); j++)
        {
            if (ROI.at<uchar>(j, i) > (uchar)(threshold_1))//阈值可改为自适应
            {
                Mat hessian(2, 2, CV_32FC1);
                hessian.at<float>(0, 0) = dxx.at<float>(j, i);
                hessian.at<float>(0, 1) = dxy.at<float>(j, i);
                hessian.at<float>(1, 0) = dxy.at<float>(j, i);
                hessian.at<float>(1, 1) = dyy.at<float>(j, i);

                Mat eValue;//特征值
                Mat eVectors;//特征向量
                eigen(hessian, eValue, eVectors);//求解海森矩阵特征向量

                double nx, ny;

                if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))  //求特征值最大时对应的特征向量
                {
                    fmaxD = eValue.at<float>(0, 0);
                    nx = eVectors.at<float>(0, 0);
                    ny = eVectors.at<float>(0, 1);
                }
                else
                {
                    fmaxD = eValue.at<float>(1, 0);
                    nx = eVectors.at<float>(1, 0);
                    ny = eVectors.at<float>(1, 1);
                }

                double t = -(-nx * dx.at<float>(j, i) + ny * dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j, i) +
                    2 * nx*ny*dxy.at<float>(j, i) + ny * ny*dyy.at<float>(j, i));
//                if (j >= 10 && j < 400 && i>1205 && i < 1235) {
//                    cout << fabs(t*nx) << endl;
//                    cout << fabs(t*ny) << endl;
//                    cout << fmaxD << endl;
//                    cout << i << " " << j << endl;
//                }

                if (fabs(t*nx) <= 0.5 && fabs(t*ny) <= 0.5 && fmaxD <= maxD)
                {
                    //像素级别
                    KeyPoint keypoint0;
                    keypoint0.pt.x = (float)i;
                    keypoint0.pt.y = (float)j;//先列后行，i为x，j为y
                    Pt.push_back(keypoint0);
                    Pt_out.push_back(keypoint0);

                    //验证hessia矩阵，保存特征值/法向
                    KeyPoint keypoint1;
                    keypoint1.pt.x = (float)nx;
                    keypoint1.pt.y = (float)ny;
                    keyValues.push_back(keypoint1);

                    //亚像素级别
                    KeyPoint keypoint2;
                    keypoint2.pt.x = (float)(i + t * nx);
                    keypoint2.pt.y = (float)(j + t * ny);
                    Pt_corr.push_back(keypoint2);
                    Pt_corr_out.push_back(keypoint2);
                }
            }
        }
    }
    //画出各中心点
    int m12=Pt.size();
    for (int k = 0; k < m12; k++)
    {
        ROI_white.at<uchar>(Pt[k].pt) = 255;
        ROI_src.at<uchar>(Pt[k].pt) = 0;
    }
    output = ROI_src.clone();
    out_line = ROI_white.clone();

}




//面积函数
bool Contour_Area(vector<Point> contour1, vector<Point> contour2)
{
    return contourArea(contour1) > contourArea(contour2);
}


/*
@param src: 输入棋盘格原图
@param slice_out: 输出矩形边框坐标
@param ret_image: 输出的原图剪辑图像
*/
void FindRect(Mat &src, Rect &slice_out ,Mat &ret_image) {
    extern bool Contour_Area(vector<Point> contour1, vector<Point> contour2);
    //换为灰度
    Mat grayImage;
    cvtColor(src, grayImage, COLOR_BGR2GRAY);

    //伽马变换
    Mat grayimg;
    grayimg.create(grayImage.size(), grayImage.type()); //创建一个大小类型相同的图像矩阵序列，也可以用clone()函数；
    int height = grayImage.rows;
    int width = grayImage.cols;
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            int gray = grayImage.at<uchar>(i, j);
            grayimg.at<uchar>(i, j) = pow(gray, 0.5);//将灰度值开方；
        }
    normalize(grayimg, grayimg, 0, 255, NORM_MINMAX);//归一化，将数据归一到0-255之间；

    grayImage = grayimg;

    //定义变量
    vector<vector<Point>>contours;
    vector<Vec4i>hierarchy;
    //双边滤波
    Mat bilater_image;
    bilateralFilter(grayImage, bilater_image, 21, 25, 25);

    //寻找边缘
    Mat canny_dst;
    Canny(bilater_image, canny_dst, 50, 150);

    //膨胀;
    Mat dila_dst;
    Mat structureElement = getStructuringElement(MORPH_RECT, Size(15, 15), Point(-1, -1)); //结构元
    dilate(canny_dst, dila_dst, structureElement, Point(-1, -1), 1);               //调用膨胀API

    //寻找轮廓
    findContours(dila_dst, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    sort(contours.begin(), contours.end(), Contour_Area);
    vector<vector<Point>> contours_second(contours.begin(), contours.begin() + 1);
    vector<Point> ret=contours_second[0];
//不排序了
//    for (auto &i : contours_second) {
//        //double length=arcLength(i, true);
//        vector<Point> out;
//        //20这个参数可变
//        approxPolyDP(i, out, 20, true);
//        if (out.size() == 4) {
//            ret = out;
//            break;
//        }
//    }
    //获得外接矩形
    Rect slice;
    slice.x=0;
    slice.y=0;
    slice.width=0;
    slice.height=0;
    Mat roi;
    if (!ret.empty()) {
        slice = boundingRect(ret);
        if(slice.y+50<slice.height-80){
            slice.y = slice.y + 50;
            slice.height = slice.height - 80;
        }
        src(slice).copyTo(roi);
    }
    slice_out = slice;
    ret_image = roi;
}






















































