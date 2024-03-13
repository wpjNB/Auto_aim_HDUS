#include "AngleSolver.h"

void AngleSolver::Init(const std::string &paramPath, float camBiasZ, float camBiasY, float gravity)
{
    // 大装甲板实际参数
    BigArmorPoint3D = {cv::Point3f(-BIG_ARMOR_LEN / 2.0, BIG_ARMOR_WID / 2.0, 0.0),
                       cv::Point3f(-BIG_ARMOR_LEN / 2.0, -BIG_ARMOR_WID / 2.0, 0.0),
                       cv::Point3f(BIG_ARMOR_LEN / 2.0, -BIG_ARMOR_WID / 2.0, 0.0),
                       cv::Point3f(BIG_ARMOR_LEN / 2.0, BIG_ARMOR_WID / 2.0, 0.0)};

    // 小装甲板实际参数
    SmallArmorPoint3D = {cv::Point3f(-SMALL_ARMOR_LEN / 2.0, SMALL_ARMOR_WID / 2.0, 0.0),
                         cv::Point3f(-SMALL_ARMOR_LEN / 2.0, -SMALL_ARMOR_WID / 2.0, 0.0),
                         cv::Point3f(SMALL_ARMOR_LEN / 2.0, -SMALL_ARMOR_WID / 2.0, 0.0),
                         cv::Point3f(SMALL_ARMOR_LEN / 2.0, SMALL_ARMOR_WID / 2.0, 0.0)};

    cv::FileStorage fsread;
    fsread.open(paramPath, cv::FileStorage::READ);
    if (!fsread.isOpened())
    {
        cout << "Failed to open xml";
        return;
    }

    fsread["camera_matrix"] >> camera_matrix;
    fsread["distortion_coefficients"] >> distortion_coefficients;
    fsread.release();
    // 相机偏移
    _cam_bias_z = camBiasZ;
    _cam_bias_y = camBiasY;
    _gravity = gravity;
}

void AngleSolver::solve_angle(rm_auto_aim::Armor &TargetArmor)
{

    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // 旋转矩阵
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // 平移矩阵
    vector<cv::Point2f> D2 = vector<cv::Point2f>{
        TargetArmor.vertex[0],
        TargetArmor.vertex[1],
        TargetArmor.vertex[2],
        TargetArmor.vertex[3]};
    switch (TargetArmor.armor_type)
    {
    case rm_auto_aim::LARGE:
        solvePnP(BigArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec);
        break;
    case rm_auto_aim::SMALL:
        solvePnP(SmallArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec);
        break;
    default:
        break;
    }

    double x_pos = tVec.at<double>(0, 0);
    double y_pos = tVec.at<double>(1, 0);
    double z_pos = tVec.at<double>(2, 0);
    // 选用拟合曲线
    this->dis = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
    if (dis > 5000)
    {
        double fx = this->camera_matrix.at<double>(0, 0);
        double fy = this->camera_matrix.at<double>(1, 1);
        double cx = this->camera_matrix.at<double>(0, 2);
        double cy = this->camera_matrix.at<double>(1, 2);
        cv::Point2f pnt;
        vector<cv::Point2f> in;
        vector<cv::Point2f> out;

        in.push_back(TargetArmor.center);
        // 对像素点去畸变
        undistortPoints(in, out, this->camera_matrix, this->camera_matrix, cv::noArray(), this->camera_matrix);
        pnt = out.front();

        // 去畸变后的比值
        double rxNew = (pnt.x - cx) / fx;
        double ryNew = (pnt.y - cy) / fy;

        yaw = atan(rxNew) / CV_PI * 180;
        pitch = -atan(ryNew) / CV_PI * 180;
    }
    else
    {
        double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
        double tan_yaw = x_pos / z_pos;
        pitch = atan(tan_pitch) * 180 / CV_PI;
        yaw = -atan(tan_yaw) * 180 / CV_PI;
    }
}
// 从这个接口获取角度
void AngleSolver::GetAngle(float &pitch, float &yaw, float &distance)
{
    pitch = this->pitch;
    yaw = this->yaw;
    distance = this->dis;
}