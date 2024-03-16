#include "detector.h"
using namespace cv;
namespace rm_auto_aim
{
    Detector::Detector()
    {
        auto model_path = "/home/wpj/RM_Vision_code_US/auto_aim_HDUS/ArmorDetector/model/mlp.onnx";
        auto label_path = "/home/wpj/RM_Vision_code_US/auto_aim_HDUS/ArmorDetector/model/label.txt";
        double threshold = 0.7;
        this->classifier = std::make_unique<rm_auto_aim::NumberClassifier>(model_path, label_path, threshold);
    }
    void Detector::run(Mat &img, int color_label, Armor &TargetArmor)
    {
#ifdef USING_ROI
        ImageByROI(img);
#endif
        Mat ShowDebug = img.clone();
        detect_for_target(img, color_label, TargetArmor);
#ifdef UsingShowImg
        drawResults(ShowDebug);
        imshow("Debug", ShowDebug);
#endif
    }

    void Detector::ImageByROI(Mat &img)
    {
        static Rect imgBound = Rect(0, 0, img.cols, img.rows);
        if (ArmorState == ARMOR_FOUND)
        {
            imgBound = Rect(img.cols / 4, img.rows / 4, img.cols / 4 * 3, img.rows / 4 * 3);
        }
        else if (ArmorState == ARMOR_NOT_FOUND)
        {
            static int lost_cnt;
            if (++lost_cnt > 5) // 装甲板丢失5振
            {
                imgBound = Rect(0, 0, img.cols, img.rows);
                lost_cnt = 0;
            }
        }
        img = img(imgBound).clone();
    }

    int Detector::detect_for_target(const Mat &frame, int color_label, Armor &TargetArmor)
    {
        detector(frame, color_label);
        //        // 根据confidence排序，并选择最大的
        //        std::sort(armors.begin(), armors.end(), [](const Armor &armor1, const Armor &armor2)
        //        {
        //            return armor1.confidence > armor2.confidence;
        //        });
        // 存在装甲板
        if (True_armors.size() > 0)
        {
            // 选择距离中心最近的装甲板
            std::sort(True_armors.begin(), True_armors.end(), [&frame](const Armor &armor1, const Armor &armor2)
                      {
            cv::Point2f _center((frame.cols - 1) / 2.0, (frame.rows - 1) / 2.0);
            return cv::norm(armor1.center-_center)>cv::norm(armor2.center-_center); });

            TargetArmor = True_armors[0];
        }
        return 1;
    }
    void Detector::detector(const cv::Mat &input, int enemy_color)
    {
        Mat inputs = input.clone();
        debug = input.clone();
        // 预处理
        PreProcessImage(inputs, binary_img, enemy_color);
        imshow("binary_img", binary_img);
        // waitKey(0);
        // 查找并筛选灯条
        FindLights(binary_img, True_lights);

        // 查找装甲板
        matchArmor(True_lights, True_armors);
        if (!True_armors.empty())
        {
            ArmorState = ARMOR_FOUND;
            // classifier->extractNumbers(inputs, True_armors);
            // classifier->classify(True_armors);
            // 数字识别
        }
        else
            ArmorState = ARMOR_NOT_FOUND;
    }

    void Detector::PreProcessImage(const cv::Mat &input, cv::Mat &output, int enemy_color)
    {
        Mat element0 = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
        Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        Mat grayImg, bin;
        // 阈值化
        cv::cvtColor(input, grayImg, COLOR_BGR2GRAY);

        threshold(grayImg, bin, 80, 255, THRESH_BINARY);
        // 通道相间
        Mat color;
        std::vector<Mat> splited;
        split(input, splited);
        if (enemy_color == BLUE)
        {
            subtract(splited[0], splited[2], color);
            threshold(color, color, 46, 255, THRESH_BINARY); // blue
        }
        else if (enemy_color == RED)
        {
            subtract(splited[2], splited[0], color);
            threshold(color, color, 40, 255, THRESH_BINARY); // red
        }
        dilate(color, color, element0);
        // 进行与运算
        output = bin & color; // _max_color获得了清晰的二值图
        dilate(output, output, element2);
    }
    void Detector::FindLights(const cv::Mat &binaryImg, std::vector<Light> &lights)
    {
        lights.clear();
        using std::vector;
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            // 得到面积率选
            float ContourArea = contourArea(contour);
            if (contour.size() < 2 || ContourArea < L_Param.LightMinArea)
                continue;
            auto r_rect = cv::minAreaRect(contour);
            auto light = Light(r_rect);
            if (isLight(light))
            {
                lights.emplace_back(light);
            }
        }
    }
    void Detector::matchArmor(std::vector<Light> &lights, std::vector<Armor> &Armors)
    {
        Armors.clear();
        // Loop all the pairing of lights
        for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++)
        {
            for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++)
            {

                if (containLight(*light_1, *light_2, lights))
                {
                    continue;
                }
                auto armor = Armor(*light_1, *light_2);
                if (isArmor(armor))
                {
                    Armors.emplace_back(armor);
                }
            }
        }
    }

    bool Detector::isLight(const Light &light)
    {
        // The ratio of light (short side / long side)
        float ratio = light.width / light.length;
        bool ratio_ok = L_Param.min_ratio < ratio && ratio < L_Param.max_ratio;

        bool angle_ok = light.tilt_angle < L_Param.max_angle;

        bool is_light = ratio_ok && angle_ok;

        // Fill in debug information
        //        auto_aim_interfaces::msg::DebugLight light_data;
        //        light_data.center_x = light.center.x;
        //        light_data.ratio = ratio;
        //        light_data.angle = light.tilt_angle;
        //        light_data.is_light = is_light;
        //        this->debug_lights.data.emplace_back(light_data);

        return is_light;
    }
    bool Detector::isArmor(Armor &armor)
    {
        Light light_1 = armor.left_light;
        Light light_2 = armor.right_light;
        // Ratio of the length of 2 lights (short side / long side)
        float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                                   : light_2.length / light_1.length;
        bool light_ratio_ok = light_length_ratio > A_Param.min_light_ratio;

        // Distance between the center of 2 lights (unit : light length)
        float avg_light_length = (light_1.length + light_2.length) / 2;
        float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
        bool center_distance_ok = (A_Param.min_small_center_distance < center_distance &&
                                   center_distance < A_Param.max_small_center_distance) ||
                                  (A_Param.min_large_center_distance < center_distance &&
                                   center_distance < A_Param.max_large_center_distance);

        // Angle of light center connection
        cv::Point2f diff = light_1.center - light_2.center;
        float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
        bool angle_ok = angle < A_Param.max_angle;

        bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
        armor.armor_type = center_distance > A_Param.min_large_center_distance ? LARGE : SMALL;
        // Fill in debug information
        //        auto_aim_interfaces::msg::DebugArmor armor_data;
        //        armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
        //        armor_data.light_ratio = light_length_ratio;
        //        armor_data.center_distance = center_distance;
        //        armor_data.angle = angle;
        //        armor_data.is_armor = is_armor;
        //        armor_data.armor_type = armor.armor_type == LARGE ? "large" : "small";
        //        this->debug_armors.data.emplace_back(armor_data);

        return is_armor;
    }

    // Check if there is another light in the boundingRect formed by the 2 lights
    bool Detector::containLight(
        const Light &light_1, const Light &light_2, const std::vector<Light> &lights)
    {
        auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
        auto bounding_rect = cv::boundingRect(points);

        for (const auto &test_light : lights)
        {
            if (test_light.center == light_1.center || test_light.center == light_2.center)
                continue;

            if (
                bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
                bounding_rect.contains(test_light.center))
            {
                return true;
            }
        }

        return false;
    }
    // 画出灯条装甲板轮廓
    void Detector::drawResults(cv::Mat &img)
    {
        // Draw Lights
        for (const auto &light : True_lights)
        {
            cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
            cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
            auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
            cv::line(img, light.top, light.bottom, line_color, 1);
        }

        // Draw armors
        for (const auto &armor : True_armors)
        {
            cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
            cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
        }

        // Show numbers and confidence
        for (const auto &armor : True_armors)
        {
            cv::putText(
                img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2);
        }
    }
    // 构造窗口显示角度姿态信息
    void Detector::showDebuginfo(float pitch, float yaw, float dis, int fps)
    {
        Mat img = Mat::zeros(250, 430, CV_8UC3);

        if (ArmorState == ARMOR_FOUND)
        {
            putText(img, "ARMOR_FOUND", Point(100, 35), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
            putText(img, format("Dis: %.1f", dis), Point(100, 140), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
            putText(img, format("pitch: %.1f", pitch), Point(100, 70), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
            putText(img, format("yaw: %.1f", yaw), Point(100, 105), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
            putText(img, format("FPS: %.d", fps), Point(100, 175), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
            imshow("Debuginfo", img);
        }
        else
        {
            putText(img, "ARMOR_NOFOUND", Point(100, 35), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
            imshow("Debuginfo", img);
        }
    }
}
