#include "detector.h"
using namespace cv;
namespace rm_auto_aim
{
  Detector::Detector(const std::string &config_file_path)
  {
    cv::FileStorage config(config_file_path, cv::FileStorage::READ);
    // 初始化灯条参数
    config["detector"]["min_lightness"] >> min_lightness;
    config["detector"]["light_params"]["min_ratio"] >> L_Param.min_ratio;
    config["detector"]["light_params"]["max_ratio"] >> L_Param.max_ratio;
    config["detector"]["light_params"]["max_angle"] >> L_Param.max_angle;
    // 初始化装甲板参数
    config["detector"]["armor_params"]["min_light_ratio"] >> A_Param.min_light_ratio;
    config["detector"]["armor_params"]["min_small_center_distance"] >> A_Param.min_small_center_distance;
    config["detector"]["armor_params"]["max_small_center_distance"] >> A_Param.max_small_center_distance;
    config["detector"]["armor_params"]["min_large_center_distance"] >> A_Param.min_large_center_distance;
    config["detector"]["armor_params"]["max_large_center_distance"] >> A_Param.max_large_center_distance;
    config["detector"]["armor_params"]["max_angle"] >> A_Param.max_angle;
    config["detector"]["armor_params"]["max_angle_diff"] >> A_Param.max_angle_diff;

    // 初始化分类器
    std::string model_path, label_path;
    std::vector<std::string> ignore_classes;
    double threshold;
    config["detector"]["classifier_params"]["model_path"] >> model_path;
    config["detector"]["classifier_params"]["label_path"] >> label_path;
    config["detector"]["classifier_params"]["threshold"] >> threshold;
    this->classifier = std::make_unique<rm_auto_aim::NumberClassifier>(model_path, label_path, threshold);
  }

  void Detector::run(Mat &img, int color_label)
  {
#ifdef USING_ROI
    ImageByROI(img);
#endif
    detector(img, color_label);
  }

  void Detector::ImageByROI(Mat &img)
  {
    static Rect imgBound = Rect(0, 0, img.cols, img.rows);
    if (ArmorState == ARMOR_FOUND)
    {
      imgBound =
          Rect(img.cols / 4, img.rows / 4, img.cols / 4 * 3, img.rows / 4 * 3);
    }
    else if (ArmorState == ARMOR_NOT_FOUND)
    {
      static int lost_cnt;
      if (++lost_cnt > 6) // 装甲板丢失5振
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
    //        std::sort(armors.begin(), armors.end(), [](const Armor &armor1,
    //        const Armor &armor2)
    //        {
    //            return armor1.confidence > armor2.confidence;
    //        });
    // 不存在装甲板直接返回
    if (True_armors.empty())
      return 0;

    // 存在装甲板
    // 选择距离中心最近的装甲板(也可以通过面积排序选择距离最近的提高命中率)
    std::sort(True_armors.begin(), True_armors.end(),
              [&frame](const Armor &armor1, const Armor &armor2)
              {
                cv::Point2f _center((frame.cols - 1) / 2.0,
                                    (frame.rows - 1) / 2.0); // point(x,y)
                return cv::norm(armor1.center - _center) >
                       cv::norm(armor2.center - _center);
              });

    TargetArmor = True_armors[0];

    return 1;
  }
  void Detector::detector(const cv::Mat &input, int enemy_color)
  {
    Mat inputs = input.clone();
    // 预处理
    PreProcessImage(inputs, binary_img, enemy_color);

    // 查找并筛选灯条
    FindLights(inputs, binary_img, True_lights);

    // 查找装甲板
    matchArmor(True_lights, True_armors, enemy_color);

    for (auto &armor : True_armors)
    {
      Point2f _center((input.cols - 1) / 2, (input.rows - 1) / 2);
      armor.distance_to_image_center = norm(armor.center - _center);
    }

    if (!True_armors.empty())
    {
      ArmorState = ARMOR_FOUND;
      // classifier->extractNumbers(inputs, True_armors);
      // classifier->classify(True_armors);
      if (True_armors.empty()) // 筛选完后可能不存在装甲版了
      {
        ArmorState = ARMOR_NOT_FOUND;
      }
    }
    else
      ArmorState = ARMOR_NOT_FOUND;
  }

  void Detector::PreProcessImage(const cv::Mat &input, cv::Mat &output,
                                 int enemy_color)
  {

    Mat grayImg, bin;
    // 灰度处理+阈值化
    cv::cvtColor(input, grayImg, COLOR_BGR2GRAY);
    threshold(grayImg, output, min_lightness, 255, THRESH_BINARY);
  }
  void Detector::FindLights(const cv::Mat &rbg_img, const cv::Mat &binaryImg, std::vector<Light> &lights)
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
      if (contour.size() < 5 || ContourArea < L_Param.LightMinArea)
        continue;
      auto r_rect = cv::minAreaRect(contour);
      auto light = Light(r_rect);
      if (isLight(light))
      {
        auto rect = light.boundingRect();
        if ( // Avoid assertion failed
            0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
            0 <= rect.height && rect.y + rect.height <= rbg_img.rows)
        {
          int sum_r = 0, sum_b = 0;
          auto roi = rbg_img(rect);
          // Iterate through the ROI
          for (int i = 0; i < roi.rows; i++)
          {
            for (int j = 0; j < roi.cols; j++)
            {
              if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0)
              {
                // if point is inside contour//摄像头采集·的画面是BGR图像
                sum_r += roi.at<cv::Vec3b>(i, j)[2]; // R
                sum_b += roi.at<cv::Vec3b>(i, j)[0]; // B
              }
            }
          }
          // Sum of red pixels > sum of blue pixels ?
          light.color = sum_r > sum_b ? RED : BLUE;
          lights.emplace_back(light);
        }
      }
    }
  }

  void Detector::matchArmor(const std::vector<Light> &lights, std::vector<Armor> &Armors, int enemy_color)
  {
    Armors.clear();
    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++)
    {
      for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++)
      {

        if (light_1->color != enemy_color || light_2->color != enemy_color)
        {
          continue;
        }

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

    return is_light;
  }
  bool Detector::isArmor(Armor &armor)
  {
    Light light_1 = armor.left_light;
    Light light_2 = armor.right_light;
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length
                                   ? light_1.length / light_2.length
                                   : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > A_Param.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance =
        cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok =
        (A_Param.min_small_center_distance < center_distance &&
         center_distance < A_Param.max_small_center_distance) ||
        (A_Param.min_large_center_distance < center_distance &&
         center_distance < A_Param.max_large_center_distance);

    // Angle of light center connection  ()
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < A_Param.max_angle;
    // // 添加 ARMOR的两条灯条角度差（平行性）效果差并不好（删去）
    float angle_diff = std::abs(light_1.absolute_angle - light_2.absolute_angle);
    bool angle_diff_ok = angle_diff < A_Param.max_angle_diff;
    // fmt::print("light_1.angle:{},light_2.angle:{}\n", light_1.absolute_angle, light_2.absolute_angle);
    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok && angle_diff_ok;
    armor.armor_type = center_distance > A_Param.min_large_center_distance ? LARGE : SMALL;

    return is_armor;
  }

  // Check if there is another light in the boundingRect formed by the 2 lights
  bool Detector::containLight(const Light &light_1, const Light &light_2,
                              const std::vector<Light> &lights)
  {
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom,
                                           light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto &test_light : lights)
    {
      if (test_light.center == light_1.center ||
          test_light.center == light_2.center)
        continue;

      if (bounding_rect.contains(test_light.top) ||
          bounding_rect.contains(test_light.bottom) ||
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
      auto line_color =
          light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
      cv::line(img, light.top, light.bottom, line_color, 1);
    }

    // Draw armors
    for (const auto &armor : True_armors)
    {
      cv::line(img, armor.left_light.top, armor.right_light.bottom,
               cv::Scalar(0, 255, 0), 2);
      cv::line(img, armor.left_light.bottom, armor.right_light.top,
               cv::Scalar(0, 255, 0), 2);
    }

    // Show numbers and confidence
    for (const auto &armor : True_armors)
    {
      cv::putText(img, armor.classfication_result, armor.left_light.top,
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
      // cv::putText(img, std::to_string(armor.position_world[0]), armor.center - Point2f(10, 10),
      //             cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 1);
    }
  }
  // 构造窗口显示角度姿态信息
  void Detector::showDebuginfo(Mat &test, Armor &armor)
  {
    // Mat img = Mat::zeros(330, 500, CV_8UC3);

    if (ArmorState == ARMOR_FOUND)
    {
      cv::Point2f pts[4] = {armor.vertex[0], armor.vertex[1], armor.vertex[2], armor.vertex[3]};
      cv::Point2f pts_center;
      pts_center = points_center(pts);
      cv::circle(test, pts_center, 7, cv::Scalar(0, 255, 255), 2);
      // putText(img, "ARMOR_FOUND", Point(100, 35), FONT_HERSHEY_SIMPLEX, 1,
      //         Scalar(255, 0, 255), 2, 8, false);

      // putText(img, format("pitch: %.2f", armor.pitch), Point(100, 70),
      //         FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
      // putText(img, format("yaw: %.2f", armor.yaw), Point(100, 105),
      //         FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);

      // putText(img, format("Dis: %.1f", armor.dis * 100), Point(100, 140),
      //         FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
      // putText(img, format("X: %.1f", armor.position_cam[0] * 100), Point(100, 175), FONT_HERSHEY_SIMPLEX,
      //         1, Scalar(255, 0, 255), 2, 8, false);
      // putText(img, format("Y: %.1f", armor.position_cam[1] * 100), Point(100, 205), FONT_HERSHEY_SIMPLEX,
      //         1, Scalar(255, 0, 255), 2, 8, false);
      // putText(img, format("Z: %.1f", armor.position_cam[2] * 100), Point(100, 240), FONT_HERSHEY_SIMPLEX,
      //         1, Scalar(255, 0, 255), 2, 8, false);
      // imshow("Debuginfo", img);
      putText(test, format("XYZ_C: %.2f  %.2f  %.2f", armor.position_cam[0] * 100, armor.position_cam[1] * 100, armor.position_cam[2] * 100), Point(10, 30), FONT_HERSHEY_SIMPLEX,
              0.65, Scalar(0, 255, 0), 2, 8, false);
      putText(test, format("XYZ_W: %.2f  %.2f  %.2f", armor.position_world[0] * 100, armor.position_world[1] * 100, armor.position_world[2] * 100), Point(10, 60), FONT_HERSHEY_SIMPLEX,
              0.65, Scalar(0, 255, 0), 2, 8, false);

      putText(test, format("yaw_cam: %.2f", armor.rotationPYR_cam[0] * 57.3), Point(10, 120), FONT_HERSHEY_SIMPLEX,
              0.65, Scalar(0, 255, 0), 2, 8, false);
      putText(test, format("yaw_world: %.2f", armor.rotationPYR_cam[1] * 57.3), Point(10, 90), FONT_HERSHEY_SIMPLEX,
              0.65, Scalar(0, 255, 0), 2, 8, false);
    }
    else
    {
      // putText(img, "ARMOR_NOFOUND", Point(100, 35), FONT_HERSHEY_SIMPLEX, 1,
      //         Scalar(255, 0, 255), 2, 8, false);
      // imshow("Debuginfo", img);
    }
  }

}

// namespace rm_vision