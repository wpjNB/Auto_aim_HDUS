#include <opencv2/opencv.hpp>
#include <iostream>
#include "Camera.h"
#include "detector.h"

int main()
{

    HDURM::HKcam hkcam;
    rm_auto_aim::Detector detectorArmor;
    auto model_path = "/home/wpj/RM_Vision_code_US/auto_aim_HDUS/ArmorDetector/model/mlp.onnx";
    auto label_path = "/home/wpj/RM_Vision_code_US/auto_aim_HDUS/ArmorDetector/model/label.txt";
    double threshold = 0.7;
    detectorArmor.classifier = std::make_unique<rm_auto_aim::NumberClassifier>(model_path, label_path, threshold);
    // hkcam.OpenCam("2BDFA2166410");
    // hkcam.SetParam();
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_EXPOSURE, 168); // 曝光 50
    if (!cap.isOpened())
        return 1;
    cv::Mat img;
    while (1)
    {
        cap >> img;
        detectorArmor.detector(img, rm_auto_aim::BLUE);
        cv::Mat img_debug = img.clone();
        detectorArmor.drawResults(img_debug);
        // hkcam.GetFlame(img);
        if (img.empty())
        {
            break;
        }

        // double t1 = (double)cv::getTickCount();
        // detectorArmor.detector(img, BLUE);
        // double t2 = (double)cv::getTickCount();
        // int fps = cv::getTickFrequency() / (t2 - t1);
        // cv::putText(img, "FPS:" + std::to_string(fps), cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2);
        cv::imshow("test", img);
        cv::imshow("debug", img_debug);
        if (cv::waitKey(1) > 0)
            break;
    }
    // hkcam.CloseCam();
    return 0;
}