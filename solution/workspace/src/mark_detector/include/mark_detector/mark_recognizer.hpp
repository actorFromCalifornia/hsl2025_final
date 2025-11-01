#pragma once

#include <vector>
#include <iostream>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"

class MarkRecognizer {
public:
    struct Detection {
        int classId;
        float confidence;
        cv::Rect box;
    };

    MarkRecognizer(const std::string& modelPath, 
                  const std::string& classesPath = "",
                  float confThreshold = 0.85f,
                  float nmsThreshold = 0.4f);
    std::vector<Detection> detect(const cv::Mat& image);
    void drawDetections(cv::Mat& frame, const std::vector<MarkRecognizer::Detection>& detections);

private:
    cv::dnn::Net mNet;
    float mConfThreshold;
    float mNMSThreshold;
    std::vector<std::string> mClassNames;

    void loadClassNames(const std::string& classesPath);
    cv::Mat preprocess(const cv::Mat& frame);
    std::vector<Detection> postprocess(std::vector<cv::Mat>& outputs,
                                       const cv::Size& originalSize);
};