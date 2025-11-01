#include "mark_detector/mark_recognizer.hpp"

#include <chrono>

MarkRecognizer::MarkRecognizer(const std::string& modelPath, 
                const std::string& classesPath,
                float confThreshold,
                float nmsThreshold) 
    : mConfThreshold(confThreshold), mNMSThreshold(nmsThreshold) {
    
    std::cout << "Загружаем модель YOLO для детекции крестиков... ";
    // Загрузка модели
    mNet = cv::dnn::readNetFromONNX(modelPath);
    std::cout << "готово" << std::endl;
    
    // Установка бэкенда (CUDA если доступно, иначе CPU)
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        mNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        mNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        std::cout << "Using CUDA backend" << std::endl;
    } else {
        mNet.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        mNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "Using CPU backend" << std::endl;
    }
    
    // Загрузка названий классов
    if (!classesPath.empty()) {
        loadClassNames(classesPath);
    }
}
    
    
std::vector<MarkRecognizer::Detection> MarkRecognizer::detect(const cv::Mat& frame) {
    cv::Size originalSize = frame.size();
    
    // Предобработка и инференс
    cv::Mat blob = preprocess(frame);
 
    mNet.setInput(blob);
    
//    std::cout << "Получение выходных данных" << std::endl;
    // Получение выходных данных
    std::vector<cv::Mat> outputs;
    mNet.forward(outputs, mNet.getUnconnectedOutLayersNames());

//    std::cout << "Постобработка" << std::endl;
    // Постобработка
    return postprocess(outputs, originalSize);    
}
    
void MarkRecognizer::drawDetections(cv::Mat& frame, const std::vector<MarkRecognizer::Detection>& detections) {
    for (const auto& detection : detections) {
        std::string label;
        if (detection.classId < static_cast<int>(mClassNames.size())) {
            label = mClassNames[detection.classId] + ": " + 
                    cv::format("%.2f", detection.confidence);
        } else {
            label = "Class " + std::to_string(detection.classId) + ": " + 
                    cv::format("%.2f", detection.confidence);
        }
        
        // Рисование bounding box
        cv::rectangle(frame, detection.box, cv::Scalar(0, 255, 0), 2);
        
        // Рисование метки
        int baseline;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::rectangle(frame, 
                        cv::Point(detection.box.x, detection.box.y - labelSize.height - 5),
                        cv::Point(detection.box.x + labelSize.width, detection.box.y),
                        cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(frame, label, 
                    cv::Point(detection.box.x, detection.box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}

void MarkRecognizer::loadClassNames(const std::string& classesPath) {
    std::ifstream classFile(classesPath);
    std::string className;
    while (std::getline(classFile, className)) {
        mClassNames.push_back(className);
    }
}

cv::Mat MarkRecognizer::preprocess(const cv::Mat& frame) {
    // Преобразование изображения в blob
    cv::Mat blob;
    cv::Mat rgbImage;
//    std::cout << "Преобразование в blob... ";
    cv::cvtColor(frame, rgbImage, cv::COLOR_GRAY2BGR, 3);
//    cv::dnn::blobFromImage(resizedImage, blob, 1.0/255.0, cv::Size(640, 640));//, cv::Scalar(), false, false);
    cv::dnn::blobFromImage(rgbImage, blob, 1.0/255.0, cv::Size(128, 128));//, cv::Scalar(), false, false);
//    std::cout << "готово" << std::endl;
    
    return blob;
}
        
std::vector<MarkRecognizer::Detection> MarkRecognizer::postprocess(std::vector<cv::Mat>& outputs,
                                    const cv::Size& originalSize) {
    const int nc = 1; // number of classes
    std::vector<Detection> detections;

//    std::cout << "cv::transposeND(outputs[0], {0, 2, 1}, outputs[0]) ";
    cv::transposeND(outputs[0], {0, 2, 1}, outputs[0]);

//    std::cout << "done" << std::endl;
    if (outputs[0].dims != 3) {
        std::cout << "Invalid output shape. The shape should be [1, #anchors, nc+5 or nc+4]" << std::endl;
    }
    if (!(outputs[0].size[2] == nc + 5 || outputs[0].size[2] == nc + 4)) {
        std::cout << "Invalid output shape: " << std::endl;
    }

     for (auto preds : outputs)
    {
        preds = preds.reshape(1, preds.size[1]); // [1, 8400, 85] -> [8400, 85]
        for (int i = 0; i < preds.rows; ++i)
        {
            cv::Mat scores = preds.row(i).colRange(4, preds.cols);
            double conf;
            cv::Point maxLoc;
            minMaxLoc(scores, 0, &conf, 0, &maxLoc);

            if (conf < mConfThreshold)
                continue;
            // std::cout << "Found object " << conf << std::endl;

            // get bbox coords
            float* det = preds.ptr<float>(i);
            double cx = det[0] / 128.0f * originalSize.width;
            double cy = det[1] / 128.0f * originalSize.height;
            double w = det[2] / 128.0f * originalSize.width;
            double h = det[3] / 128.0f * originalSize.height;

            if (w < 10 || w > 24 || h < 10 || h > 24) {
                continue;
            }
            // std::cout << " det[0]: " << det[0]
            //           << " det[1]: " << det[1]
            //           << " det[2]: " << det[2]
            //           << " det[3]: " << det[0]
            //           << std::endl;
            // std::cout << originalSize.width << "x" << originalSize.height << std::endl;

            // [x1, y1, x2, y2]
            detections.push_back({
                maxLoc.x,
                static_cast<float>(conf),
                cv::Rect(static_cast<int>(cx - 0.5 * w), 
                        static_cast<int>(cy - 0.5 * h),
                        static_cast<int>(w), 
                        static_cast<int>(h)
                        )
            });
        }
    }

    // Применение NMS
    std::vector<int> indices;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    for (const auto& detection : detections) {
        confidences.push_back(detection.confidence);
        boxes.push_back(detection.box);
    }
    
    cv::dnn::NMSBoxes(boxes, confidences, mConfThreshold, mNMSThreshold, indices);
    
    std::vector<Detection> finalDetections;
    for (int idx : indices) {
        finalDetections.push_back(detections[idx]);
    }
    
    return finalDetections;
    /*
    // YOLOv8 output обычно имеет размер [1, 84, 8400]
    const int dimensions = output.size[1];
    const int rows = output.size[2];
    
    std::cout << "Output " << dimensions << " " << rows << std::endl;
    
    // Ресайз выхода в 2D матрицу [rows, dimensions]
    cv::Mat outputMatrix(rows, dimensions, CV_32F, (float*)output.data);
    
    // Транспонирование для удобства [dimensions, rows] -> [rows, dimensions]
    outputMatrix = outputMatrix.t();
    
    for (int i = 0; i < outputMatrix.rows; ++i) {
        cv::Mat scores = outputMatrix.row(i).colRange(4, dimensions);
        cv::Point classIdPoint;
        double confidence;
        
        // Нахождение класса с максимальной уверенностью
        minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        
        if (confidence > mConfThreshold) {
            float centerX = outputMatrix.at<float>(i, 0);
            float centerY = outputMatrix.at<float>(i, 1);
            float width = outputMatrix.at<float>(i, 2);
            float height = outputMatrix.at<float>(i, 3);
            
            // Преобразование координат из [0,1] в пиксели
            int left = static_cast<int>((centerX - width / 2) * originalSize.width);
            int top = static_cast<int>((centerY - height / 2) * originalSize.height);
            int right = static_cast<int>((centerX + width / 2) * originalSize.width);
            int bottom = static_cast<int>((centerY + height / 2) * originalSize.height);
            
            // Обеспечение границ изображения
            left = std::max(0, std::min(left, originalSize.width - 1));
            top = std::max(0, std::min(top, originalSize.height - 1));
            right = std::max(0, std::min(right, originalSize.width - 1));
            bottom = std::max(0, std::min(bottom, originalSize.height - 1));
            
            Detection detection;
            detection.classId = classIdPoint.x;
            detection.confidence = static_cast<float>(confidence);
            detection.box = cv::Rect(left, top, right - left, bottom - top);

            if (detection.box.width > 0 && detection.box.height > 0) {
                detections.push_back(detection);
            }
        }
    }
    
    */
}
