#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <deque>

#include "mark_detector/geom.hpp"

struct PointXYZI: Point3f {
    double intensity;
};

struct Plane {
    Point3f normal;
    Point3f point;
};

class MarkRecognizer;

class MarkDetectorNode : public rclcpp::Node
{
public:
    MarkDetectorNode();

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    sensor_msgs::msg::PointCloud2::SharedPtr filterHighIntensityPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg, float intensityThreshold);
    std::vector<std::vector<PointXYZI>> detectPointClusters(sensor_msgs::msg::PointCloud2::SharedPtr);
    cv::Mat projectPointsOnPlane(const std::vector<PointXYZI> &points);
    void saveDebugDebugImage(const cv::Mat &image);
    sensor_msgs::msg::PointCloud2::SharedPtr transformToMapFrame(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    void processAccumulatedPoints();
    geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::TransformStamped& transform);

    // Подписки и публикации
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudSubscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mHighIntesityPoints;
    int mNextDebugImageIndex;

    // Для аккумуляции точек
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> mPointCloudBuffer;
    sensor_msgs::msg::PointCloud2::SharedPtr mAccumulatedCloud;
    int mFrameCounter;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;

    std::shared_ptr<MarkRecognizer> m_pMarkRecognizer;
};
