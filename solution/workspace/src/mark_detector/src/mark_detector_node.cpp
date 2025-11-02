#include "mark_detector/mark_detector_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

#include "mark_detector/mark_recognizer.hpp"

using namespace std::chrono_literals;

const int ACCUM_FRAMES_COUNT = 5;
const float INTENSITY_THRESHOLD = 160.0f;
const float image_resolution = 0.005f;

tf2::Vector3 getGoal(tf2::Vector3 center, tf2::Quaternion rotation_quat) {
    tf2::Vector3 dirVector;
    dirVector.setX(0.5);
    dirVector.setY(0);
    dirVector.setZ(0);

    // Нормализация кватерниона (рекомендуется)
    rotation_quat.normalize();

    // Поворот вектора
    dirVector = tf2::quatRotate(rotation_quat, dirVector);
    return center + dirVector;
}

MarkDetectorNode::MarkDetectorNode()
: Node("mark_detector_node"), mNextDebugImageIndex(0), mFrameCounter(0)
{
    // Инициализация TF2
    mTfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(20));
    mTfBuffer->setUsingDedicatedThread(true);
    mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
    m_pMarksPublisher = create_publisher<visualization_msgs::msg::Marker>("detected_marks", 1);
    
    // Инициализация аккумулированного облака
    mAccumulatedCloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    
    // Подписки и публикации
    mPointCloudSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&MarkDetectorNode::pointcloud_callback, this, std::placeholders::_1));
    
    mHighIntesityPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_high", 10);
    
    RCLCPP_INFO(this->get_logger(), "Mark detector node initialized with frame accumulation");
}

void MarkDetectorNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try {
        // RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
        
        // Фильтруем точки с высокой интенсивностью
        auto high_intensity_cloud = filterHighIntensityPoints(msg, INTENSITY_THRESHOLD);

        // Преобразуем облако в фрейм map (используем текущее время вместо времени сообщения)
        auto transformed_cloud = transformToMapFrame(high_intensity_cloud);
        // auto transformed_cloud = msg;
        if (!transformed_cloud) {
            RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud to map frame");
            return;
        }
        
        // Добавляем в буфер
        mPointCloudBuffer.push_back(transformed_cloud);
        
        // Ограничиваем размер буфера (на всякий случай)
        if (mPointCloudBuffer.size() > ACCUM_FRAMES_COUNT) {
            mPointCloudBuffer.pop_front();
        }
        
        mFrameCounter++;
        
        // Обрабатываем каждые 3 кадра
        if (mFrameCounter >= ACCUM_FRAMES_COUNT) {
            processAccumulatedPoints();
            mFrameCounter = 0;
            mPointCloudBuffer.clear();
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

void MarkDetectorNode::processAccumulatedPoints()
{
    if (mPointCloudBuffer.empty()) {
        // RCLCPP_DEBUG(this->get_logger(), "No point clouds to process");
        return;
    }
    
    try {
        // Объединяем все облака из буфера
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        auto frameId = mPointCloudBuffer[mPointCloudBuffer.size() - 1]->header.frame_id;
        auto stamp = mPointCloudBuffer[mPointCloudBuffer.size() - 1]->header.stamp;
        
        for (const auto& cloud : mPointCloudBuffer) {
            pcl::PointCloud<pcl::PointXYZI> temp_cloud;
            pcl::fromROSMsg(*cloud, temp_cloud);
            *accumulated_pcl_cloud += temp_cloud;
        }
        
        // RCLCPP_INFO(this->get_logger(), "Accumulated %zu points from %zu frames", 
        //            accumulated_pcl_cloud->size(), mPointCloudBuffer.size());
        
        if (accumulated_pcl_cloud->empty()) {
            // RCLCPP_DEBUG(this->get_logger(), "No points accumulated");
            return;
        }
        
        // Конвертируем обратно в ROS сообщение
        pcl::toROSMsg(*accumulated_pcl_cloud, *mAccumulatedCloud);
        mAccumulatedCloud->header.stamp = stamp;
        mAccumulatedCloud->header.frame_id = frameId;
        
        if (mAccumulatedCloud->width * mAccumulatedCloud->height == 0) {
            // RCLCPP_DEBUG(this->get_logger(), "No high intensity points found in accumulated clouds");
            return;
        }
        
        // RCLCPP_DEBUG(this->get_logger(), "Found %d high intensity points in accumulated clouds", 
        //             mAccumulatedCloud->width * mAccumulatedCloud->height);
        
        // Детектируем кластеры точек
        auto clusters = detectPointClusters(mAccumulatedCloud);
        
        RCLCPP_INFO(this->get_logger(), "Detected %zu clusters in accumulated clouds", clusters.size());
        
        // Обрабатываем каждый кластер
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (clusters[i].size() < 5) { // Минимальный размер кластера
                continue;
            }

            auto &cluster = clusters[i];
            
            // Находим плоскость для кластера
            auto plane = buildPlaneForPoints(cluster);
            // Проецируем точки на плоскость
            cv::Mat projected_image = projectPointsOnPlane(cluster, plane);
            
            if (!projected_image.empty()) {
                // Сохраняем отладочное изображение
                saveDebugDebugImage(projected_image);
                double markWidth = projected_image.cols * image_resolution;
                double markHeight = projected_image.rows * image_resolution;
                Point3f center;
                for (auto& p : cluster) {
                    center += p;
                }
                center /= cluster.size();

                bool isStable = false;
                bool isRed = false;

                // признаки для меток:
                // поля - 0
                // 1. ширина и высота. 
                //    красная метка - 0.15
                //    белая метка - 0.1
                //    синяя метка - 0.125 x 0.195
                // 2. "Яркость" метки:
                //   красная - 0.8
                //   белая - 1.0
                //   синяя - 0.5

                cv::Scalar meanColor = cv::mean(projected_image);
                auto g = meanColor[1];//((meanColor & 0xff00) >> 8);
                float brightness = g / 255.0f;
                
                if (m_pMarkRecognizer) {
                    auto detections = m_pMarkRecognizer->detect(projected_image);
                    for (auto &detection : detections) {
                        RCLCPP_INFO(this->get_logger(), "Detected mark %d with confidence %f", detection.classId, detection.confidence);
                    }
                } else {
                    if (markWidth >= 0.125 && markWidth <= 0.17
                    && markHeight >= 0.125 && markHeight <= 0.17
                    && brightness >= 0.6 && brightness <= 0.9) {
                        // красная метка
                        isRed = true;
                    } else if (markWidth >= 0.08 && markWidth <= 0.125
                    && markHeight >= 0.08 && markHeight <= 0.125
                    && brightness >= 0.9) {
                        // белая метка
                    } else if (markWidth >= 0.105 && markWidth <= 0.145
                    && markHeight >= 0.175 && markHeight <= 0.215
                    && brightness >= 0.3 && brightness <= 0.55) {
                        // синяя метка
                    // RCLCPP_WARN(this->get_logger(), "Mark recognizer not initialized");
                    } else {
                    // мусор
                        continue;
                    }
                }

                Mark detectedMark;
                detectedMark.id = mMarks.size();
                detectedMark.isRed = isRed;
                detectedMark.isStable = isStable;
                detectedMark.center = center;
                detectedMark.normal = plane.normal;
                for (int markId = 0; markId < mMarks.size(); markId++) {
                    auto mark = mMarks[markId];
                    if ((mark.center - detectedMark.center).length() < 0.3) {
                        detectedMark.center = (mark.center + detectedMark.center) / 2;
                        detectedMark.normal = (mark.normal + detectedMark.normal) / 2;
                        detectedMark.id = mark.id;
                        break;
                    }
                }
                if (detectedMark.id >= mMarks.size()) {
                    mMarks.push_back(detectedMark);
                }

                sendMarker(
                    detectedMark.id, 
                    detectedMark.center, 
                    detectedMark.normal, 
                    detectedMark.isRed ? visualization_msgs::msg::Marker::SPHERE : visualization_msgs::msg::Marker::CUBE, 
                    detectedMark.isRed ? 1 : 0, 
                    detectedMark.isRed ? 0 : 1, 
                    0
                );
            }
        }
        
        // Публикуем точки высокой интенсивности
        mHighIntesityPoints->publish(*mAccumulatedCloud);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing accumulated point clouds: %s", e.what());
    }
}

void MarkDetectorNode::sendMarker(int id, const Point3f &center, const Point3f &normal, int32_t markerType, float r, float g, float b) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();

    marker.ns = "marks";
    marker.id = id;

    marker.type = markerType;

    marker.action = visualization_msgs::msg::Marker::ADD;

    Point2f n2;
    n2.x = normal.x;
    n2.y = normal.y;
    n2.normalize();
    double yaw = n2.angle(Point2f{1, 0});

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = center.z;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    {
        tf2::Vector3 c;
        c.setX(center.x);
        c.setY(center.y);
        c.setZ(center.z);

        tf2::Quaternion q;
        q.setX(marker.pose.orientation.x);
        q.setY(marker.pose.orientation.y);
        q.setZ(marker.pose.orientation.z);
        q.setW(marker.pose.orientation.w);

        auto nextGoal = getGoal(c, q);
        RCLCPP_INFO(this->get_logger(), "DETECTED MARKER: %d %d center (%lf, %lf, %lf) -> goal (%lf, %lf, %lf)", 
            id, markerType, 
            center.x, center.y, center.z, 
            nextGoal.x(), nextGoal.y(), nextGoal.z()    
        );
    }

    marker.scale.x = 0.01;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    m_pMarksPublisher->publish(marker);      
}

sensor_msgs::msg::PointCloud2::SharedPtr MarkDetectorNode::transformToMapFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try {
        // Получаем трансформацию в фрейм map - используем текущее время вместо времени сообщения
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Используем текущее время для трансформации вместо времени сообщения
            rclcpp::Time now = this->now();
            transform = mTfBuffer->lookupTransform("map", msg->header.frame_id, 
                                                 msg->header.stamp, rclcpp::Duration::from_seconds(10.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            
            // Попробуем получить самую свежую доступную трансформацию
            try {
                transform = mTfBuffer->lookupTransform("map", msg->header.frame_id, 
                                                     tf2::TimePointZero);
//                RCLCPP_INFO(this->get_logger(), "Using latest available transform");
            } catch (tf2::TransformException &ex2) {
                RCLCPP_ERROR(this->get_logger(), "Cannot get transform even with latest time: %s", ex2.what());
                return nullptr;
            }
        }
        
        // Конвертируем облако в PCL для ручного преобразования
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *input_cloud);
        
        // Создаем новое облако для преобразованных точек
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        transformed_cloud->header = input_cloud->header;
        transformed_cloud->is_dense = input_cloud->is_dense;
        
        // Преобразуем каждую точку
        for (const auto& point : *input_cloud) {
            geometry_msgs::msg::Point input_point;
            input_point.x = point.x;
            input_point.y = point.y;
            input_point.z = point.z;
            
            geometry_msgs::msg::Point transformed_point = transformPoint(input_point, transform);
            
            pcl::PointXYZI new_point;
            new_point.x = transformed_point.x;
            new_point.y = transformed_point.y;
            new_point.z = transformed_point.z;
            new_point.intensity = point.intensity;
            
            transformed_cloud->push_back(new_point);
        }
        
        // Конвертируем обратно в ROS сообщение
        auto result_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*transformed_cloud, *result_cloud);
        result_cloud->header.frame_id = "map";
        result_cloud->header.stamp = transform.header.stamp; // Используем текущее время
        
        return result_cloud;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error transforming point cloud: %s", e.what());
        return nullptr;
    }
}

geometry_msgs::msg::Point MarkDetectorNode::transformPoint(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::TransformStamped& transform)
{
    geometry_msgs::msg::PointStamped input_stamped;
    input_stamped.point = point;
    input_stamped.header.frame_id = transform.child_frame_id;
    input_stamped.header.stamp = transform.header.stamp;
    
    geometry_msgs::msg::PointStamped output_stamped;
    tf2::doTransform(input_stamped, output_stamped, transform);
    
    return output_stamped.point;
}

sensor_msgs::msg::PointCloud2::SharedPtr MarkDetectorNode::filterHighIntensityPoints(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, float intensityThreshold)
{
    auto filtered_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    
    // Конвертируем в PCL формат
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) {
        return filtered_cloud;
    }
    
    // Фильтруем точки по интенсивности
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    for (const auto& point : *cloud) {
        double len_2 = point.x * point.x + point.y * point.y;
        if (point.intensity >= intensityThreshold && len_2 < 4 && len_2 > 0.09) {
            filtered_pcl_cloud->push_back(point);
        }
    }
    
    // Конвертируем обратно в ROS сообщение
    pcl::toROSMsg(*filtered_pcl_cloud, *filtered_cloud);
    filtered_cloud->header = msg->header;

    return filtered_cloud;
}

std::vector<std::vector<PointXYZI>> MarkDetectorNode::detectPointClusters(
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    std::vector<std::vector<PointXYZI>> clusters;
    
    // Конвертируем в PCL формат
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    if (cloud->empty()) {
        return clusters;
    }
    
    // Создаем KdTree для поиска соседей
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    
    // Евклидова кластеризация
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.3); // 50cm
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    // Преобразуем индексы в точки
    for (const auto& indices : cluster_indices) {
        std::vector<PointXYZI> cluster;
        for (const auto& index : indices.indices) {
            PointXYZI point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.intensity = cloud->points[index].intensity;
            cluster.push_back(point);
        }
        clusters.push_back(cluster);
    }
    
    return clusters;
}

Plane MarkDetectorNode::buildPlaneForPoints(const std::vector<PointXYZI> &points) {
    if (points.size() < 3) {
        return Plane();
    }

    try {
        // Конвертируем точки в PCL формат
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            cloud->push_back(pcl_point);
        }
        
        // Находим плоскость с помощью RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough inliers for plane fitting");
            return Plane();
        }
        
        // Извлекаем параметры плоскости
        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        
        // Нормаль плоскости
        Point3f normal(a, b, c);
        normal = normal.normalized();
        
        // Находим точку на плоскости (ближайшую к началу координат)
        Point3f plane_point;
        if (std::abs(c) > 1e-6) {
            plane_point.z = -d / c;
            plane_point.x = 0;
            plane_point.y = 0;
        } else if (std::abs(b) > 1e-6) {
            plane_point.y = -d / b;
            plane_point.x = 0;
            plane_point.z = 0;
        } else {
            plane_point.x = -d / a;
            plane_point.y = 0;
            plane_point.z = 0;
        }

        Plane plane;
        plane.normal = normal;
        plane.point = plane_point;
        return plane;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to build plane: %s", e.what());
        return Plane();
    }
}

cv::Mat MarkDetectorNode::projectPointsOnPlane(const std::vector<PointXYZI> &points, const Plane &plane)
{
    if (points.size() < 3 || plane.normal.length() < 0.01) {
        return cv::Mat();
    }
    
    try {
        auto normal = plane.normal;
        auto plane_point = plane.point;
        // Создаем систему координат на плоскости
        Point3f u_axis, v_axis;
        
        // Выбираем ось U произвольно, но ортогонально нормали
        u_axis = Point3f(normal.y, -normal.x, 0).normalized();
        
        // Ось V как векторное произведение нормали и оси U
        v_axis = normal.cross(u_axis).normalized();
        
        // Проецируем точки на плоскость
        std::vector<cv::Point2f> projected_points;
        float min_u = std::numeric_limits<float>::max();
        float max_u = std::numeric_limits<float>::lowest();
        float min_v = std::numeric_limits<float>::max();
        float max_v = std::numeric_limits<float>::lowest();
        
        for (const auto& point : points) {
            Point3f vec(point.x - plane_point.x, point.y - plane_point.y, point.z - plane_point.z);
            
            float u = vec.dot(u_axis);
            float v = vec.dot(v_axis);
            
            projected_points.push_back(cv::Point2f(u, v));
            
            min_u = std::min(min_u, u);
            max_u = std::max(max_u, u);
            min_v = std::min(min_v, v);
            max_v = std::max(max_v, v);
        }
        
        // Создаем изображение
        const float margin = 0.00f; // 2cm margin
        
        float range_u = max_u - min_u + 2 * margin;
        float range_v = max_v - min_v + 2 * margin;
        int image_width = static_cast<int>(range_u / image_resolution);
        int image_height = static_cast<int>(range_v / image_resolution);
        float scale_x = image_width / range_u;
        float scale_y = image_height / range_v;
        
        cv::Mat image(image_height, image_width, CV_8UC3, cv::Scalar(0, 0, 0));
        
        for (const auto& proj_point : projected_points) {
            int x = static_cast<int>((proj_point.x - min_u + margin) * scale_x);
            int y = static_cast<int>((proj_point.y - min_v + margin) * scale_y);
            
            if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
                cv::circle(image, cv::Point(x, image_height - y - 1), 2, cv::Scalar(0, 255, 0), -1);
            }
        }
        
        return image;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error projecting points: %s", e.what());
        return cv::Mat();
    }
}

void MarkDetectorNode::saveDebugDebugImage(const cv::Mat &image)
{
    if (image.empty()) {
        return;
    }
    
    try {
        std::string filename = "/workspace/docker_ws/marks/mark_cluster_" + std::to_string(mNextDebugImageIndex++) + ".png";
        cv::imwrite(filename, image);
        // RCLCPP_DEBUG(this->get_logger(), "Saved debug image: %s", filename.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error saving debug image: %s", e.what());
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkDetectorNode>());
  rclcpp::shutdown();

  return 0;
}
