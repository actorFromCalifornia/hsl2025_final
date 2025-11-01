#pragma once

#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#pragma pack(push, 1)
struct Point2f {
    double x;
    double y;

    Point2f(double x = 0, double y = 0);
    Point2f(const geometry_msgs::msg::Point& p);
    Point2f(const geometry_msgs::msg::Vector3& p);

    inline double lengthSqr() const { return x * x + y * y; }
    inline double length() const { return std::sqrt(lengthSqr()); }

    Point2f& operator+=(const Point2f& p);
    Point2f& operator-=(const Point2f& p);
    Point2f& operator*=(double scale);
    Point2f& operator/=(double scale);

    inline double dot(const Point2f& p) const { return x * p.x + y * p.y; }
    inline double cross(const Point2f& p) const { return x * p.y - y * p.x; }

    double angle(const Point2f& p) const;
    Point2f project(const Point2f& base) const;
    Point2f normalized() const;
    void normalize();
    Point2f rotated(double angle_rad) const;
    
    // Расстояние от точки до прямой, заданной точкой и направляющим вектором
    double distanceToLine(const Point2f& line_point, const Point2f& line_dir) const;
    
    // Расстояние от точки до отрезка, заданного двумя точками
    double distanceToSegment(const Point2f& segment_start, const Point2f& segment_end) const;
};

struct Point3f {
    double x;
    double y;
    double z;

    Point3f(double x = 0, double y = 0, double z = 0);
    Point3f(const geometry_msgs::msg::Point& p);

    inline double lengthSqr() const { return x * x + y * y + z * z; }
    inline double length() const { return std::sqrt(lengthSqr()); }

    Point3f& operator+=(const Point3f& p);
    Point3f& operator-=(const Point3f& p);
    Point3f& operator*=(double scale);
    Point3f& operator/=(double scale);

    inline double dot(const Point3f& p) const { return x * p.x + y * p.y + z * p.z; }
    Point3f cross(const Point3f& p) const;

    double angle(const Point3f& p) const;
    Point3f project(const Point3f& base) const;
    Point3f normalized() const;
    void normalize();

    // Rotation methods
    Point3f rotated(const Point3f& axis, double angle_rad) const;
    Point3f rotatedX(double angle_rad) const;
    Point3f rotatedY(double angle_rad) const;
    Point3f rotatedZ(double angle_rad) const;
    Point3f rotatedEuler(double yaw, double pitch, double roll) const;
    Point3f rotatedEuler(const Point3f& euler_angles) const;
    
    // Расстояние от точки до прямой, заданной точкой и направляющим вектором
    double distanceToLine(const Point3f& line_point, const Point3f& line_dir) const;
    
    // Расстояние от точки до отрезка, заданного двумя точками
    double distanceToSegment(const Point3f& segment_start, const Point3f& segment_end) const;
};
#pragma pack(pop)

// Point2f operators
Point2f operator+(const Point2f &p1, const Point2f &p2);
Point2f operator-(const Point2f &p1, const Point2f &p2);
Point2f operator*(const Point2f &p, double scale);
Point2f operator/(const Point2f &p, double scale);
Point2f min(const Point2f &p1, const Point2f &p2);
Point2f max(const Point2f &p1, const Point2f &p2);

// Point3f operators
Point3f operator+(const Point3f &p1, const Point3f &p2);
Point3f operator-(const Point3f &p1, const Point3f &p2);
Point3f operator*(const Point3f &p, double scale);
Point3f operator/(const Point3f &p, double scale);
Point3f min(const Point3f &p1, const Point3f &p2);
Point3f max(const Point3f &p1, const Point3f &p2);


// Полярные координаты с математикой в круге [0; 2pi) 
struct PolarCoordinate {
private:
    double mValue;

public:
    PolarCoordinate();
    explicit PolarCoordinate(double value);
    
    operator double() const;
    
    PolarCoordinate& operator+=(PolarCoordinate const& right);
    PolarCoordinate& operator-=(PolarCoordinate const& right);

    friend PolarCoordinate operator+(PolarCoordinate left, PolarCoordinate const& right);
    friend PolarCoordinate operator-(PolarCoordinate left, PolarCoordinate const& right);

    // минимальная дистанция в интервале [-180; 180)
    static double distance(PolarCoordinate const& from, PolarCoordinate const& to) {
        double d = to.mValue - from.mValue;
        
        while (d >= M_PI) {
            d -= 2 * M_PI;
        }
        while (d < -M_PI) {
            d += 2 * M_PI;
        }
        return d;
    }
    
private:
    void cropTo360();
};
