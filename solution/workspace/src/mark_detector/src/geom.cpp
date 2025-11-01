#include "mark_detector/geom.hpp"

// Point2f implementations

Point2f::Point2f(double x, double y) : x(x), y(y) {}
Point2f::Point2f(const geometry_msgs::msg::Point& p) : x(p.x), y(p.y) {}
Point2f::Point2f(const geometry_msgs::msg::Vector3& p) : x(p.x), y(p.y) {}

Point2f& Point2f::operator+=(const Point2f& p) {
    x += p.x;
    y += p.y;
    return *this;
}

Point2f& Point2f::operator-=(const Point2f& p) {
    x -= p.x;
    y -= p.y;
    return *this;
}

Point2f& Point2f::operator*=(double scale) {
    x *= scale;
    y *= scale;
    return *this;
}

Point2f& Point2f::operator/=(double scale) {
    x /= scale;
    y /= scale;
    return *this;
}

double Point2f::angle(const Point2f& p) const {
    return std::atan2(cross(p), dot(p));
}

Point2f Point2f::project(const Point2f& base) const {
    double baseLengthSqr = base.lengthSqr();
    if (baseLengthSqr == 0) return Point2f();
    return base * (dot(base) / baseLengthSqr);
}

Point2f Point2f::normalized() const {
    double len = length();
    if (len == 0) return *this;
    return *this / len;
}

void Point2f::normalize() {
    double len = length();
    if (len != 0) {
        x /= len;
        y /= len;
    }
}

Point2f Point2f::rotated(double angle_rad) const {
    double cos_a = std::cos(angle_rad);
    double sin_a = std::sin(angle_rad);
    return {
        x * cos_a - y * sin_a,
        x * sin_a + y * cos_a
    };
}

double Point2f::distanceToLine(const Point2f& line_point, const Point2f& line_dir) const {
    double dirLength = line_dir.length();
    if (dirLength < 0.0001f) {
        // Если направляющий вектор нулевой, возвращаем расстояние до точки
        return (*this - line_point).length();
    }
    
    // Вектор от точки на прямой до нашей точки
    Point2f to_point = *this - line_point;
    
    // Расстояние = |(to_point × line_dir)| / |line_dir|
    // где × - псевдоскалярное произведение (cross product в 2D)
    return std::abs(to_point.cross(line_dir)) / dirLength;
}

double Point2f::distanceToSegment(const Point2f& segment_start, const Point2f& segment_end) const {
    Point2f segment_dir = segment_end - segment_start;
    double segment_length_sqr = segment_dir.lengthSqr();
    
    if (segment_length_sqr == 0) {
        // Отрезок вырожден в точку
        return (*this - segment_start).length();
    }
    
    // Параметр проекции точки на прямую отрезка
    // t = ( (p - start) · (end - start) ) / |end - start|^2
    Point2f to_point = *this - segment_start;
    double t = to_point.dot(segment_dir) / segment_length_sqr;
    
    if (t < 0) {
        // Ближайшая точка - начало отрезка
        return (*this - segment_start).length();
    } else if (t > 1) {
        // Ближайшая точка - конец отрезка
        return (*this - segment_end).length();
    } else {
        // Ближайшая точка внутри отрезка
        Point2f projection = segment_start + segment_dir * t;
        return (*this - projection).length();
    }
}

// Point3f implementations

Point3f::Point3f(double x, double y, double z) : x(x), y(y), z(z) {}
Point3f::Point3f(const geometry_msgs::msg::Point& p) : x(p.x), y(p.y), z(p.z) {}

Point3f& Point3f::operator+=(const Point3f& p) {
    x += p.x;
    y += p.y;
    z += p.z;
    return *this;
}

Point3f& Point3f::operator-=(const Point3f& p) {
    x -= p.x;
    y -= p.y;
    z -= p.z;
    return *this;
}

Point3f& Point3f::operator*=(double scale) {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
}

Point3f& Point3f::operator/=(double scale) {
    x /= scale;
    y /= scale;
    z /= scale;
    return *this;
}

Point3f Point3f::cross(const Point3f& p) const {
    return {
        y * p.z - z * p.y,
        z * p.x - x * p.z,
        x * p.y - y * p.x
    };
}

double Point3f::angle(const Point3f& p) const {
    double dotProduct = dot(p);
    double lenProduct = length() * p.length();
    if (lenProduct == 0) return 0;
    return std::acos(std::clamp(dotProduct / lenProduct, -1.0, 1.0));
}

Point3f Point3f::project(const Point3f& base) const {
    double baseLengthSqr = base.lengthSqr();
    if (baseLengthSqr == 0) return Point3f();
    return base * (dot(base) / baseLengthSqr);
}

Point3f Point3f::normalized() const {
    double len = length();
    if (len == 0) return *this;
    return *this / len;
}

void Point3f::normalize() {
    double len = length();
    if (len != 0) {
        x /= len;
        y /= len;
        z /= len;
    }
}

Point3f Point3f::rotated(const Point3f& axis, double angle_rad) const {
    Point3f norm_axis = axis.normalized();
    double cos_a = std::cos(angle_rad);
    double sin_a = std::sin(angle_rad);
    
    // Формула Родрига
    return *this * cos_a + 
           norm_axis.cross(*this) * sin_a + 
           norm_axis * (norm_axis.dot(*this) * (1 - cos_a));
}

Point3f Point3f::rotatedX(double angle_rad) const {
    double cos_a = std::cos(angle_rad);
    double sin_a = std::sin(angle_rad);
    return {
        x,
        y * cos_a - z * sin_a,
        y * sin_a + z * cos_a
    };
}

Point3f Point3f::rotatedY(double angle_rad) const {
    double cos_a = std::cos(angle_rad);
    double sin_a = std::sin(angle_rad);
    return {
        x * cos_a + z * sin_a,
        y,
        -x * sin_a + z * cos_a
    };
}

Point3f Point3f::rotatedZ(double angle_rad) const {
    double cos_a = std::cos(angle_rad);
    double sin_a = std::sin(angle_rad);
    return {
        x * cos_a - y * sin_a,
        x * sin_a + y * cos_a,
        z
    };
}

Point3f Point3f::rotatedEuler(double yaw, double pitch, double roll) const {
    // Сначала поворот вокруг Z (yaw), потом Y (pitch), потом X (roll)
    return rotatedZ(yaw).rotatedY(pitch).rotatedX(roll);
}

Point3f Point3f::rotatedEuler(const Point3f& euler_angles) const {
    return rotatedEuler(euler_angles.z, euler_angles.y, euler_angles.x);
}

double Point3f::distanceToLine(const Point3f& line_point, const Point3f& line_dir) const {
    if (line_dir.lengthSqr() == 0) {
        // Если направляющий вектор нулевой, возвращаем расстояние до точки
        return (*this - line_point).length();
    }
    
    // Вектор от точки на прямой до нашей точки
    Point3f to_point = *this - line_point;
    
    // Расстояние = |(to_point × line_dir)| / |line_dir|
    // где × - векторное произведение
    return to_point.cross(line_dir).length() / line_dir.length();
}

double Point3f::distanceToSegment(const Point3f& segment_start, const Point3f& segment_end) const {
    Point3f segment_dir = segment_end - segment_start;
    double segment_length_sqr = segment_dir.lengthSqr();
    
    if (segment_length_sqr == 0) {
        // Отрезок вырожден в точку
        return (*this - segment_start).length();
    }
    
    // Параметр проекции точки на прямую отрезка
    // t = ( (p - start) · (end - start) ) / |end - start|^2
    Point3f to_point = *this - segment_start;
    double t = to_point.dot(segment_dir) / segment_length_sqr;
    
    if (t < 0) {
        // Ближайшая точка - начало отрезка
        return (*this - segment_start).length();
    } else if (t > 1) {
        // Ближайшая точка - конец отрезка
        return (*this - segment_end).length();
    } else {
        // Ближайшая точка внутри отрезка
        Point3f projection = segment_start + segment_dir * t;
        return (*this - projection).length();
    }
}

// Point2f operators

Point2f operator+(const Point2f &p1, const Point2f &p2) {
    return {p1.x + p2.x, p1.y + p2.y};
}

Point2f operator-(const Point2f &p1, const Point2f &p2) {
    return {p1.x - p2.x, p1.y - p2.y};
}

Point2f operator*(const Point2f &p, double scale) {
    return {p.x * scale, p.y * scale};
}

Point2f operator/(const Point2f &p, double scale) {
    return {p.x / scale, p.y / scale};
}

Point2f min(const Point2f &p1, const Point2f &p2) {
    return {std::min(p1.x, p2.x), std::min(p1.y, p2.y)};
}

Point2f max(const Point2f &p1, const Point2f &p2) {
    return {std::max(p1.x, p2.x), std::max(p1.y, p2.y)};
}

// Point3f operators
Point3f operator+(const Point3f &p1, const Point3f &p2) {
    return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z};
}

Point3f operator-(const Point3f &p1, const Point3f &p2) {
    return {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
}

Point3f operator*(const Point3f &p, double scale) {
    return {p.x * scale, p.y * scale, p.z * scale};
}

Point3f operator/(const Point3f &p, double scale) {
    return {p.x / scale, p.y / scale, p.z / scale};
}

Point3f min(const Point3f &p1, const Point3f &p2) {
    return {std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)};
}

Point3f max(const Point3f &p1, const Point3f &p2) {
    return {std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z)};
}

// PolarCoordinate

PolarCoordinate::PolarCoordinate() : mValue(0) {

}

PolarCoordinate::PolarCoordinate(double value) : mValue(value) {
    cropTo360();
}

PolarCoordinate::operator double() const {
    return mValue;
}

void PolarCoordinate::cropTo360() {
    while (mValue >= 2 * M_PI) {
        mValue -= 2 * M_PI;
    }
    while (mValue < 0) {
        mValue += 2 * M_PI;
    }
}  

PolarCoordinate& PolarCoordinate::operator+=(PolarCoordinate const& right) {
    mValue += right.mValue;
    cropTo360();
    return *this;
}

PolarCoordinate& PolarCoordinate::operator-=(PolarCoordinate const& right) {
    mValue -= right.mValue;
    cropTo360();
    return *this;
}

PolarCoordinate operator+(PolarCoordinate left, PolarCoordinate const& right) {
    left += right;
    return left;
}

PolarCoordinate operator-(PolarCoordinate left, PolarCoordinate const& right) {
    left -= right;
    return left;
}
