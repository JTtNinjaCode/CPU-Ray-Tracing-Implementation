#pragma once
#include <cmath>
#include <iostream>

class vec3 {
 public:
  double e[3];
  vec3() : e{0, 0, 0} {}
  vec3(double ee) : e{ee, ee, ee} {}
  vec3(double e0, double e1, double e2) : e{e0, e1, e2} {}
  double x() const { return e[0]; }
  double y() const { return e[1]; }
  double z() const { return e[2]; }

  vec3 operator-() const { return {-e[0], -e[1], -e[2]}; }
  double operator[](int i) const { return e[i]; }
  double& operator[](int i) { return e[i]; }

  vec3& operator+=(const vec3& other) {
    e[0] += other.e[0];
    e[1] += other.e[1];
    e[2] += other.e[2];
    return *this;
  }
  vec3& operator-=(const vec3& other) {
    e[0] -= other.e[0];
    e[1] -= other.e[1];
    e[2] -= other.e[2];
    return *this;
  }
  vec3& operator*=(double c) {
    e[0] *= c;
    e[1] *= c;
    e[2] *= c;
    return *this;
  }
  vec3& operator/=(double c) {
    e[0] /= c;
    e[1] /= c;
    e[2] /= c;
    return *this;
  }

  double length() const {
    return std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
  }
  double length_squared() const {
    return e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
  }

  // Return true if the vector is close to zero in all dimensions.
  bool near_zero() const {
    auto s = 1e-8;
    return (std::fabs(e[0]) < s) && (std::fabs(e[1]) < s) &&
           (std::fabs(e[2]) < s);
  }
};

using point3 = vec3;

inline std::ostream& operator<<(std::ostream& out, const vec3& v) {
  out << v.x() << ' ' << v.y() << ' ' << v.z();
  return out;
}

double dot(const vec3& lhs, const vec3& rhs) {
  return lhs.x() * rhs.x() + lhs.y() * rhs.y() + lhs.z() * rhs.z();
}

vec3 operator+(const vec3& lhs, const vec3& rhs) {
  return {lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z()};
}

vec3 operator-(const vec3& lhs, const vec3& rhs) {
  return {lhs.x() - rhs.x(), lhs.y() - rhs.y(), lhs.z() - rhs.z()};
}

vec3 operator*(const vec3& lhs, const vec3& rhs) {
  return {lhs.x() * rhs.x(), lhs.y() * rhs.y(), lhs.z() * rhs.z()};
}

vec3 operator*(const vec3& lhs, double c) {
  return {lhs.x() * c, lhs.y() * c, lhs.z() * c};
}

vec3 operator*(double c, const vec3& rhs) {
  return {rhs.x() * c, rhs.y() * c, rhs.z() * c};
}

vec3 operator/(const vec3& lhs, const vec3& rhs) {
  return {lhs.x() / rhs.x(), lhs.y() / rhs.y(), lhs.z() / rhs.z()};
}

vec3 operator/(const vec3& lhs, double c) {
  return {lhs.x() / c, lhs.y() / c, lhs.z() / c};
}

vec3 unit_vector(const vec3& v) { return v / v.length(); }

vec3 cross(const vec3& lhs, const vec3& rhs) {
  return vec3(lhs.e[1] * rhs.e[2] - lhs.e[2] * rhs.e[1],
              lhs.e[2] * rhs.e[0] - lhs.e[0] * rhs.e[2],
              lhs.e[0] * rhs.e[1] - lhs.e[1] * rhs.e[0]);
}
