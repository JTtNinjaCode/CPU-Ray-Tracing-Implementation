#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
// Common Headers
#include "color.h"
#include "ray.h"
#include "vec3.h"
// Constants
const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;
// Utility Functions
inline double degrees_to_radians(double degrees) {
  return degrees * pi / 180.0;
}
// Returns a random real in [0,1).
inline double random_double() { return std::rand() / (RAND_MAX + 1.0); }
// Returns a random real in [min,max).
inline double random_double(double min, double max) {
  return min + (max - min) * random_double();
}
// Returns a random int in [min, max).
inline int random_int(int min, int max) {
  return min + (max - min) * random_double();
}
// �H������ vec3([-1, 1],[-1, 1],[-1, 1]) ���H���V�q
inline vec3 random_vec() {
  return vec3(random_double(), random_double(), random_double());
}
// �H������ vec3([min, max],[min, max],[min, max]) ���H���V�q
inline vec3 random_vec(double min, double max) {
  return vec3(random_double(min, max), random_double(min, max),
              random_double(min, max));
}
// �H�����ͳ��y�餺���N�V�q�A���פ��@�w���@
inline vec3 random_in_unit_sphere() {
  while (true) {
    vec3 rand_v = random_vec(-1, 1);
    if (rand_v.length_squared() < 1.0) return rand_v;
  }
}
// �H�����ͳ��y�餺���N�V�q�A���פ@�w���@
inline vec3 random_uint_vec() { return unit_vector(random_in_unit_sphere()); }

// �H�����ͳ��ꤺ���N�V�q�A���פ��@�w���@�Az �T�w�� 0
inline vec3 random_in_unit_disk() {
  while (true) {
    auto p = vec3(random_double(-1, 1), random_double(-1, 1), 0);
    if (p.length_squared() < 1) return p;
  }
}
// �H�����ͥb�ꤺ�����N�V�q�A�b�ꪺ��V�Ϊk�V�q���w�A���פ@�w���@
inline vec3 random_on_hemisphere(const vec3& normal) {
  auto rand_v = random_uint_vec();
  if (dot(rand_v, normal) > 0.0)
    return rand_v;
  else
    return -rand_v;
}

inline vec3 reflect(const vec3& v, const vec3& n) {
  return v - 2 * dot(v, n) * n;
}

inline vec3 refract(const vec3& v, const vec3& n, double eta) {
  auto cos_theta = std::fmin(dot(-v, n), 1.0);
  vec3 r_out_perp = eta * (v + cos_theta * n);
  vec3 r_out_parallel =
      -std::sqrt(std::fabs(1.0 - r_out_perp.length_squared())) * n;
  return r_out_perp + r_out_parallel;
}

inline double clamp(double x, double min, double max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

inline double linear_interpolation(double t, double a, double b) {
  return (1 - t) * a + t * b;
}