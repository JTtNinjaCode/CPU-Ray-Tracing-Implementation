#pragma once

// Common Headers
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include "color.h"
#include "ray.h"
#include "vec3.h"

// Constants
const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;

// Utility Functions
inline double degrees_to_radians(double degrees) { return degrees * pi / 180.0; }
// Return a random real number in [0,1).
inline double random_double() { return std::rand() / (RAND_MAX + 1.0); }
// Return a random real number in [min,max).
inline double random_double(double min, double max) { return min + (max - min) * random_double(); }
// Return a random integer in [min, max).
inline int random_int(int min, int max) { return min + (max - min) * random_double(); }
// Return a random vector in [-1,1) x [-1,1) x [-1,1).
inline vec3 random_vec() { return vec3(random_double(), random_double(), random_double()); }
// Return a random vector in [min,max) x [min,max) x [min,max).
inline vec3 random_vec(double min, double max) { return vec3(random_double(min, max), random_double(min, max), random_double(min, max)); }
// Return a random vector in unit sphere, but length not normalized.
inline vec3 random_in_unit_sphere() {
  double u1 = random_double();
  double u2 = random_double();

  double cos_theta = 1 - 2 * u1;
  double sin_theta = std::sqrt(1 - cos_theta * cos_theta);
  double phi = 2 * pi * u2; // phi is uniform in [0, 2£k]

  double x = sin_theta * std::cos(phi);
  double y = cos_theta;
  double z = sin_theta * std::sin(phi);
  return vec3{x, y, z};
}
// Return a random vector in unit sphere, length normalized.
inline vec3 random_unit_vec() { return unit_vector(random_in_unit_sphere()); }
// Return a random vector in unit disk, forward is z-axis.
inline vec3 random_in_unit_disk() {
  while (true) {
    auto p = vec3(random_double(-1, 1), random_double(-1, 1), 0);
    if (p.length_squared() < 1) return p;
  }
}
// Return a random vector in unit hemisphere, and the direction of hemisphere is specified by normal vector.
inline vec3 random_on_hemisphere(const vec3 &normal) {
  auto rand_v = random_unit_vec();
  if (dot(rand_v, normal) > 0.0)
    return rand_v;
  else
    return -rand_v;
}
inline vec3 random_cosine_direction() {
  auto r1 = random_double();
  auto r2 = random_double();
  auto phi = 2 * pi * r1;
  auto x = std::cos(phi) * std::sqrt(r2);
  auto y = std::sqrt(1 - r2);
  auto z = std::sin(phi) * std::sqrt(r2);
  return vec3(x, y, z);
}
inline vec3 reflect(const vec3 &v, const vec3 &n) { return v - 2 * dot(v, n) * n; }
inline vec3 refract(const vec3 &v, const vec3 &n, double eta) {
  auto cos_theta = std::fmin(dot(-v, n), 1.0);
  vec3 r_out_perp = eta * (v + cos_theta * n);
  vec3 r_out_parallel = -std::sqrt(std::fabs(1.0 - r_out_perp.length_squared())) * n;
  return r_out_perp + r_out_parallel;
}
inline double clamp(double x, double min, double max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}
inline double linear_interpolation(double t, double a, double b) { return (1 - t) * a + t * b; }
inline vec3 linear_interpolation(double t, const vec3 &a, const vec3 &b) { return (1 - t) * a + t * b; }