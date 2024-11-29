#pragma once
#include "vec3.h"
class ray {
public:
  ray() = default;
  ray(const point3 &origin, const vec3 &direction) : orig_(origin), dir_(direction), tm_(0) {}
  ray(const point3 &origin, const vec3 &direction, double time) : orig_(origin), dir_(direction), tm_(time) {}
  const point3 &origin() const { return orig_; }
  const vec3 &direction() const { return dir_; }
  double time() const { return tm_; }
  point3 at(double t) const { return orig_ + t * dir_; }

private:
  point3 orig_;
  vec3 dir_;
  double tm_; // indicating when the ray is shot
};
