#pragma once
#include "interval.h"
// 建構時給定範圍，並且透過 hit 檢查是否有碰到這個 aabb
class aabb {
 public:
  aabb() {}
  aabb(interval x, interval y, interval z) {
    x_ = x;
    y_ = y;
    z_ = z;
    pad_to_minimums();
  }
  // Treat the two points a and b as extrema for the bounding box, so we don't
  // require a particular minimum/maximum coordinate order.
  aabb(const point3& a, const point3& b) {
    x_ = (a[0] <= b[0]) ? interval(a[0], b[0]) : interval(b[0], a[0]);
    y_ = (a[1] <= b[1]) ? interval(a[1], b[1]) : interval(b[1], a[1]);
    z_ = (a[2] <= b[2]) ? interval(a[2], b[2]) : interval(b[2], a[2]);
  }

  const interval& axis_interval(int n) const {
    if (n == 1) return y_;
    if (n == 2) return z_;
    return x_;
  }

  // 檢查 ray 的區間 ray_t 是否有碰到 aabb
  bool hit(const ray& r, interval ray_t) const {
    interval int1 = compute_instersection_x(r, x_.min, x_.max);
    interval int2 = compute_instersection_y(r, y_.min, y_.max);
    interval int3 = compute_instersection_z(r, z_.min, z_.max);
    return overlaps(int1, int2, int3, ray_t);
  }

  static aabb enclose(const aabb& box1, const aabb& box2) {
    interval new_interval_x = interval::enclose(box1.x_, box2.x_);
    interval new_interval_y = interval::enclose(box1.y_, box2.y_);
    interval new_interval_z = interval::enclose(box1.z_, box2.z_);
    return aabb(new_interval_x, new_interval_y, new_interval_z);
  }

  aabb offset(const vec3& offset) const {
    return aabb(x_.offset(offset.x()), y_.offset(offset.y()),
                z_.offset(offset.z()));
  }

 private:
  interval compute_instersection_x(ray r, double x1, double x2) const {
    double t1 = (x1 - r.origin().x()) / r.direction().x();
    double t2 = (x2 - r.origin().x()) / r.direction().x();
    return {std::min(t1, t2), std::max(t1, t2)};
  }
  interval compute_instersection_y(ray r, double y1, double y2) const {
    double t1 = (y1 - r.origin().y()) / r.direction().y();
    double t2 = (y2 - r.origin().y()) / r.direction().y();
    return {std::min(t1, t2), std::max(t1, t2)};
  }
  interval compute_instersection_z(ray r, double z1, double z2) const {
    double t1 = (z1 - r.origin().z()) / r.direction().z();
    double t2 = (z2 - r.origin().z()) / r.direction().z();
    return {std::min(t1, t2), std::max(t1, t2)};
  }

  // 檢查三個 interval 的 t 區間是否有覆蓋，有就代表有碰到
  bool overlaps(interval int1, interval int2, interval int3,
                interval ray_t) const {
    double t_min = std::max(std::max(int1.min, int2.min), int3.min);
    double t_max = std::min(std::min(int1.max, int2.max), int3.max);

    if (t_min > ray_t.min) ray_t.min = t_min;
    if (t_max < ray_t.max) ray_t.max = t_max;
    return ray_t.min < ray_t.max;
  }

  static const aabb empty, universe;

 private:
  interval x_;
  interval y_;
  interval z_;

 private:
  // Adjust the AABB so that no side is narrower than some delta, padding if
  // necessary.
  void pad_to_minimums() {
    double delta = 0.0001;
    if (x_.size() < delta) x_ = x_.expand(delta);
    if (y_.size() < delta) y_ = y_.expand(delta);
    if (z_.size() < delta) z_ = z_.expand(delta);
  }
};

const aabb aabb::empty =
    aabb(interval::empty, interval::empty, interval::empty);
const aabb aabb::universe =
    aabb(interval::universe, interval::universe, interval::universe);
