#pragma once
#include "hittable.h"

class sphere : public hittable {
public:
  // stationary sphere
  sphere(point3 center, double radius, std::shared_ptr<material> mat) {
    center_ = center;
    radius_ = std::fmax(0, radius);
    mat_ = mat;
    is_moving_ = false;
    box_ = aabb(center - vec3(radius), center + vec3(radius));
  }

  // stationary sphere
  sphere(double radius, std::shared_ptr<material> mat) {
    center_ = point3(0);
    radius_ = std::fmax(0, radius);
    mat_ = mat;
    is_moving_ = false;
    box_ = aabb(center_ - vec3(radius), center_ + vec3(radius));
  }

  // moving sphere
  sphere(point3 center1, point3 center2, double radius, std::shared_ptr<material> mat) {
    begin_center_ = center1;
    end_center_ = center2;
    radius_ = std::fmax(0, radius);
    mat_ = mat;
    is_moving_ = true;

    auto box_1 = aabb(begin_center_ - vec3(radius_), begin_center_ + vec3(radius_));
    auto box_2 = aabb(end_center_ - vec3(radius_), end_center_ + vec3(radius_));
    box_ = aabb::enclose(box_1, box_2);
  }

  aabb get_bounding_box() const { return box_; }

  // [out] record
  bool hit(const ray &r, interval interval, hit_record &record) const override {
    point3 center = point3(0, 0, 0);
    if (is_moving_) {
      center = get_center_by_time(r.time());
    } else {
      center = center_;
    }
    vec3 unit_direction = unit_vector(r.direction());
    double a = dot(r.direction(), r.direction());
    double b = 2.0 * dot(r.direction(), (r.origin() - center));
    double c = dot(r.origin() - center, r.origin() - center) - radius_ * radius_;
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;

    double sqrtd = std::sqrt(discriminant);
    double root = (-b - sqrtd) / (2.0 * a);
    // make sure t is between min and max, otherwise treat as no hit
    if (!interval.is_contains(root)) {
      // 先從小的 root 開始看(因為比較近)，如果還是沒碰到，再看另一個大的 root
      root = (-b + sqrtd) / (2.0 * a);
      if (!interval.is_contains(root)) {
        return false;
      }
    }

    // Fill the hit_record
    record.t = root;
    record.p = r.at(root);
    vec3 outward_normal = (record.p - center_) / radius_;
    get_sphere_uv(outward_normal, record.u, record.v);
    record.set_face_normal(r, outward_normal);
    record.mat = mat_;
    return true;
  }

  double pdf_value(const point3 &origin, const vec3 &direction) const {
    return radius_ * radius_ * pi / (origin - center_).length_squared();
  }

  // origin 往 hittable 的方向上產生隨機的 direction
  vec3 random(const point3 &origin) const { return random_in_unit_sphere() * radius_; }

  vec3 get_center_by_time(double time) const { return begin_center_ + time * (end_center_ - begin_center_); }

  // [out] u, v
  // Calculates the texture coordinates (u, v) for a point on a sphere based on its normal vector.
  // The u coordinate is computed based on the projection of the normal vector onto the xz-plane,
  // and the v coordinate is calculated using the angle between the normal and the negative y-axis.
  // v 角度從 -y 到 y 軸， u 角度從 -x -> z -> x -> -z -> -x
  void get_sphere_uv(const point3 &p, double &u, double &v) const {
    auto theta = std::acos(-p.y());
    auto phi = std::atan2(-p.z(), p.x()) + pi;
    u = phi / (2 * pi);
    v = theta / pi;
  }

private:
  point3 center_;
  double radius_;
  std::shared_ptr<material> mat_;

  bool is_moving_;
  point3 begin_center_;
  point3 end_center_;

  aabb box_;
};