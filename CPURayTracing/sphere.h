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

  // moving sphere
  sphere(point3 center1, point3 center2, double radius,
         std::shared_ptr<material> mat) {
    center1_ = center1;
    center2_ = center2;
    radius_ = std::fmax(0, radius);
    mat_ = mat;
    is_moving_ = true;

    auto box_1 = aabb(center1_ - vec3(radius_), center1_ + vec3(radius_));
    auto box_2 = aabb(center2_ - vec3(radius_), center2_ + vec3(radius_));
    box_ = aabb::enclose(box_1, box_2);
  }

  aabb get_bounding_box() const { return box_; }

  // [out] record
  bool hit(const ray& r, interval interval, hit_record& record) const override {
    point3 center = point3(0, 0, 0);
    if (is_moving_) {
      center = get_center_by_time(r.time());
    } else {
      center = center_;
    }
    vec3 unit_direction = unit_vector(r.direction());
    double a = dot(r.direction(), r.direction());
    double b = 2.0 * dot(r.direction(), (r.origin() - center));
    double c =
        dot(r.origin() - center, r.origin() - center) - radius_ * radius_;
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;

    double sqrtd = std::sqrt(discriminant);
    double root = (-b - sqrtd) / (2.0 * a);
    // ���I t �������� min �� max �����A�_�h��@�S���I��
    if (!interval.is_contains(root)) {
      // ���q�p�� root �}�l��(�]�������)�A�p�G�٬O�S�I��A�A�ݥt�@�Ӥj�� root
      root = (-b + sqrtd) / (2.0 * a);
      if (!interval.is_contains(root)) {
        return false;
      }
    }

    // �w�g�簣�� t ���b�d�򤺪����p�A���U�ӴN�O�n��R hit_record
    record.t = root;
    record.p = r.at(root);
    vec3 outward_normal = (record.p - center_) / radius_;
    get_sphere_uv(record, outward_normal);
    record.set_face_normal(r, outward_normal);
    record.mat = mat_;  // ����ڤF�A�ڧi�D�A�ڪ� mat
    return true;
  }

  // ���w 0~1 ���ɶ��I�A�ݸӮɶ��I���ꤤ�ߦ�m�b��?
  vec3 get_center_by_time(double time) const {
    return center1_ + time * (center2_ - center1_);
  }

  // [out] u, v,  v ���ױq -y �� y �b�A u ���ױq -x -> z -> x -> -z -> -x
  void get_sphere_uv(hit_record& record, vec3 normal) const {
    // u
    vec3 project_u_vec = unit_vector(vec3(normal.x(), 0, normal.z()));
    double u = std::acos(dot(project_u_vec, vec3(-1, 0, 0))) / pi;
    if (project_u_vec.z() < 0) u = -u + 2;
    u = u / 2.0;
    record.u = u;
    // v
    record.v = std::acos(dot(normal, vec3(0, -1, 0))) / pi;
  }

 private:
  point3 center_;
  double radius_;
  std::shared_ptr<material> mat_;

  bool is_moving_;
  point3 center1_;  // begin
  point3 center2_;  // end

  aabb box_;
};