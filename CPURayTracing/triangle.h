#pragma once
#include "hittable.h"
#include "ray.h"
#include "vec3.h"
// return vec3(t, b0, b1), b0, b1 are barycentric
// b0 correspond to p1 - p0 axis
// b1 correspond to p2 - p0 axis
inline vec3 moller_trumbore(ray r, vec3 p0, vec3 p1, vec3 p2) {
  vec3 e1 = p1 - p0;
  vec3 e2 = p2 - p0;
  vec3 s = r.origin() - p0;
  vec3 s1 = cross(r.direction(), e2);
  vec3 s2 = cross(s, e1);
  return vec3(dot(s2, e2), dot(s1, s), dot(s2, r.direction())) / dot(s1, e1);
}

class triangle : public hittable {
public:
  triangle(vec3 p0, vec3 p1, vec3 p2, std::shared_ptr<material> mat) {
    p0_ = p0;
    p1_ = p1;
    p2_ = p2;
    mat_ = mat;
    normal_ = unit_vector(cross(p1 - p0, p2 - p0));
  }

  bool hit(const ray &r, interval ray_t, hit_record &record) const {
    vec3 barycentric = moller_trumbore(r, p0_, p1_, p2_);
    double t = barycentric.x();
    double b0 = barycentric.y();
    double b1 = barycentric.z();
    if (t < ray_t.min || t > ray_t.max) return false;
    // b0 + b1 <= 1 means the point is inside the triangle
    if (b0 < 0 || b1 < 0 || b0 + b1 > 1) return false;
    record.t = t;
    record.p = r.at(t);
    record.set_face_normal(r, normal_);
    record.mat = mat_;
    return true;
  }

  aabb get_bounding_box() const {
    vec3 min = vec3(std::fmin(p0_.x(), std::fmin(p1_.x(), p2_.x())), std::fmin(p0_.y(), std::fmin(p1_.y(), p2_.y())),
                    std::fmin(p0_.z(), std::fmin(p1_.z(), p2_.z())));
    vec3 max = vec3(std::fmax(p0_.x(), std::fmax(p1_.x(), p2_.x())), std::fmax(p0_.y(), std::fmax(p1_.y(), p2_.y())),
                    std::fmax(p0_.z(), std::fmax(p1_.z(), p2_.z())));
    return aabb(min, max);
  }

private:
  vec3 p0_;
  vec3 p1_;
  vec3 p2_;
  vec3 normal_;
  std::shared_ptr<material> mat_;
};