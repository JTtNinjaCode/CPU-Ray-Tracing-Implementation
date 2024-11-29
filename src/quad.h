#pragma once
#include "aabb.h"
#include "hittable.h"
#include "material.h"
#include "vec3.h"

class quad : public hittable {
public:
  quad(point3 corner, vec3 u, vec3 v, std::shared_ptr<material> mat) {
    corner_ = corner;
    u_ = u;
    v_ = v;
    // define quad's front face as the direction of cross(u, v)
    vec3 norm = cross(u, v);
    unorm_ = unit_vector(norm);
    mat_ = mat;

    area_ = norm.length();

    auto box1 = aabb(corner_, corner_ + u + v);
    auto box2 = aabb(corner_ + u, corner_ + v);
    box_ = aabb::enclose(box1, box2);
  }

  // 平面方程式 a(x - x0) + b(y - y0) + c(z - z0) = 0
  // (a, b, c) 是法向量，(x0, y0, z0) 是 平面上某點
  // 分配律及移項拆解後 ax + by + cz = a x0 + b y0 + c z0
  // dot(norm_, x) = dot(norm_, corner_)
  // dot(norm_, p + t * d) = dot(norm_, corner_)
  bool hit(const ray &r, interval ray_t, hit_record &record) const override {
    // 使用平面方程式算出 t，得到交點
    double d = dot(unorm_, corner_);
    double t = (d - dot(unorm_, r.origin())) / dot(unorm_, r.direction());
    if (!ray_t.is_contains(t)) // check if t is in the interval
      return false;
    // hit the plane, then generate the uv coordinates of the hit point in the quad, which are (hit_point_u, hit_point_v)
    point3 hit_point = r.at(t);
    vec3 p = hit_point - corner_;
    vec3 n = cross(u_, v_); // not always unit vec
    vec3 w = n / dot(n, n);

    double hit_point_u = dot(w, cross(p, v_));
    double hit_point_v = dot(w, cross(u_, p));

    if (!is_interior(hit_point_u, hit_point_v, record)) return false;
    // hit the quad! fill the hit record
    record.t = t;
    record.p = hit_point;
    record.mat = mat_;
    record.set_face_normal(r, unorm_);
    return true;
  }

  aabb get_bounding_box() const override { return box_; }

  // Given the hit point in plane coordinates, return false if it is outside the primitive, otherwise set the hit record UV coordinates and
  // return true.
  virtual bool is_interior(double a, double b, hit_record &rec) const {
    interval unit_interval = interval(0, 1);
    if (!unit_interval.is_contains(a) || !unit_interval.is_contains(b)) return false;
    rec.u = a;
    rec.v = b;
    return true;
  }

  double pdf_value(const point3 &origin, const vec3 &direction) const override {
    hit_record rec;
    // If hit this quad, calculate its pdf weight that projected to the hemisphere
    if (!this->hit(ray(origin, direction), interval(0.001, infinity), rec)) return 0;
    auto distance_squared = rec.t * rec.t * direction.length_squared();
    auto cosine = std::fabs(dot(unit_vector(direction), rec.normal));
    return distance_squared / (cosine * area_);
  }

  vec3 random(const point3 &origin) const override {
    auto p = corner_ + (random_double() * u_) + (random_double() * v_);
    return p - origin;
  }

private:
  point3 corner_;
  vec3 u_;
  vec3 v_;
  vec3 unorm_;
  aabb box_;
  std::shared_ptr<material> mat_;
  double area_;
};

// Returns the 3D box (six sides) that contains the two opposite vertices a & b.
inline std::shared_ptr<hittable_list> box(const point3 &a, const point3 &b, std::shared_ptr<material> mat) {
  auto sides = std::make_shared<hittable_list>();
  // Construct the two opposite vertices with the minimum and maximum coordinates.
  auto min = point3(std::fmin(a.x(), b.x()), std::fmin(a.y(), b.y()), std::fmin(a.z(), b.z()));
  auto max = point3(std::fmax(a.x(), b.x()), std::fmax(a.y(), b.y()), std::fmax(a.z(), b.z()));
  auto dx = vec3(max.x() - min.x(), 0, 0);
  auto dy = vec3(0, max.y() - min.y(), 0);
  auto dz = vec3(0, 0, max.z() - min.z());
  // front
  sides->push_back(std::make_shared<quad>(point3(min.x(), min.y(), max.z()), dy, dx, mat));
  // right
  sides->push_back(std::make_shared<quad>(point3(max.x(), min.y(), max.z()), dy, -dz, mat));
  // back
  sides->push_back(std::make_shared<quad>(point3(max.x(), min.y(), min.z()), dy, -dx, mat));
  // left
  sides->push_back(std::make_shared<quad>(point3(min.x(), min.y(), min.z()), dy, dz, mat));
  // top
  sides->push_back(std::make_shared<quad>(point3(min.x(), max.y(), max.z()), -dz, dx, mat));
  // bottom
  sides->push_back(std::make_shared<quad>(point3(min.x(), min.y(), min.z()), dz, dx, mat));
  return sides;
}