#pragma once

#include "interval.h"
#include "ray.h"

class material;
class hit_record {
 public:
  point3 p;     // 打到的點
  vec3 normal;  // 不一定為 unit vector
  double t;
  double u;
  double v;
  bool front_face;  // norm 的朝向，true 正面，false 背面

  // 當光線撞擊到一個表面（例如某個特定的球體）時，hit_record 中的 material
  // 指針會被設置為指向在 main() 中設置球體時分配給它的 material 指針。當
  // ray_color() 函數獲取 hit_record 時，它可以調用 material
  // 指針的成員函數來查找光線是否被散射，若有，則找到被散射的光線。為了實現這一點，hit_record
  // 需要知道分配給球體的材料。
  std::shared_ptr<material> mat;

  // 如果 ray.direction, outward_normal 內積小於 0，代表 outward_normal 是
  // frontface 如果 ray.direction, outward_normal 內積大於 0，代表
  // outward_normal 是 insideface
  void set_face_normal(const ray& r, const vec3& outward_normal) {
    front_face = dot(r.direction(), outward_normal) < 0.0;
    normal = front_face ? outward_normal : -outward_normal;
  }
};

class hittable {
 public:
  virtual ~hittable() = default;
  virtual bool hit(const ray& r, interval ray_t, hit_record& record) const = 0;
  virtual aabb get_bounding_box() const = 0;

  static bool compare_x(std::shared_ptr<hittable> h1,
                        std::shared_ptr<hittable> h2) {
    auto h1_x_interval = h1->get_bounding_box().axis_interval(0);
    auto h2_x_interval = h2->get_bounding_box().axis_interval(0);
    if (h1_x_interval.min < h2_x_interval.min) return true;
    return false;
  }

  static bool compare_y(std::shared_ptr<hittable> h1,
                        std::shared_ptr<hittable> h2) {
    auto h1_y_interval = h1->get_bounding_box().axis_interval(1);
    auto h2_y_interval = h2->get_bounding_box().axis_interval(1);
    if (h1_y_interval.min < h2_y_interval.min) return true;
    return false;
  }

  static bool compare_z(std::shared_ptr<hittable> h1,
                        std::shared_ptr<hittable> h2) {
    auto h1_z_interval = h1->get_bounding_box().axis_interval(2);
    auto h2_z_interval = h2->get_bounding_box().axis_interval(2);
    if (h1_z_interval.min < h2_z_interval.min) return true;
    return false;
  }
};

// translate 的原理就是把 ray 先從 world space 轉換到 object space，再做 hit
// test，最後再把 hit record 的位置轉換回 world
class translate : public hittable {
 public:
  translate(vec3 offset, std::shared_ptr<hittable> object) {
    offset_ = offset;
    object_ = object;
  }

  // 移動的不是物件，而是 ray
  bool hit(const ray& r, interval ray_t, hit_record& record) const override {
    ray offset_r = ray(r.origin() - offset_, r.direction(), r.time());
    if (!object_->hit(offset_r, ray_t, record)) {
      return false;
    }
    record.p += offset_;  // 調整回真正 hit 到的位置
    return true;
  }

  aabb get_bounding_box() const override {
    return object_->get_bounding_box().offset(offset_);
  }

 private:
  std::shared_ptr<hittable> object_;
  vec3 offset_;
};


// rotate 的原理就是把 ray 先從 world space 轉換到 object space，再做 hit
// test，最後再把 hit record 的位置轉換回 world
class rotate_x : public hittable {
 public:
  rotate_x(std::shared_ptr<hittable> object, double angle) : object(object) {
    auto radians = degrees_to_radians(angle);
    sin_theta = std::sin(radians);
    cos_theta = std::cos(radians);

    // 以下都是在設定 bounding box
    bbox = object->get_bounding_box();
    point3 min(infinity, infinity, infinity);
    point3 max(-infinity, -infinity, -infinity);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          // x, y, z 隨著八次迴圈會分別對應 bounding box 的 8 個點
          auto x = i * bbox.axis_interval(0).max +
                   (1 - i) * bbox.axis_interval(0).min;
          auto y = j * bbox.axis_interval(1).max +
                   (1 - j) * bbox.axis_interval(1).min;
          auto z = k * bbox.axis_interval(2).max +
                   (1 - k) * bbox.axis_interval(2).min;
          auto newy = cos_theta * y + sin_theta * z;
          auto newz = -sin_theta * y + cos_theta * z;
          vec3 tester(x, newy, newz);  // 旋轉後的點
          // 旋轉後 bounding box 可能範圍會變，因此重新計算 bounding box
          for (int c = 0; c < 3; c++) {
            min[c] = std::fmin(min[c], tester[c]);
            max[c] = std::fmax(max[c], tester[c]);
          }
        }
      }
    }
    bbox = aabb(min, max);
  }

  bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
    // 把 ray 從 world space 轉換到 object space(也就是往反方向旋轉)
    auto origin = r.origin();
    auto direction = r.direction();
    origin[1] = cos_theta * r.origin()[1] - sin_theta * r.origin()[2];
    origin[2] = sin_theta * r.origin()[1] + cos_theta * r.origin()[2];
    direction[1] = cos_theta * r.direction()[1] - sin_theta * r.direction()[2];
    direction[2] = sin_theta * r.direction()[1] + cos_theta * r.direction()[2];

    ray rotated_r(origin, direction, r.time());
    if (!object->hit(rotated_r, ray_t, rec)) return false;

    // 把 ray 從 object space 轉換到 world space(也就是往反方向旋轉)
    auto p = rec.p;
    p[1] = cos_theta * rec.p[1] + sin_theta * rec.p[2];
    p[2] = -sin_theta * rec.p[1] + cos_theta * rec.p[2];

    // normal 要從 object space 轉換到 world space
    auto normal = rec.normal;
    normal[1] = cos_theta * rec.normal[1] + sin_theta * rec.normal[2];
    normal[2] = -sin_theta * rec.normal[1] + cos_theta * rec.normal[2];
    rec.p = p;
    rec.normal = normal;
    return true;
  }

  aabb get_bounding_box() const override { return bbox; }

 private:
  std::shared_ptr<hittable> object;
  double sin_theta;
  double cos_theta;
  aabb bbox;
};

class rotate_y : public hittable {
 public:
  rotate_y(std::shared_ptr<hittable> object, double angle) : object(object) {
    auto radians = degrees_to_radians(angle);
    sin_theta = std::sin(radians);
    cos_theta = std::cos(radians);

    // 以下都是在設定 bounding box
    bbox = object->get_bounding_box();
    point3 min(infinity, infinity, infinity);
    point3 max(-infinity, -infinity, -infinity);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          // x, y, z 隨著八次迴圈會分別對應 bounding box 的 8 個點
          auto x = i * bbox.axis_interval(0).max +
                   (1 - i) * bbox.axis_interval(0).min;
          auto y = j * bbox.axis_interval(1).max +
                   (1 - j) * bbox.axis_interval(1).min;
          auto z = k * bbox.axis_interval(2).max +
                   (1 - k) * bbox.axis_interval(2).min;
          auto newx = cos_theta * x + sin_theta * z;
          auto newz = -sin_theta * x + cos_theta * z;
          vec3 tester(newx, y, newz);  // 旋轉後的點
          // 旋轉後 bounding box 可能範圍會變，因此重新計算 bounding box
          for (int c = 0; c < 3; c++) {
            min[c] = std::fmin(min[c], tester[c]);
            max[c] = std::fmax(max[c], tester[c]);
          }
        }
      }
    }
    bbox = aabb(min, max);
  }

  bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
    // 把 ray 從 world space 轉換到 object space(也就是往反方向旋轉)
    auto origin = r.origin();
    auto direction = r.direction();
    origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
    origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];
    direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
    direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

    ray rotated_r(origin, direction, r.time());
    if (!object->hit(rotated_r, ray_t, rec)) return false;

    // 把 ray 從 object space 轉換到 world space(也就是往反方向旋轉)
    auto p = rec.p;
    p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[2];
    p[2] = -sin_theta * rec.p[0] + cos_theta * rec.p[2];

    // normal 要從 object space 轉換到 world space
    auto normal = rec.normal;
    normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
    normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];
    rec.p = p;
    rec.normal = normal;
    return true;
  }

  aabb get_bounding_box() const override { return bbox; }

 private:
  std::shared_ptr<hittable> object;
  double sin_theta;
  double cos_theta;
  aabb bbox;
};

class rotate_z : public hittable {
 public:
  rotate_z(std::shared_ptr<hittable> object, double angle) : object(object) {
    auto radians = degrees_to_radians(angle);
    sin_theta = std::sin(radians);
    cos_theta = std::cos(radians);

    // 以下都是在設定 bounding box
    bbox = object->get_bounding_box();
    point3 min(infinity, infinity, infinity);
    point3 max(-infinity, -infinity, -infinity);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          // x, y, z 隨著八次迴圈會分別對應 bounding box 的 8 個點
          auto x = i * bbox.axis_interval(0).max +
                   (1 - i) * bbox.axis_interval(0).min;
          auto y = j * bbox.axis_interval(1).max +
                   (1 - j) * bbox.axis_interval(1).min;
          auto z = k * bbox.axis_interval(2).max +
                   (1 - k) * bbox.axis_interval(2).min;
          auto newx = cos_theta * x + sin_theta * y;
          auto newy = -sin_theta * x + cos_theta * y;
          vec3 tester(newx, newy, z);  // 旋轉後的點
          // 旋轉後 bounding box 可能範圍會變，因此重新計算 bounding box
          for (int c = 0; c < 3; c++) {
            min[c] = std::fmin(min[c], tester[c]);
            max[c] = std::fmax(max[c], tester[c]);
          }
        }
      }
    }
    bbox = aabb(min, max);
  }

  bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
    // 把 ray 從 world space 轉換到 object space(也就是往反方向旋轉)
    auto origin = r.origin();
    auto direction = r.direction();

    origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[1];
    origin[1] = sin_theta * r.origin()[0] + cos_theta * r.origin()[1];
    direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[1];
    direction[1] = sin_theta * r.direction()[0] + cos_theta * r.direction()[1];

    ray rotated_r(origin, direction, r.time());
    if (!object->hit(rotated_r, ray_t, rec)) return false;

    // 把 ray 從 object space 轉換到 world space(也就是往反方向旋轉)
    auto p = rec.p;
    p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[1];
    p[1] = -sin_theta * rec.p[0] + cos_theta * rec.p[1];
    
    // normal 要從 object space 轉換到 world space
    auto normal = rec.normal;
    normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[1];
    normal[1] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[1];
    rec.p = p;
    rec.normal = normal;
    return true;
  }

  aabb get_bounding_box() const override { return bbox; }

 private:
  std::shared_ptr<hittable> object;
  double sin_theta;
  double cos_theta;
  aabb bbox;
};