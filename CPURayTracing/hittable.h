#pragma once

#include "interval.h"
#include "ray.h"

class material;
class hit_record {
 public:
  point3 p;     // ���쪺�I
  vec3 normal;  // ���@�w�� unit vector
  double t;
  double u;
  double v;
  bool front_face;  // norm ���¦V�Atrue �����Afalse �I��

  // ����u������@�Ӫ��]�Ҧp�Y�ӯS�w���y��^�ɡAhit_record ���� material
  // ���w�|�Q�]�m�����V�b main() ���]�m�y��ɤ��t������ material ���w�C��
  // ray_color() ������ hit_record �ɡA���i�H�ե� material
  // ���w��������ƨӬd����u�O�_�Q���g�A�Y���A�h���Q���g�����u�C���F��{�o�@�I�Ahit_record
  // �ݭn���D���t���y�骺���ơC
  std::shared_ptr<material> mat;

  // �p�G ray.direction, outward_normal ���n�p�� 0�A�N�� outward_normal �O
  // frontface �p�G ray.direction, outward_normal ���n�j�� 0�A�N��
  // outward_normal �O insideface
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

// translate ����z�N�O�� ray ���q world space �ഫ�� object space�A�A�� hit
// test�A�̫�A�� hit record ����m�ഫ�^ world
class translate : public hittable {
 public:
  translate(vec3 offset, std::shared_ptr<hittable> object) {
    offset_ = offset;
    object_ = object;
  }

  // ���ʪ����O����A�ӬO ray
  bool hit(const ray& r, interval ray_t, hit_record& record) const override {
    ray offset_r = ray(r.origin() - offset_, r.direction(), r.time());
    if (!object_->hit(offset_r, ray_t, record)) {
      return false;
    }
    record.p += offset_;  // �վ�^�u�� hit �쪺��m
    return true;
  }

  aabb get_bounding_box() const override {
    return object_->get_bounding_box().offset(offset_);
  }

 private:
  std::shared_ptr<hittable> object_;
  vec3 offset_;
};


// rotate ����z�N�O�� ray ���q world space �ഫ�� object space�A�A�� hit
// test�A�̫�A�� hit record ����m�ഫ�^ world
class rotate_x : public hittable {
 public:
  rotate_x(std::shared_ptr<hittable> object, double angle) : object(object) {
    auto radians = degrees_to_radians(angle);
    sin_theta = std::sin(radians);
    cos_theta = std::cos(radians);

    // �H�U���O�b�]�w bounding box
    bbox = object->get_bounding_box();
    point3 min(infinity, infinity, infinity);
    point3 max(-infinity, -infinity, -infinity);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          // x, y, z �H�ۤK���j��|���O���� bounding box �� 8 ���I
          auto x = i * bbox.axis_interval(0).max +
                   (1 - i) * bbox.axis_interval(0).min;
          auto y = j * bbox.axis_interval(1).max +
                   (1 - j) * bbox.axis_interval(1).min;
          auto z = k * bbox.axis_interval(2).max +
                   (1 - k) * bbox.axis_interval(2).min;
          auto newy = cos_theta * y + sin_theta * z;
          auto newz = -sin_theta * y + cos_theta * z;
          vec3 tester(x, newy, newz);  // ����᪺�I
          // ����� bounding box �i��d��|�ܡA�]�����s�p�� bounding box
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
    // �� ray �q world space �ഫ�� object space(�]�N�O���Ϥ�V����)
    auto origin = r.origin();
    auto direction = r.direction();
    origin[1] = cos_theta * r.origin()[1] - sin_theta * r.origin()[2];
    origin[2] = sin_theta * r.origin()[1] + cos_theta * r.origin()[2];
    direction[1] = cos_theta * r.direction()[1] - sin_theta * r.direction()[2];
    direction[2] = sin_theta * r.direction()[1] + cos_theta * r.direction()[2];

    ray rotated_r(origin, direction, r.time());
    if (!object->hit(rotated_r, ray_t, rec)) return false;

    // �� ray �q object space �ഫ�� world space(�]�N�O���Ϥ�V����)
    auto p = rec.p;
    p[1] = cos_theta * rec.p[1] + sin_theta * rec.p[2];
    p[2] = -sin_theta * rec.p[1] + cos_theta * rec.p[2];

    // normal �n�q object space �ഫ�� world space
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

    // �H�U���O�b�]�w bounding box
    bbox = object->get_bounding_box();
    point3 min(infinity, infinity, infinity);
    point3 max(-infinity, -infinity, -infinity);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          // x, y, z �H�ۤK���j��|���O���� bounding box �� 8 ���I
          auto x = i * bbox.axis_interval(0).max +
                   (1 - i) * bbox.axis_interval(0).min;
          auto y = j * bbox.axis_interval(1).max +
                   (1 - j) * bbox.axis_interval(1).min;
          auto z = k * bbox.axis_interval(2).max +
                   (1 - k) * bbox.axis_interval(2).min;
          auto newx = cos_theta * x + sin_theta * z;
          auto newz = -sin_theta * x + cos_theta * z;
          vec3 tester(newx, y, newz);  // ����᪺�I
          // ����� bounding box �i��d��|�ܡA�]�����s�p�� bounding box
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
    // �� ray �q world space �ഫ�� object space(�]�N�O���Ϥ�V����)
    auto origin = r.origin();
    auto direction = r.direction();
    origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
    origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];
    direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
    direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

    ray rotated_r(origin, direction, r.time());
    if (!object->hit(rotated_r, ray_t, rec)) return false;

    // �� ray �q object space �ഫ�� world space(�]�N�O���Ϥ�V����)
    auto p = rec.p;
    p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[2];
    p[2] = -sin_theta * rec.p[0] + cos_theta * rec.p[2];

    // normal �n�q object space �ഫ�� world space
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

    // �H�U���O�b�]�w bounding box
    bbox = object->get_bounding_box();
    point3 min(infinity, infinity, infinity);
    point3 max(-infinity, -infinity, -infinity);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          // x, y, z �H�ۤK���j��|���O���� bounding box �� 8 ���I
          auto x = i * bbox.axis_interval(0).max +
                   (1 - i) * bbox.axis_interval(0).min;
          auto y = j * bbox.axis_interval(1).max +
                   (1 - j) * bbox.axis_interval(1).min;
          auto z = k * bbox.axis_interval(2).max +
                   (1 - k) * bbox.axis_interval(2).min;
          auto newx = cos_theta * x + sin_theta * y;
          auto newy = -sin_theta * x + cos_theta * y;
          vec3 tester(newx, newy, z);  // ����᪺�I
          // ����� bounding box �i��d��|�ܡA�]�����s�p�� bounding box
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
    // �� ray �q world space �ഫ�� object space(�]�N�O���Ϥ�V����)
    auto origin = r.origin();
    auto direction = r.direction();

    origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[1];
    origin[1] = sin_theta * r.origin()[0] + cos_theta * r.origin()[1];
    direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[1];
    direction[1] = sin_theta * r.direction()[0] + cos_theta * r.direction()[1];

    ray rotated_r(origin, direction, r.time());
    if (!object->hit(rotated_r, ray_t, rec)) return false;

    // �� ray �q object space �ഫ�� world space(�]�N�O���Ϥ�V����)
    auto p = rec.p;
    p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[1];
    p[1] = -sin_theta * rec.p[0] + cos_theta * rec.p[1];
    
    // normal �n�q object space �ഫ�� world space
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