#pragma once
#include <memory>
#include <vector>

#include "hittable.h"
class hittable_list : public hittable {
 public:
  std::vector<std::shared_ptr<hittable>> objects;

  hittable_list() = default;
  hittable_list(std::shared_ptr<hittable> object) { push_back(object); }
  void clear() { objects.clear(); }
  void push_back(std::shared_ptr<hittable> object) {
    objects.push_back(object);
    box_ = aabb::enclose(box_, object->get_bounding_box());
  }

  // 跑過所有的 object，得到 hit 最近的點
  bool hit(const ray& r, interval interval, hit_record& rec) const override {
    hit_record temp_rec;
    bool hit_anything = false;
    for (const auto& object : objects) {
      if (object->hit(r, interval, temp_rec)) {
        hit_anything = true;
        interval.max = temp_rec.t;
        rec = temp_rec;
      }
    }
    return hit_anything;
  }

  aabb get_bounding_box() const override { return box_; }

 private:
  aabb box_;
};