#pragma once
#include <algorithm>

#include "aabb.h"
#include "hittable.h"
#include "hittable_list.h"
// bvh_node 是 hittable
// 其他物件也是 hittable
// 他們都可以構成 BVH Tree
// BVH Tree 的元素可以有 BVH Node 以及 Sphere 等物件，因為他們都是 hittable
class bvh_node : public hittable {
public:
  bvh_node(hittable_list list) : bvh_node(list.objects, 0, list.objects.size()) {
    {}
  }

  // 負責把一堆 hittable object 建構成 BVH，begin_i, end_i 是前閉後開
  bvh_node(std::vector<std::shared_ptr<hittable>> &objects, size_t begin_i, size_t end_i) {
    //  0 代表以 x 分，1 代表以 y 分，2 代表以 z 分
    // int split_index = random_int(0, 3);
    int split_index = 0;
    auto comp = &hittable::compare_x;
    if (split_index == 0) {
      comp = hittable::compare_x;
    } else if (split_index == 1) {
      comp = hittable::compare_y;
    } else if (split_index == 2) {
      comp = hittable::compare_z;
    }
    // 排序以後，從中間分開，並且
    std::sort(objects.begin() + begin_i, objects.begin() + end_i, comp);

    int count = end_i - begin_i;
    if (count == 1) {
      // 左右都要指向同一個，不能是 nullptr，因為這裡沒有 nullptr 處理
      left_ = right_ = objects[begin_i];
    } else if (count == 2) {
      left_ = objects[begin_i];
      right_ = objects[begin_i + 1];
    } else {
      // split again
      size_t mid_i = (begin_i + end_i) / 2;
      left_ = std::make_shared<bvh_node>(objects, begin_i, mid_i);
      right_ = std::make_shared<bvh_node>(objects, mid_i, end_i);
    }
    box_ = aabb::enclose(left_->get_bounding_box(), right_->get_bounding_box());
  }

  bool hit(const ray &r, interval ray_t, hit_record &record) const override {
    if (box_.hit(r, ray_t)) {
      bool hit_left = left_->hit(r, ray_t, record);
      // 如果碰到了，會更新 record.t
      auto update_ray_t = interval(ray_t.min, hit_left ? record.t : ray_t.max);
      bool hit_right = right_->hit(r, update_ray_t, record);

      return hit_left || hit_right;
    };
    return false;
  }
  aabb get_bounding_box() const override { return box_; }

private:
  std::shared_ptr<hittable> left_;
  std::shared_ptr<hittable> right_;
  aabb box_;
};