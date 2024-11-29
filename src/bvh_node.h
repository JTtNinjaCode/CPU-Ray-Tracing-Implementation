#pragma once
#include <algorithm>

#include "aabb.h"
#include "hittable.h"
#include "hittable_list.h"
// bvh_node �O hittable
// ��L����]�O hittable
// �L�̳��i�H�c�� BVH Tree
// BVH Tree �������i�H�� BVH Node �H�� Sphere ������A�]���L�̳��O hittable
class bvh_node : public hittable {
public:
  bvh_node(hittable_list list) : bvh_node(list.objects, 0, list.objects.size()) {
    {}
  }

  // �t�d��@�� hittable object �غc�� BVH�Abegin_i, end_i �O�e����}
  bvh_node(std::vector<std::shared_ptr<hittable>> &objects, size_t begin_i, size_t end_i) {
    //  0 �N��H x ���A1 �N��H y ���A2 �N��H z ��
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
    // �ƧǥH��A�q�������}�A�åB
    std::sort(objects.begin() + begin_i, objects.begin() + end_i, comp);

    int count = end_i - begin_i;
    if (count == 1) {
      // ���k���n���V�P�@�ӡA����O nullptr�A�]���o�̨S�� nullptr �B�z
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
      // �p�G�I��F�A�|��s record.t
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