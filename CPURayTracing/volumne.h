#pragma once
#include "aabb.h"
#include "hittable.h"
#include "utility"

// constant 相當於一個包裝物件，他讓裡面的 hittable
// 賦予一個密度，並且有機會穿過該物件，因為是
// volumne，因此所包住的物件必須是有內外之分的
class volumne : public hittable {
 public:
  volumne(std::shared_ptr<hittable> boundary, double density,
          std::shared_ptr<texture> tex) {
    boundary_ = boundary;
    density_ = density;
    phase_function_ = std::make_shared<isotropic>(tex);
  }

  // 碰到 volumne 以後，碰到的點會在 volumne 內
  bool hit(const ray& r, interval ray_t, hit_record& record) const {
    hit_record record1, record2;
    // record 分別代表碰到 volumne 的兩個點
    if (!boundary_->hit(r, interval::universe, record1)) return false;
    if (!boundary_->hit(r, interval(record1.t + 0.0001, infinity), record2))
      return false;

    // 把兩個 record 都 clamp 到 ray_t 範圍內
    if (record1.t < ray_t.min) record1.t = ray_t.min;
    if (record2.t > ray_t.max) record2.t = ray_t.max;

    if (record1.t >= record2.t) return false;
    if (record1.t < 0) record1.t = 0;

    // ray 在 box 裡面有多長
    auto ray_length = r.direction().length();
    auto distance_inside_boundary = (record2.t - record1.t) * ray_length;

    // ray 在 box 內
    auto hit_distance = -1.0 / density_ * std::log(random_double());

    // false 代表光直接穿過整個 hittable 物件  啥都沒打到
    if (hit_distance > distance_inside_boundary) return false;
    record.t = record1.t + hit_distance / ray_length;
    record.p = r.at(record.t);
    record.normal = vec3(1, 0, 0);  // arbitrary
    record.front_face = true;       // arbitrary
    record.mat = phase_function_;
    return true;
  }

  aabb get_bounding_box() const { return boundary_->get_bounding_box(); }

 private:
  std::shared_ptr<hittable> boundary_;
  double density_;
  std::shared_ptr<material> phase_function_;

  // phase
  // function（相位函數）是一個描述光線在參與介質（如霧、煙、液體、皮膚等）中發生散射後，沿著不同方向的散射強度的數學函數。它用來計算光線從一個方向進入介質後，以不同角度離開介質的概率和強度分佈。
  // 當光線穿過參與介質時，它會與介質中的微粒相互作用，這種相互作用會導致光線改變其方向（散射）。相位函數用來決定光線散射後的新方向與原方向之間的角度關係，以及每個方向上散射光的強度。
  //
};