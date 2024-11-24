#pragma once
#include "vec3.h"

class frame {
 public:
  vec3 transform(vec3 v) { return 0 + v.x() * x + v.y() * y + v.z() * z; }
  vec3 trans_to_conanical(vec3 v) {}
  vec3 trans_to_onb(vec3 v) {}
  vec3 x = vec3(0.0);
  vec3 y = vec3(0.0);
  vec3 z = vec3(0.0);
  vec3 o = vec3(0.0);
};
// 使用 normal 建構出一個 orthnormal basis，選擇垂直於 normal
// 的平面中的兩個垂直向量以及 normal 作為 basis，只保證 y direction 指向 normal
// x, z 的可能性有無限多種，但保證是平面中的兩垂直向量

class onb : public frame {
 public:
  onb(const vec3& normal) {
    y = unit_vector(normal);
    // a 正常來說要是 vec3(1, 0, 0)(自己選)，但如果 normal
    // 太靠近該方向，便無法展開出 basis，因此改為另一個方向，在這裡我使用
    // vec3(0, 0, 1)
    vec3 a = (std::fabs(y.x()) > 0.9) ? vec3(0, 0, 1) : vec3(1, 0, 0);
    z = unit_vector(cross(y, a));
    x = cross(y, z);
  }
};