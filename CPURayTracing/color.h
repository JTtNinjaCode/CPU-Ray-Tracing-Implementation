#pragma once
#include <iostream>

#include "vec3.h"
using color = vec3;

// 模擬的是物理光線
// 物理光線下，亮的部分人不太能分辨，對稍微暗一點得比較敏感
// 因此把物理值都做 1/ 2.2，如此比較暗的顏色佔 0~1 的比重就會比較多
inline double linear_to_gamma(double linear_component) {
  if (linear_component > 0) return std::pow(linear_component, 1 / 2.2);
  return 0;
}

void write_color(std::ostream& out, const color& pixel_color) {
  auto r = pixel_color.x();
  auto g = pixel_color.y();
  auto b = pixel_color.z();

  r = linear_to_gamma(r);
  g = linear_to_gamma(g);
  b = linear_to_gamma(b);

  // Translate the [0,1] component values to the byte range [0,255].
  int rbyte = int(255.999 * r);
  int gbyte = int(255.999 * g);
  int bbyte = int(255.999 * b);
  // Write out the pixel color components.
  out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
}
