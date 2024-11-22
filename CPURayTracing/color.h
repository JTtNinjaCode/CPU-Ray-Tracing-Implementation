#pragma once
#include <iostream>

#include "vec3.h"
using color = vec3;

// �������O���z���u
// ���z���u�U�A�G�������H���ӯ����A��y�L�t�@�I�o����ӷP
// �]���⪫�z�ȳ��� 1/ 2.2�A�p������t���C��� 0~1 ���񭫴N�|����h
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
