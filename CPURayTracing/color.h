#pragma once
#include <iostream>

#include "vec3.h"
using color = vec3;

inline color red = color(1, 0, 0);
inline color green = color(0, 1, 0);
inline color blue = color(0, 0, 1);
inline color yellow = color(1, 1, 0);
inline color cyan = color(0, 1, 1);
inline color magenta = color(1, 0, 1);
inline color white = color(1, 1, 1);
inline color black = color(0, 0, 0);

inline double linear_to_gamma(double linear_component) {
  if (linear_component > 0)
    return std::pow(linear_component, 1 / 2.2);
  return 0;
}

void write_color(std::ostream &out, const color &pixel_color) {
  auto r = pixel_color.x();
  auto g = pixel_color.y();
  auto b = pixel_color.z();

  // Replace NaN components with zero.
  if (r != r)
    r = 0.0;
  if (g != g)
    g = 0.0;
  if (b != b)
    b = 0.0;

  r = linear_to_gamma(r);
  g = linear_to_gamma(g);
  b = linear_to_gamma(b);

  // Translate the [0,1] component values to the byte range [0,255].
  int rbyte = int(255.999 * r);
  int gbyte = int(255.999 * g);
  int bbyte = int(255.999 * b);
  out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
}
