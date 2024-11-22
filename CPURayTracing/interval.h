#pragma once
#include "utility.h"

class interval {
 public:
  double min;
  double max;
  // default interval is empty
  interval() : min(infinity), max(-infinity) {}
  interval(double min, double max) : min(min), max(max) {}

  double size() const { return max - min; }
  // 閉區間
  bool is_contains(double x) const { return min <= x && x <= max; }
  // 開區間
  bool is_surrounds(double x) const { return min < x && x < max; }

  double clamp(double x) const {
    if (x < min) return min;
    if (x > max) return max;
    return x;
  }

  // 產生當前 interval 的擴展，前後各擴展 delta 的一半
  interval expand(double delta) const {
    auto padding = delta / 2;
    return interval(min - padding, max + padding);
  }

  interval offset(double offset) const {
    return interval(min + offset, max + offset);
  }

  static const interval empty, universe;
  static interval enclose(interval i1, interval i2) {
    double new_min = i1.min < i2.min ? i1.min : i2.min;
    double new_max = i1.max > i2.max ? i1.max : i2.max;
    return interval(new_min, new_max);
  }
};

const interval interval::empty = interval(+infinity, -infinity);
const interval interval::universe = interval(-infinity, +infinity);