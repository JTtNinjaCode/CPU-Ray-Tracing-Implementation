#pragma once
#include "color.h"
#include "image.h"
#include "perlin.h"
#include "ray.h"
class texture {
 public:
  virtual ~texture(){};
  virtual color sample(double u, double v, point3 p) = 0;
};

class solid_color : public texture {
 public:
  solid_color(color color) { color_ = color; }
  color sample(double u, double v, point3 p) override { return color_; }

  static std::shared_ptr<solid_color> black;
  static std::shared_ptr<solid_color> white;

 private:
  color color_;
};

std::shared_ptr<solid_color> solid_color::black = std::make_shared<solid_color>(color(0, 0, 0));
std::shared_ptr<solid_color> solid_color::white = std::make_shared<solid_color>(color(1, 1, 1));

class checker_texture : public texture {
 public:
  // scale = 格子的長寬
  checker_texture(color odd, color even, double scale) {
    odd_ = odd;
    even_ = even;
    scale_ = scale;
  }
  color sample(double u, double v, point3 p) override {
    point3 uv = p / scale_;
    int ix = std::floor(uv.x());
    int iy = std::floor(uv.y());
    int iz = std::floor(uv.z());

    int total = ix + iy + iz;

    return (total % 2 == 0) ? even_ : odd_;
  }

 private:
  color odd_;
  color even_;
  double scale_;
  bool pos_or_uv_;
};

class picture_texture : public texture {
 public:
  picture_texture(std::shared_ptr<image> image) { image_ = image; }
  color sample(double u, double v, point3 p) override {
    int i = image_->width() * u;
    int j = image_->height() * (1 - v);
    auto data_ptr = image_->pixel_data(i, j);
    double color_scale = 1 / 256.0;
    return color(data_ptr[0] * color_scale, data_ptr[1] * color_scale,
                 data_ptr[2] * color_scale);
  }

 private:
  std::shared_ptr<image> image_;
};

class perlin_texture : public texture {
 public:
  // 固定顏色為黑白
  perlin_texture(double scale) { scale_ = scale; }
  color sample(double u, double v, point3 p) override {
    return color(.5, .5, .5) *
           (1 + std::sin(scale_ * p.z() + 10 * noise_.turb(7, p)));
  }

 private:
  perlin noise_;
  double scale_;
};