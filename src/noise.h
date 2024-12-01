#pragma once
#include "utility.h"
#include "vec3.h"

class noise_base {
public:
  virtual double noise(const point3 &p) const = 0;
};

class perlin : public noise_base {
public:
  perlin() {
    for (int i = 0; i < point_count; i++) {
      rand_offset[i] = unit_vector(random_vec(-1, 1));
    }

    perlin_generate_perm(perm_x);
    perlin_generate_perm(perm_y);
    perlin_generate_perm(perm_z);
  }

  double perlin_interp(int iu, int iv, int iw, double u, double v, double w) const {
    iu = iu & (point_count - 1); // mod it into range:[0, 255]
    iv = iv & (point_count - 1); // mod it into range:[0, 255]
    iw = iw & (point_count - 1); // mod it into range:[0, 255]

    double uu = u * u * (3 - 2 * u);
    double vv = v * v * (3 - 2 * v);
    double ww = w * w * (3 - 2 * w);

    auto accum = 0.0;
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < 2; j++)
        for (int k = 0; k < 2; k++) {
          vec3 weight_v(u - i, v - j, w - k);
          vec3 v = rand_offset[perm_x[(iu + i) % point_count] ^ perm_x[(iv + j) % point_count] ^ perm_x[(iw + k) % point_count]];
          accum += (i * uu + (1 - i) * (1 - uu)) * (j * vv + (1 - j) * (1 - vv)) * (k * ww + (1 - k) * (1 - ww)) * dot(v, weight_v);
        }

    return accum;
  }

  double turb(int depth, const point3 &p) const {
    double accum = 0;
    point3 temp_p = p;
    double weight = 1.0;
    for (int i = 0; i < depth; i++) {
      accum += weight * noise(temp_p);
      weight *= 0.5; // half the weight
      temp_p *= 2.0; // double the frequency
    }
    return std::fabs(accum);
  }

  double noise(const point3 &p) const override {
    // 取分段
    int iu = int(std::floor(p.x()));
    int iv = int(std::floor(p.y()));
    int iw = int(std::floor(p.z()));

    // 取分段中多出來的比例
    double du = (p.x() - iu);
    double dv = (p.y() - iv);
    double dw = (p.z() - iw);

    // 隨便拿一堆數字 xor，取其 float
    return perlin_interp(iu, iv, iw, du, dv, dw);
  }

private:
  static const int point_count = 256;
  vec3 rand_offset[point_count];
  int perm_x[point_count];
  int perm_y[point_count];
  int perm_z[point_count];

  // 產生 0 ~ point_count-1 並且打亂
  static void perlin_generate_perm(int *p) {
    for (int i = 0; i < point_count; i++)
      p[i] = i;
    permute(p, point_count);
  }

  // Fisher-Yates Shuffle
  static void permute(int *p, int n) {
    for (int i = n - 1; i > 0; i--) {
      int target = random_int(0, i);
      int tmp = p[i];
      p[i] = p[target];
      p[target] = tmp;
    }
  }
};

class value_noise : public noise_base {
public:
  value_noise(int resolution) : rand_offset(resolution * resolution * resolution), resolution_(resolution) {
    for (int i = 0; i < resolution; i++) {
      for (int j = 0; j < resolution; j++) {
        for (int k = 0; k < resolution; k++) {
          rand_offset[i * resolution * resolution + j * resolution + k] = random_double();
        }
      }
    }
  }

  double noise(const point3 &p) const override {
    vec3 floor_p = floor(p);
    float x0_y0_z0 = rand_offset[floor_p.x() * resolution_ * resolution_ + floor_p.y() * resolution_ + floor_p.z()];
    float x1_y0_z0 = rand_offset[(floor_p.x() + 1) * resolution_ * resolution_ + floor_p.y() * resolution_ + floor_p.z()];
    float x0_y1_z0 = rand_offset[floor_p.x() * resolution_ * resolution_ + (floor_p.y() + 1) * resolution_ + floor_p.z()];
    float x1_y1_z0 = rand_offset[(floor_p.x() + 1) * resolution_ * resolution_ + (floor_p.y() + 1) * resolution_ + floor_p.z()];
    float x0_y0_z1 = rand_offset[floor_p.x() * resolution_ * resolution_ + floor_p.y() * resolution_ + floor_p.z() + 1];
    float x1_y0_z1 = rand_offset[(floor_p.x() + 1) * resolution_ * resolution_ + floor_p.y() * resolution_ + floor_p.z() + 1];
    float x0_y1_z1 = rand_offset[floor_p.x() * resolution_ * resolution_ + (floor_p.y() + 1) * resolution_ + floor_p.z() + 1];
    float x1_y1_z1 = rand_offset[(floor_p.x() + 1) * resolution_ * resolution_ + (floor_p.y() + 1) * resolution_ + floor_p.z() + 1];

    double x = p.x() - floor_p.x();
    double y = p.y() - floor_p.y();
    double z = p.z() - floor_p.z();

    double y0_z0 = linear_interpolation(x, x0_y0_z0, x1_y0_z0);
    double y1_z0 = linear_interpolation(x, x0_y1_z0, x1_y1_z0);
    double y0_z1 = linear_interpolation(x, x0_y0_z1, x1_y0_z1);
    double y1_z1 = linear_interpolation(x, x0_y1_z1, x1_y1_z1);

    double z0 = linear_interpolation(y, y0_z0, y1_z0);
    double z1 = linear_interpolation(y, y0_z1, y1_z1);

    double result = linear_interpolation(z, z0, z1);
    return result;
  }

private:
  int resolution_;
  std::vector<float> rand_offset;
};

class worley_noise : public noise_base { // also called cell noise, voronoi noise
public:
  vec3 get_rand_offset(vec3 u) const {
    vec3 rand_v = vec3(dot(u, vec3(127.1, 311.7, 74.7)), dot(u, vec3(269.5, 183.3, 246.1)), dot(u, vec3(113.5, 271.9, 307.7)));
    vec3 result = vec3(sin(rand_v.x()), sin(rand_v.y()), sin(rand_v.z())) * 43758.5453;
    return fract(result);
  }

  double noise(const point3 &p) const override {
    vec3 floor_p = floor(p);
    float min_dist = std::numeric_limits<float>::max();
    float color = 0.0;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          vec3 neighbor_cell = floor_p + vec3(i, j, k);

          vec3 rand_offset = get_rand_offset(neighbor_cell);
          vec3 pos = neighbor_cell + rand_offset;

          float dist = (pos - p).length();
          if (dist < min_dist)
            min_dist = dist;
        }
      }
    }

    return min_dist * min_dist;
  }
};

class voronoi_noise : public noise_base {
public:
  vec3 get_rand_offset(vec3 u) const {
    vec3 rand_v = vec3(dot(u, vec3(127.1, 311.7, 74.7)), dot(u, vec3(269.5, 183.3, 246.1)), dot(u, vec3(113.5, 271.9, 307.7)));
    vec3 result = vec3(sin(rand_v.x()), sin(rand_v.y()), sin(rand_v.z())) * 43758.5453;
    return fract(result);
  }

  double noise(const point3 &p) const override {
    vec3 floor_p = floor(p);
    float min_dist = std::numeric_limits<float>::max();
    float color = 0.0;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          vec3 neighbor_cell = floor_p + vec3(i, j, k);

          vec3 rand_offset = get_rand_offset(neighbor_cell);
          vec3 pos = neighbor_cell + rand_offset;

          float dist = (pos - p).length();
          if (dist < min_dist) {
            min_dist = dist;
            color = get_rand_offset(pos).x();
          }
        }
      }
    }

    return color;
  }
};