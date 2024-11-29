#pragma once
#include "utility.h"
#include "vec3.h"
class perlin {
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

  double noise(const point3 &p) const {
    // �����q
    int iu = int(std::floor(p.x()));
    int iv = int(std::floor(p.y()));
    int iw = int(std::floor(p.z()));

    // �����q���h�X�Ӫ����
    double du = (p.x() - iu);
    double dv = (p.y() - iv);
    double dw = (p.z() - iw);

    // �H�K���@��Ʀr xor�A���� float
    return perlin_interp(iu, iv, iw, du, dv, dw);
  }

private:
  static const int point_count = 256;
  vec3 rand_offset[point_count];
  int perm_x[point_count];
  int perm_y[point_count];
  int perm_z[point_count];

  // ���� 0 ~ point_count-1 �åB����
  static void perlin_generate_perm(int *p) {
    for (int i = 0; i < point_count; i++) p[i] = i;
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