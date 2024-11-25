#pragma once
#include <map>

#include "hittable.h"
#include "onb.h"
#include "pdf.h"
#include "ray.h"
#include "spectrum.h"
#include "texture.h"
#include "utility.h"
#include "vec3.h"

// material 決定如何濾光並且如何散射
// scatter
// 的定義:指的是光線在與物體或介質相互作用後，"改變其方向的現象"，折射、反射就是
// scatter 更進階的說，次表面反射，體散射也都是如此

// 體散射（Volumetric
// Scattering）：當光線穿過參與介質時，它會與介質中的微小粒子或分子相互作用，改變其方向，這叫做體散射。這包括瑞利散射（Rayleigh
// Scattering）和米氏散射（Mie
// Scattering），是用來模擬雲霧、煙霧、皮膚次表面散射等現象。
//
// 次表面散射（Subsurface
// Scattering）：這是一種特殊的散射，發生在光線穿透物體表面後在內部被多次散射，再次從表面逃逸的現象。這在模擬半透明材質（如皮膚、大理石、生魚片）時非常重要

// kRandom: return [attenuation, pdf], user can combine it with other pdf and sample it
// kDetermined: return [attenuation, ray_scattered], user can directly use it
enum class scatter_mode { kRandom, kDetermined };
// 下一個散射的方向
struct scatter_record {
  scatter_mode mode;
  vec3 attenuation;
  ray ray_scattered;
  std::shared_ptr<pdf> pdf;
};

class material {
public:
  virtual ~material() {}
  // [out] scattered, attenuation, pdf
  // ray 已經碰到 object，接著決定下一條 ray
  // 要往哪散射或被吸收（根據回傳值決定是否有效），決定下一條 ray 往哪裡射是由
  // material 決定的，pdf 表示建議使用的抽樣方式的機率密度函數
  virtual bool scatter(const ray &r_in, const hit_record &hit_record, scatter_record &scatter_record) const { return false; }

  // pScatter 的值，表示 input radiance 會被散射到特定方向的強度
  virtual double p_scattered(const ray &r_in, const hit_record &rec, const ray &scattered) const { return 0; }

  virtual color emitted(const ray &r_in, const hit_record &rec, double u, double v, const point3 &p) const { return color(0); }

  virtual bool spectrum_scatter(const ray &r_in, const hit_record &record, spectrum &attenuation, ray &scattered, double &pdf) const {
    return false;
  }

  virtual spectrum spectrum_emitted(double u, double v, const point3 &p) const { return spectrum(); }
};

class lambertian : public material {
public:
  lambertian(std::shared_ptr<texture> albedo) { tex_ = albedo; }
  lambertian(color albedo) { tex_ = std::make_shared<solid_color>(albedo); }

  virtual bool scatter(const ray &r_in, const hit_record &hit_record, scatter_record &scatter_record) const override {
    scatter_record.mode = scatter_mode::kRandom;
    scatter_record.attenuation = tex_->sample(hit_record.u, hit_record.v, hit_record.p);
    scatter_record.pdf = std::make_shared<hemisphere_cosine_pdf>(hit_record.normal);
    return true;
  }

  double p_scattered(const ray &r_in, const hit_record &rec, const ray &scattered) const override {
    auto cosine = dot(rec.normal, unit_vector(scattered.direction()));
    return cosine < 0 ? 0 : cosine / pi;
  }

private:
  std::shared_ptr<texture> tex_;
};

class metal : public material {
public:
  metal(std::shared_ptr<texture> albedo, float fuzz = 0.0) : tex_(albedo) { fuzz_ = interval(0, 1).clamp(fuzz); }
  metal(color albedo, float fuzz = 0.0) {
    fuzz_ = interval(0, 1).clamp(fuzz);
    tex_ = std::make_shared<solid_color>(albedo);
  }
  virtual bool scatter(const ray &r_in, const hit_record &hit_record, scatter_record &scatter_record) const override {
    if (fuzz_ != 0) {
      auto scatter_direction = unit_vector(reflect(r_in.direction(), hit_record.normal));
      scatter_direction += fuzz_ * random_unit_vec();
      scatter_record.mode = scatter_mode::kRandom;
      scatter_record.ray_scattered = ray(hit_record.p, scatter_direction, r_in.time());
      scatter_record.attenuation = tex_->sample(hit_record.u, hit_record.v, hit_record.p);
      // if fuzz's radius is too large, the vector will shoot into the object, so we need to make sure that vector is the same direction as
      // the normal
      return (dot(scatter_record.ray_scattered.direction(), hit_record.normal) > 0);
    } else {
      auto scatter_direction = unit_vector(reflect(r_in.direction(), hit_record.normal));
      scatter_record.mode = scatter_mode::kDetermined;
      scatter_record.ray_scattered = ray(hit_record.p, scatter_direction, r_in.time());
      scatter_record.attenuation = tex_->sample(hit_record.u, hit_record.v, hit_record.p);
      // if fuzz's radius is too large, the vector will shoot into the object, so we need to make sure that vector is the same direction as
      // the normal
      return (dot(scatter_record.ray_scattered.direction(), hit_record.normal) > 0);
    }
  }

private:
  std::shared_ptr<texture> tex_;
  float fuzz_;
};

// 以折射率決定怎麼散射，折射率要大於 1 才有機會會發生全反射
class dielectric : public material {
public:
  dielectric(std::shared_ptr<texture> albedo, float refract) {
    tex_ = albedo;
    refract_ = refract;
  }

  dielectric(float refract) {
    tex_ = std::make_shared<solid_color>(color(1));
    refract_ = refract;
  }

  // [out] scattered, attenuation
  virtual bool scatter(const ray &r_in, const hit_record &hit_record, scatter_record &scatter_record) const override {
    scatter_record.mode = scatter_mode::kDetermined;
    scatter_record.attenuation = tex_->sample(hit_record.u, hit_record.v, hit_record.p);
    // 折射率定義: 外/內，如果發現是從內部方向來的光則要倒數
    double ri = hit_record.front_face ? (1.0 / refract_) : refract_;
    vec3 unit_direction = unit_vector(r_in.direction());
    double cos_theta = std::fmin(dot(-unit_direction, hit_record.normal), 1.0);
    double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
    // 判別式 ri * sin_theta，超過 1 代表全反射
    bool cant_refract = ri * sin_theta > 1.0;
    if (cant_refract || reflectance(cos_theta, ri) > random_double()) { // Must Reflect
      auto reflect_dir = reflect(unit_direction, hit_record.normal);
      scatter_record.ray_scattered = ray(hit_record.p, reflect_dir, r_in.time());
    } else { // Can Refract
      auto refract_dir = refract(unit_direction, hit_record.normal, ri);
      scatter_record.ray_scattered = ray(hit_record.p, refract_dir, r_in.time());
    }
    return true;
  }

private:
  // Use Schlick's approximation for reflectance. just a magic formula.
  static double reflectance(double cosine, double refraction_index) {
    auto r0 = (1 - refraction_index) / (1 + refraction_index);
    r0 = r0 * r0;
    return r0 + (1 - r0) * std::pow((1 - cosine), 5);
  }

  std::shared_ptr<texture> tex_;
  float refract_;
};

class isotropic : public material {
public:
  isotropic(const color &albedo) { tex_ = std::make_shared<solid_color>(albedo); }

  isotropic(std::shared_ptr<texture> tex) : tex_(tex) {}

  bool scatter(const ray &r_in, const hit_record &hit_record, scatter_record &scatter_record) const override {
    scatter_record.mode = scatter_mode::kRandom;
    scatter_record.attenuation = tex_->sample(hit_record.u, hit_record.v, hit_record.p);
    scatter_record.pdf = std::make_shared<spherical_pdf>();
    return true;
  }

  double p_scattered(const ray &r_in, const hit_record &rec, const ray &scattered) const override { return 1 / (4 * pi); }

private:
  std::shared_ptr<texture> tex_;
};

class diffuse_light : public material {
public:
  diffuse_light(std::shared_ptr<texture> tex_) : tex_(tex_) {}
  diffuse_light(color albedo) { tex_ = std::make_shared<solid_color>(albedo); }

  color emitted(const ray &r_in, const hit_record &record, double u, double v, const point3 &p) const override {
    if (!record.front_face) return color(0, 0, 0);
    return tex_->sample(u, v, p);
  }

private:
  std::shared_ptr<texture> tex_;
};

//// spectrum 版本
// class spectrum_lambertian : public material {
// public:
//   spectrum_lambertian(const spectrum &albedo) : albedo_(albedo) {}
//
//   // [out] scattered, attenuation
//   bool spectrum_scatter(const ray &r_in, const hit_record &record, spectrum &attenuation, ray &scattered, double &pdf) const override {
//     auto scatter_direction = record.normal + random_unit_vec();
//     if (scatter_direction.near_zero()) scatter_direction = record.normal;
//     scattered = ray(record.p, scatter_direction, r_in.time());
//     attenuation = albedo_; // Use spectrum instead of color
//     return true;
//   }
//
//   double p_scattered(const ray &r_in, const hit_record &rec, const ray &scattered) const override {
//     auto cos_theta = dot(rec.normal, unit_vector(scattered.direction()));
//     return cos_theta < 0 ? 0 : cos_theta / pi;
//   }
//
// private:
//   spectrum albedo_;
// };

// class spectrum_diffuse_light : public material {
// public:
//   spectrum_diffuse_light(const spectrum &albedo) : albedo_(albedo) {}
//
//   spectrum spectrum_emitted(double u, double v, const point3 &p) const override { return albedo_; }
//
// private:
//   spectrum albedo_;
// };
//
//// 根據不同的波長使用不同的折射率，使用柯西色散公式
// class spectrum_dielectric : public material {
// public:
//   spectrum_dielectric(double a, double b) {
//     a_ = a;
//     b_ = b;
//   }
//
//   // [out] scattered, attenuation
//   bool spectrum_scatter(const ray &r_in, const hit_record &record, spectrum &attenuation, ray &scattered, double &pdf) const override {
//     attenuation = spectrum();
//     // 折射率定義: 外/內，如果發現是從內部方向來的光則要倒數
//     for (int i = 0; i < spectrum::size(); i++) {
//       double lambda = spectrum::wavelength(i);
//       double r = cauchy(lambda);
//       double ri = record.front_face ? (1.0 / r) : r;
//       vec3 r_dir = unit_vector(r_in.direction());
//       double cos_theta = std::fmin(dot(-r_dir, record.normal), 1.0);
//       double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
//       // 判別式 ri * sin_theta，超過 1 代表全反射
//       bool cant_refract = ri * sin_theta > 1.0;
//       if (cant_refract || reflectance(cos_theta, ri) > random_double()) {
//         // Must Reflect
//         auto reflect_dir = reflect(r_dir, record.normal);
//         scattered = ray(record.p, reflect_dir, r_in.time());
//       } else {
//         // Can Refract
//         auto refract_dir = refract(r_dir, record.normal, ri);
//         scattered = ray(record.p, refract_dir, r_in.time());
//       }
//     }
//     return true;
//   }
//
// private:
//   // Use Schlick's approximation for reflectance. just a magic.
//   static double reflectance(double cosine, double refraction_index) {
//     auto r0 = (1 - refraction_index) / (1 + refraction_index);
//     r0 = r0 * r0;
//     return r0 + (1 - r0) * std::pow((1 - cosine), 5);
//   }
//
//   double cauchy(double lambda) const { return a_ + b_ / (pow(lambda, 2)); }
//
//   double a_, b_; // 柯西色散公式的參數
// };
