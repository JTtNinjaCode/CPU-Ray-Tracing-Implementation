#pragma once
#include <map>

#include "hittable.h"
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

class material {
 public:
  virtual ~material() {}
  // [out] scattered, attenuation, pdf
  // scatter 代表光發生散射、折射出去，但有也可能被吸收，return bool
  // 就代表是否該條 ray 有成功 scatter 出去 or 被吸收，抽樣出的 ray 會對應一個
  // pdf value
  virtual bool scatter(const ray& r_in, const hit_record& record,
                       color& attenuation, ray& scattered, double& pdf) const {
    return false;
  }

  virtual double scattering_pdf(const ray& r_in, const hit_record& rec,
                                const ray& scattered) const {
    return 0;
  }

  virtual color emitted(double u, double v, const point3& p) const {
    return color(0, 0, 0);
  }

  virtual bool spectrum_scatter(const ray& r_in, const hit_record& record,
                                spectrum& attenuation, ray& scattered,
                                double& pdf) const {
    return false;
  }

  virtual spectrum spectrum_emitted(double u, double v, const point3& p) const {
    return spectrum();
  }
};

class lambertian : public material {
 public:
  lambertian(std::shared_ptr<texture> albedo) { tex_ = albedo; }
  lambertian(color albedo) { tex_ = std::make_shared<solid_color>(albedo); }

  // [out] scattered, attenuation
  virtual bool scatter(const ray& r_in, const hit_record& record,
                       color& attenuation, ray& scattered,
                       double& pdf) const override {
    auto scatter_direction = record.normal + random_uint_vec();
    if (scatter_direction.near_zero()) scatter_direction = record.normal;
    scattered = ray(record.p, scatter_direction, r_in.time());
    attenuation = tex_->sample(record.u, record.v, record.p);
    return true;
  }

  double scattering_pdf(const ray& r_in, const hit_record& rec,
                        const ray& scattered) const override {
    auto cos_theta = dot(rec.normal, unit_vector(scattered.direction()));
    return cos_theta < 0 ? 0 : cos_theta / pi;
  }

 private:
  std::shared_ptr<texture> tex_;
};

// 反射
class metal : public material {
 public:
  metal(std::shared_ptr<texture> albedo, float fuzz) : tex_(albedo) {
    fuzz_ = interval(0, 1).clamp(fuzz);
  }
  // [out] scattered, attenuation
  virtual bool scatter(const ray& r_in, const hit_record& record,
                       color& attenuation, ray& scattered,
                       double& pdf) const override {
    auto scatter_direction =
        unit_vector(reflect(r_in.direction(), record.normal));
    scatter_direction += fuzz_ * random_uint_vec();
    scattered = ray(record.p, scatter_direction, r_in.time());
    attenuation = tex_->sample(record.u, record.v, record.p);
    // 如果 fuzz 半徑太大，會導致向量往內部射，此時要回傳 false 表示沒有 scatter
    return (dot(scattered.direction(), record.normal) > 0);
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
  // [out] scattered, attenuation
  virtual bool scatter(const ray& r_in, const hit_record& record,
                       color& attenuation, ray& scattered,
                       double& pdf) const override {
    attenuation = tex_->sample(record.u, record.v, record.p);
    // 折射率定義: 外/內，如果發現是從內部方向來的光則要倒數
    double ri = record.front_face ? (1.0 / refract_) : refract_;
    vec3 unit_direction = unit_vector(r_in.direction());
    double cos_theta = std::fmin(dot(-unit_direction, record.normal), 1.0);
    double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
    // 判別式 ri * sin_theta，超過 1 代表全反射
    bool cant_refract = ri * sin_theta > 1.0;
    if (cant_refract || reflectance(cos_theta, ri) > random_double()) {
      // Must Reflect
      auto reflect_dir = reflect(unit_direction, record.normal);
      scattered = ray(record.p, reflect_dir, r_in.time());
    } else {
      // Can Refract
      auto refract_dir = refract(unit_direction, record.normal, ri);
      scattered = ray(record.p, refract_dir, r_in.time());
    }
    return true;
  }

 private:
  // Use Schlick's approximation for reflectance. just a magic.
  static double reflectance(double cosine, double refraction_index) {
    auto r0 = (1 - refraction_index) / (1 + refraction_index);
    r0 = r0 * r0;
    return r0 + (1 - r0) * std::pow((1 - cosine), 5);
  }

  std::shared_ptr<texture> tex_;
  float refract_;
};

// 往360 度各種方向散射
class isotropic : public material {
 public:
  isotropic(const color& albedo) {
    tex_ = std::make_shared<solid_color>(albedo);
  }

  isotropic(std::shared_ptr<texture> tex) : tex_(tex) {}

  bool scatter(const ray& r_in, const hit_record& rec, color& attenuation,
               ray& scattered, double& pdf) const override {
    scattered = ray(rec.p, random_uint_vec(), r_in.time());
    attenuation = tex_->sample(rec.u, rec.v, rec.p);
    pdf = 1 / (4 * pi);
    return true;
  }

  double scattering_pdf(const ray& r_in, const hit_record& rec,
                        const ray& scattered) const override {
    return 1 / (4 * pi);
  }

 private:
  std::shared_ptr<texture> tex_;
};

class diffuse_light : public material {
 public:
  diffuse_light(std::shared_ptr<texture> tex_) : tex_(tex_) {}
  diffuse_light(color albedo) { tex_ = std::make_shared<solid_color>(albedo); }

  color emitted(double u, double v, const point3& p) const override {
    return tex_->sample(u, v, p);
  }

 private:
  std::shared_ptr<texture> tex_;
};

// spectrum 版本
class spectrum_lambertian : public material {
 public:
  spectrum_lambertian(const spectrum& albedo) : albedo_(albedo) {}

  // [out] scattered, attenuation
  bool spectrum_scatter(const ray& r_in, const hit_record& record,
                        spectrum& attenuation, ray& scattered,
                        double& pdf) const override {
    auto scatter_direction = record.normal + random_uint_vec();
    if (scatter_direction.near_zero()) scatter_direction = record.normal;
    scattered = ray(record.p, scatter_direction, r_in.time());
    attenuation = albedo_;  // Use spectrum instead of color
    return true;
  }

  double scattering_pdf(const ray& r_in, const hit_record& rec,
                        const ray& scattered) const override {
    auto cos_theta = dot(rec.normal, unit_vector(scattered.direction()));
    return cos_theta < 0 ? 0 : cos_theta / pi;
  }

 private:
  spectrum albedo_;
};

class spectrum_diffuse_light : public material {
 public:
  spectrum_diffuse_light(const spectrum& albedo) : albedo_(albedo) {}

  spectrum spectrum_emitted(double u, double v,
                            const point3& p) const override {
    return albedo_;
  }

 private:
  spectrum albedo_;
};

// 根據不同的波長使用不同的折射率，使用柯西色散公式
class spectrum_dielectric : public material {
 public:
  spectrum_dielectric(double a, double b) {
    a_ = a;
    b_ = b;
  }

  // [out] scattered, attenuation
  bool spectrum_scatter(const ray& r_in, const hit_record& record,
                        spectrum& attenuation, ray& scattered,
                        double& pdf) const override {
    attenuation = spectrum();
    // 折射率定義: 外/內，如果發現是從內部方向來的光則要倒數
    for (int i = 0; i < spectrum::size(); i++) {
      double lambda = spectrum::wavelength(i);
      double r = cauchy(lambda);
      double ri = record.front_face ? (1.0 / r) : r;
      vec3 r_dir = unit_vector(r_in.direction());
      double cos_theta = std::fmin(dot(-r_dir, record.normal), 1.0);
      double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
      // 判別式 ri * sin_theta，超過 1 代表全反射
      bool cant_refract = ri * sin_theta > 1.0;
      if (cant_refract || reflectance(cos_theta, ri) > random_double()) {
        // Must Reflect
        auto reflect_dir = reflect(r_dir, record.normal);
        scattered = ray(record.p, reflect_dir, r_in.time());
      } else {
        // Can Refract
        auto refract_dir = refract(r_dir, record.normal, ri);
        scattered = ray(record.p, refract_dir, r_in.time());
      }
    }
    return true;
  }

 private:
  // Use Schlick's approximation for reflectance. just a magic.
  static double reflectance(double cosine, double refraction_index) {
    auto r0 = (1 - refraction_index) / (1 + refraction_index);
    r0 = r0 * r0;
    return r0 + (1 - r0) * std::pow((1 - cosine), 5);
  }

  double cauchy(double lambda) const { return a_ + b_ / (pow(lambda, 2)); }

  double a_, b_;  // 柯西色散公式的參數
};
