#pragma once
#include <map>

#include "hittable.h"
#include "ray.h"
#include "spectrum.h"
#include "texture.h"
#include "utility.h"
#include "vec3.h"

// material �M�w�p���o���åB�p�󴲮g
// scatter
// ���w�q:�����O���u�b�P����Τ���ۤ��@�Ϋ�A"���ܨ��V���{�H"�A��g�B�Ϯg�N�O
// scatter ��i�������A�����Ϯg�A�鴲�g�]���O�p��

// �鴲�g�]Volumetric
// Scattering�^�G����u��L�ѻP����ɡA���|�P���褤���L�p�ɤl�Τ��l�ۤ��@�ΡA���ܨ��V�A�o�s���鴲�g�C�o�]�A��Q���g�]Rayleigh
// Scattering�^�M�̤󴲮g�]Mie
// Scattering�^�A�O�ΨӼ��������B�����B�ֽ��������g���{�H�C
//
// �������g�]Subsurface
// Scattering�^�G�o�O�@�دS�����g�A�o�ͦb���u��z�������b�����Q�h�����g�A�A���q���k�h���{�H�C�o�b�����b�z������]�p�ֽ��B�j�z�ۡB�ͳ����^�ɫD�`���n

class material {
 public:
  virtual ~material() {}
  // [out] scattered, attenuation, pdf
  // scatter �N����o�ʹ��g�B��g�X�h�A�����]�i��Q�l���Areturn bool
  // �N�N��O�_�ӱ� ray �����\ scatter �X�h or �Q�l���A��˥X�� ray �|�����@��
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

// �Ϯg
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
    // �p�G fuzz �b�|�Ӥj�A�|�ɭP�V�q�������g�A���ɭn�^�� false ��ܨS�� scatter
    return (dot(scattered.direction(), record.normal) > 0);
  }

 private:
  std::shared_ptr<texture> tex_;
  float fuzz_;
};

// �H��g�v�M�w��򴲮g�A��g�v�n�j�� 1 �~�����|�|�o�ͥ��Ϯg
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
    // ��g�v�w�q: �~/���A�p�G�o�{�O�q������V�Ӫ����h�n�˼�
    double ri = record.front_face ? (1.0 / refract_) : refract_;
    vec3 unit_direction = unit_vector(r_in.direction());
    double cos_theta = std::fmin(dot(-unit_direction, record.normal), 1.0);
    double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
    // �P�O�� ri * sin_theta�A�W�L 1 �N����Ϯg
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

// ��360 �צU�ؤ�V���g
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

// spectrum ����
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

// �ھڤ��P���i���ϥΤ��P����g�v�A�ϥά_��ⴲ����
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
    // ��g�v�w�q: �~/���A�p�G�o�{�O�q������V�Ӫ����h�n�˼�
    for (int i = 0; i < spectrum::size(); i++) {
      double lambda = spectrum::wavelength(i);
      double r = cauchy(lambda);
      double ri = record.front_face ? (1.0 / r) : r;
      vec3 r_dir = unit_vector(r_in.direction());
      double cos_theta = std::fmin(dot(-r_dir, record.normal), 1.0);
      double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
      // �P�O�� ri * sin_theta�A�W�L 1 �N����Ϯg
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

  double a_, b_;  // �_��ⴲ�������Ѽ�
};
