#pragma once
#include "onb.h"
#include "utility.h"
#include <initializer_list>
#include <memory>
#include <vector>
class pdf {
public:
  virtual ~pdf() {}
  virtual double value(const vec3 &direction) const = 0;
  virtual vec3 generate() const = 0;
};

// uniformly sample a direction in the sphere
class spherical_pdf : public pdf {
public:
  spherical_pdf() = default;
  double value(const vec3 &direction) const override { return 1 / (4 * pi); }
  vec3 generate() const override { return random_unit_vec(); }
};

// uniformly sample a direction in the hemisphere with normal
class hemisphere_pdf : public pdf {
public:
  hemisphere_pdf(const vec3 &normal) : uvw_(normal) {}
  double value(const vec3 &direction) const override { return dot(uvw_.y, direction) > 0 ? dot(uvw_.y, direction) / pi : 0; }
  vec3 generate() const override { return uvw_.transform(random_on_hemisphere(uvw_.y)); }

private:
  onb uvw_;
};

// uniformly sample a direction in the cosine hemisphere with normal
class hemisphere_cosine_pdf : public pdf {
public:
  hemisphere_cosine_pdf(const vec3 &normal) : uvw_(normal) {}
  double value(const vec3 &direction) const override {
    auto cosine_theta = dot(unit_vector(direction), uvw_.y);
    return std::fmax(0, cosine_theta / pi);
  }
  vec3 generate() const override { return uvw_.transform(random_cosine_direction()); }

private:
  onb uvw_;
};

// uniformly combine multiple pdfs
class dual_pdf : public pdf {
public:
  dual_pdf(std::shared_ptr<pdf> pdf1, std::shared_ptr<pdf> pdf2) : pdfs_{pdf1, pdf2} {}
  double value(const vec3 &dir) { return 0.5 * pdfs_[0]->value(dir) + 0.5 * pdfs_[1]->value(dir); }
  vec3 generate() {
    if (random_double() < 0.5)
      return pdfs_[0]->generate();
    else
      return pdfs_[1]->generate();
  }

private:
  std::shared_ptr<pdf> pdfs_[2];
};