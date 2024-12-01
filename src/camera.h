#pragma once
// The camera class will be responsible for two important jobs:
// 1. Construct and dispatch rays into the world.
// 2. Use the results of these rays to construct the rendered image.

#include <algorithm>
#include <execution>
#include <fstream>
#include <mutex>

#include "hittable.h"
#include "interval.h"
#include "material.h"
#include "pdf.h"
#include "sphere.h"
#include "utility.h"

enum camera_mode { kPerspective, kOrthnormal, kFisheye, kLens };
class camera {
public:
  void initialize_perspective(int image_width, double aspect_ratio, point3 pos, vec3 lookat, float focal_length = 1, float fovy_degree = 90,
                              int sample_per_pixel = 100, int max_recur_depth = 5) {
    mode_ = kPerspective;
    pos_ = pos;
    world_up_ = vec3(0, 1, 0);
    dir_ = unit_vector(lookat - pos);
    right_ = unit_vector(cross(dir_, world_up_));
    up_ = cross(right_, dir_);

    fovy_degree_ = fovy_degree;
    focal_length_ = focal_length;

    aspect_ratio_ = aspect_ratio;
    image_width_ = image_width;
    image_height_ = int(image_width / aspect_ratio);
    image_height_ = (image_height_ < 1) ? 1 : image_height_;

    samples_per_pixel_ = sample_per_pixel;
    max_recur_depth_ = max_recur_depth;

    // Why use (double(width) / height) instead of aspect_ratio?
    // aspect_ratio is the ideal value, but since width and height
    // are integers, the actual aspect ratio will be slightly off,
    // possibly something like 15.99xx / 9.0 or 16.00xx / 9.0.
    float theta = degrees_to_radians(fovy_degree_);
    viewport_height_ = 2.0 * std::tan(theta / 2.0) * focal_length_;
    viewport_width_ = viewport_height_ * (double(image_width_) / image_height_);

    image_.resize(image_width_ * image_height_);
  }

  void initialize_orthnormal(int image_width, double aspect_ratio, double viewport_height, point3 pos, vec3 lookat,
                             int sample_per_pixel = 100, int max_recur_depth = 5) {
    mode_ = kOrthnormal;
    pos_ = pos;
    world_up_ = vec3(0, 1, 0);
    dir_ = unit_vector(lookat - pos);
    right_ = unit_vector(cross(dir_, world_up_));
    up_ = cross(right_, dir_);

    aspect_ratio_ = aspect_ratio;
    image_width_ = image_width;
    image_height_ = int(image_width / aspect_ratio);
    image_height_ = (image_height_ < 1) ? 1 : image_height_;
    image_.resize(image_width_ * image_height_);

    samples_per_pixel_ = sample_per_pixel;
    max_recur_depth_ = max_recur_depth;

    viewport_height_ = viewport_height;
    viewport_width_ = viewport_height * (double(image_width_) / image_height_);
  }

  void initialize_fisheye(int image_width, double aspect_ratio, point3 pos, vec3 lookat, float focal_length = 1, float fovy_degree = 90,
                          int sample_per_pixel = 100, int max_recur_depth = 5) {
    mode_ = kFisheye;
    pos_ = pos;
    world_up_ = vec3(0, 1, 0);
    dir_ = unit_vector(lookat - pos);
    right_ = unit_vector(cross(dir_, world_up_));
    up_ = cross(right_, dir_);

    fovy_degree_ = fovy_degree;
    focal_length_ = focal_length;

    aspect_ratio_ = aspect_ratio;
    image_width_ = image_width;
    image_height_ = int(image_width / aspect_ratio);
    image_height_ = (image_height_ < 1) ? 1 : image_height_;

    samples_per_pixel_ = sample_per_pixel;
    max_recur_depth_ = max_recur_depth;

    // Why use (double(width) / height) instead of aspect_ratio?
    // aspect_ratio is the ideal value, but since width and height
    // are integers, the actual aspect ratio will be slightly off,
    // possibly something like 15.99xx / 9.0 or 16.00xx / 9.0.
    float theta = degrees_to_radians(fovy_degree_);
    viewport_height_ = 2.0 * std::tan(theta / 2.0) * focal_length_;
    viewport_width_ = viewport_height_ * (double(image_width_) / image_height_);
    image_.resize(image_width_ * image_height_);
  }

  void initialize_lens(int image_width, float aspect_ratio, point3 pos, vec3 lookat, float defocus_angle_, float focus_dist = 1,
                       float fovy_degree = 90, int sample_per_pixel = 100, int max_recur_depth = 5) {
    mode_ = kLens;
    pos_ = pos;
    world_up_ = vec3(0, 1, 0);
    dir_ = unit_vector(lookat - pos);
    right_ = unit_vector(cross(dir_, world_up_));
    up_ = cross(right_, dir_);

    fovy_degree_ = fovy_degree;
    focus_dist_ = focus_dist;

    aspect_ratio_ = aspect_ratio;
    image_width_ = image_width;
    image_height_ = int(image_width / aspect_ratio);
    image_height_ = (image_height_ < 1) ? 1 : image_height_;

    samples_per_pixel_ = sample_per_pixel;
    max_recur_depth_ = max_recur_depth;

    float theta = degrees_to_radians(fovy_degree_);
    viewport_height_ = 2.0 * std::tan(theta / 2.0) * focus_dist_;
    viewport_width_ = viewport_height_ * (double(image_width_) / image_height_);
    image_.resize(image_width_ * image_height_);

    auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle_ / 2));
    defocus_disk_u = right_ * defocus_radius;
    defocus_disk_v = up_ * defocus_radius;
  }

  // If light is not nullptr, then we will use importance sampling
  void render(std::ofstream &out, const hittable &world, const std::shared_ptr<const hittable> light = nullptr) {
    std::cout << "Start to render...\n";
    point3 delta_u = viewport_width_ * right_ / image_width_;
    point3 delta_v = -viewport_height_ * up_ / image_height_;

    point3 ray_dir00 =
        pos_ + focal_length_ * dir_ - viewport_width_ / 2.0 * right_ + viewport_height_ / 2.0 * up_ + 0.5 * (delta_u + delta_v);

    int max = 255;
    if (!out) {
      std::cerr << "Fail to open file.\n";
      return;
    }

    out << "P3\n";
    out << image_width_ << ' ' << image_height_ << '\n';
    out << max << '\n';

    // iota : fill the range [first, last) with sequentially increasing values, starting with value and repetitively evaluating ++value.
    std::vector<int> row_indices(image_height_);
    std::iota(row_indices.begin(), row_indices.end(), 0);

    // std::execution::par : this will use parallel execution policy
    std::for_each(std::execution::par_unseq, row_indices.begin(), row_indices.end(), [&](int i) {
      {
        std::lock_guard<std::mutex> lock(clog_mux_);
        std::cout << "Write Line: " << i << '\n';
      }
      for (int j = 0; j < image_width_; j++) {
        color pixel_color(0);
        for (int k = 0; k < samples_per_pixel_; k++) {
          ray emit_ray = generate_ray(i, j, delta_u, delta_v);
          pixel_color += ray_color(emit_ray, world, max_recur_depth_, light);
        }
        pixel_color /= samples_per_pixel_;
        image_[i * image_width_ + j] = pixel_color;
      }
    });

    std::for_each(image_.begin(), image_.end(), [&](const color &c) { write_color(out, c); });
    std::cout << "Done.\n";
  }

private:
  // Correspond to vulkan's Miss Shader
  color miss(ray r) const {
    hit_record record;
    if (background_) {
      sphere unit_sphere(r.origin(), 1.0f, nullptr);
      if (unit_sphere.hit(r, interval(0.001, infinity), record))
        return background_->sample(record.u, record.v, record.p);
      else
        return color(0);
    }
    return color(0);
  }

  // Get a color from a single ray, corresponding to vulkan's Closest Hit Shader
  color ray_color(const ray &r, const hittable &world, int iteration, const std::shared_ptr<const hittable> light) const {
    if (iteration <= 0)
      return color(0);

    hit_record record;
    if (!world.hit(r, interval(0.001, infinity), record))
      return miss(r);
    // hit object! record is valid, check the material of the object hit

    color color_from_emission = record.mat->emitted(r, record, record.u, record.v, record.p);

    scatter_record scatter_record;
    double pdf_value = 0;
    if (!record.mat->scatter(r, record, scatter_record))
      return color_from_emission;
    // 以下代表成功 scatter，接著計算下一條 ray 的方向並且得到顏色

    if (scatter_record.mode == scatter_mode::kDetermined) {
      ray scattered_ray = scatter_record.ray_scattered;
      color input_color = ray_color(scattered_ray, world, iteration - 1, light);
      return scatter_record.attenuation * input_color + color_from_emission;
    }

    // scatter_record.mode == scatter_mode::kRandom
    if (light == nullptr) { // not importance sampling
      auto pdf = scatter_record.pdf;
      ray scattered_ray = ray(record.p, pdf->generate(), r.time());
      pdf_value = pdf->value(scattered_ray.direction());

      // get pScattered value to calculate the color from the scattered ray
      double p_scattered = record.mat->p_scattered(r, record, scattered_ray);
      color input_color = ray_color(scattered_ray, world, iteration - 1, light);
      color color_from_scatter = (scatter_record.attenuation * p_scattered * input_color) / pdf_value;
      return color_from_scatter + color_from_emission;
    } else { // importance sampling
      auto p0 = std::make_shared<hittable_pdf>(*light, record.p);
      auto p1 = scatter_record.pdf;

      dual_pdf dual_pdf(p0, p1);
      ray scattered_ray = ray(record.p, dual_pdf.generate(), r.time());
      pdf_value = dual_pdf.value(scattered_ray.direction());

      // get pScattered value to calculate the color from the scattered ray
      double p_scattered = record.mat->p_scattered(r, record, scattered_ray);
      color input_color = ray_color(scattered_ray, world, iteration - 1, light);
      color color_from_scatter = (scatter_record.attenuation * p_scattered * input_color) / pdf_value;
      return color_from_scatter + color_from_emission;
    }
  }

  // Correspond to vulkan's Ray Generation Shader
  ray generate_ray(int y, int x, vec3 delta_u, vec3 delta_v) {
    if (mode_ == kPerspective) {
      point3 ray_dir00 = focal_length_ * dir_ - viewport_width_ / 2.0 * right_ + viewport_height_ / 2.0 * up_ + 0.5 * (delta_u + delta_v);
      point3 ray_dir = ray_dir00 + x * delta_u + y * delta_v;
      vec3 offset = sample_square();
      point3 rand_ray_dir = ray_dir + offset.x() * delta_u + offset.y() * delta_v;
      double ray_time = random_double();
      return {pos_, rand_ray_dir, ray_time};
    } else if (mode_ == kOrthnormal) {
      point3 pos00 = pos_ - viewport_width_ / 2.0 * right_ + viewport_height_ / 2.0 * up_ + 0.5 * (delta_u + delta_v);
      point3 pos = pos00 + x * delta_u + y * delta_v;
      vec3 offset = sample_square();
      point3 rand_pos = pos + offset.x() * delta_u + offset.y() * delta_v;
      double ray_time = random_double();
      return {rand_pos, dir_, ray_time};
    } else if (mode_ == kFisheye) {
      point3 ray_dir00 = focal_length_ * dir_ - viewport_width_ / 2.0 * right_ + viewport_height_ / 2.0 * up_ + 0.5 * (delta_u + delta_v);
      point3 ray_dir = ray_dir00 + x * delta_u + y * delta_v;
      vec3 offset = sample_square();
      point3 rand_ray_dir = ray_dir + offset.x() * delta_u + offset.y() * delta_v;

      // https://www.politesi.polimi.it/retrieve/330f59c4-1a55-40fe-a197-bf0f412af6e6/2022_12_Abelli_02.pdf
      double r = (rand_ray_dir - dir_).length();
      double theta = std::asin(r / focal_length_);
      vec3 v1 = unit_vector(dir_);
      vec3 v2 = unit_vector(rand_ray_dir);
      double b_prime = std::sqrt(std::sin(theta) * std::sin(theta) / (1 - dot(v1, v2) * dot(v1, v2)));
      double a_prime = std::cos(theta) - b_prime * dot(v1, v2);
      vec3 v3 = a_prime * v1 + b_prime * v2;

      double ray_time = random_double();
      return {pos_, v3, ray_time};
    } else if (mode_ == kLens) {
      vec3 offset = sample_square();
      point3 focus_plane00 = pos_ - viewport_width_ / 2.0 * right_ + viewport_height_ / 2.0 * up_ + 0.5 * (delta_u + delta_v);
      point3 ray_dir = focus_plane00 + x * delta_u + y * delta_v + offset.x() * delta_u + offset.y() * delta_v + focus_dist_ * dir_;
      point3 ray_origin = pos_ + sample_defocus_disk();
      ray_dir -= ray_origin;
      return {ray_origin, ray_dir};
    }
  }

  // Generate random vec in defocus disk, the defoucs disk's directions are streched by defocus_disk_u and defocus_disk_v
  vec3 sample_defocus_disk() const {
    auto p = random_in_unit_disk();
    return (p.x() * defocus_disk_u) + (p.y() * defocus_disk_v);
  }

  // Generate random vector in little square, forward is z-axis
  vec3 sample_square() { return vec3(random_double() - 0.5, random_double() - 0.5, 0); }

  // Generate random vector in disk with radius r
  vec3 sample_disk(double radius) const { return radius * random_in_unit_disk(); }

public:
  camera_mode mode_ = kPerspective;
  double aspect_ratio_;
  int image_width_;
  int image_height_;

  float fovy_degree_;

  int max_recur_depth_ = 10;

  int samples_per_pixel_;

  double viewport_height_;
  double viewport_width_;

  double focal_length_;

  double focus_dist_ = 3.4;
  double defocus_angle_ = 10.0;
  vec3 defocus_disk_u; // Defocus disk horizontal direction
  vec3 defocus_disk_v; // Defocus disk vertical direction

  point3 pos_;
  vec3 dir_;
  vec3 world_up_;
  vec3 up_;
  vec3 right_;

  std::mutex clog_mux_;

  std::vector<color> image_;
  std::shared_ptr<texture> background_;
};