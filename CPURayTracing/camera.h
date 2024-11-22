#pragma once
// The camera class will be responsible for two important jobs:
// 1. Construct and dispatch rays into the world.
// 2. Use the results of these rays to construct the rendered image.

#include <fstream>
#include <mutex>
#include <thread>

#include "hittable.h"
#include "interval.h"
#include "material.h"
#include "sphere.h"
#include "utility.h"
class camera {
 public:
  // 初始化 image 的長寬，並且設定 viewport，viewport 的高固定介於 -1 ~ 1 之間
  void initialize_perspective(int image_width, double aspect_ratio, point3 pos,
                              vec3 lookat, float focal_length = 1,
                              float fovy_degree = 90,
                              int sample_per_pixel = 100,
                              int max_recur_depth = 5) {
    mode_ = 1;
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
    // viewport_height = 2.0 * std::tan(theta / 2.0) * focus_dist_;
    viewport_width_ = viewport_height_ * (double(image_width_) / image_height_);

    image_.resize(image_width_ * image_height_);
  }

  void initialize_orthnormal(int image_width, double viewport_height,
                             double aspect_ratio, point3 pos, vec3 lookat,
                             int sample_per_pixel = 100,
                             int max_recur_depth = 5) {
    mode_ = 2;
    pos_ = pos;
    world_up_ = vec3(0, 1, 0);
    dir_ = unit_vector(lookat - pos);
    right_ = unit_vector(cross(dir_, world_up_));
    up_ = cross(right_, dir_);

    aspect_ratio_ = aspect_ratio;
    image_width_ = image_width;
    image_height_ = int(image_width / aspect_ratio);
    image_height_ = (image_height_ < 1) ? 1 : image_height_;

    samples_per_pixel_ = sample_per_pixel;
    max_recur_depth_ = max_recur_depth;

    viewport_height_ = viewport_height;
    viewport_width_ = viewport_height * (double(image_width_) / image_height_);
    image_.resize(image_width_ * image_height_);
  }

  void initialize_fisheye(int image_width, double aspect_ratio, point3 pos,
                          vec3 lookat, float focal_length = 1,
                          float fovy_degree = 90, int sample_per_pixel = 100,
                          int max_recur_depth = 5) {
    mode_ = 3;
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

  // defocus_angle 決定鏡片的大小，鏡片的高度等於 2.0 * std::tan(theta / 2.0) *
  // focus_dist_
  void initialize_lens(int image_width, float aspect_ratio, point3 pos,
                       vec3 lookat, float defocus_angle_, float focus_dist = 1,
                       float fovy_degree = 90, int sample_per_pixel = 100,
                       int max_recur_depth = 5) {
    mode_ = 4;
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

    auto defocus_radius =
        focus_dist * std::tan(degrees_to_radians(defocus_angle_ / 2));
    defocus_disk_u = right_ * defocus_radius;
    defocus_disk_v = up_ * defocus_radius;
  }

  // Generate a lot ray and render it, print the result to ofstream
  void render(std::ofstream& out, const hittable& world) {
    std::cout << "start to render\n";
    point3 delta_u = viewport_width_ * right_ / image_width_;
    point3 delta_v = -viewport_height_ * up_ / image_height_;

    point3 ray_dir00 = pos_ + focal_length_ * dir_ -
                       viewport_width_ / 2.0 * right_ +
                       viewport_height_ / 2.0 * up_ + 0.5 * (delta_u + delta_v);

    int max = 255;
    if (!out) {
      std::cerr << "fail to open file";
    }
    out << "P3\n";
    out << image_width_ << ' ' << image_height_ << '\n';
    out << max << '\n';

    // 使用單執行緒執行
    // for (int i = 0; i < image_height_; i++) {
    //  std::clog << "Write Line:" << i << '\n';
    //  for (int j = 0; j < image_width_; j++) {
    //    color pixel_color = color(0);
    //    for (int k = 0; k < samples_per_pixel_; k++) {
    //      ray emit_ray = generate_ray(i, j, delta_u, delta_v);
    //      pixel_color += ray_color(emit_ray, world, max_recur_depth_);
    //    }
    //    pixel_color /= samples_per_pixel_;
    //    image_[i * image_width_ + j] = pixel_color;
    //  }
    //}

    // 使用多執行緒執行，一個執行緒負責 image_height_ / thread_count_; 行
    std::vector<std::thread> threads;

    for (size_t i = 0; i < thread_count_; i++) {
      int begin_height = i * image_height_ / thread_count_;
      int height_count = image_height_ / thread_count_;

      threads.emplace_back(
          [&](int begin_height, int height_count) {
            for (int i = begin_height; i < begin_height + height_count; i++) {
              {
                std::lock_guard<std::mutex> lock(cerr_mux_);
                std::clog << "Write Line:" << i << '\n';
              }

              for (int j = 0; j < image_width_; j++) {
                color pixel_color = color(0);
                for (int k = 0; k < samples_per_pixel_; k++) {
                  ray emit_ray = generate_ray(i, j, delta_u, delta_v);
                  pixel_color += ray_color(emit_ray, world, max_recur_depth_);
                }
                pixel_color /= samples_per_pixel_;
                image_[i * image_width_ + j] = pixel_color;
              }
            }
          },
          begin_height, height_count);
    }
    for (auto& t : threads) {
      t.join();
    }

    // 等到所有的 thread 都結束
    std::clog << "done\n";
    for (size_t i = 0; i < image_.size(); i++) {
      write_color(out, image_[i]);
    }
  }

 private:
  // 啥都沒打到，只能打到 skybox
  color hit_skybox(ray r, hit_record& record) const {
    if (background_) {
      sphere unit_sphere(r.origin(), 1.0f, nullptr);
      if (unit_sphere.hit(r, interval(0.001, infinity), record)) {
        return background_->sample(record.u, record.v, record.p);
      }
    }
    return color(0, 0, 0);
  }

  // 得到 ray 的顏色
  color ray_color(const ray& r, const hittable& world, int iteration) const {
    if (iteration <= 0) return color(0.0, 0.0, 0.0);

    hit_record record;
    if (!world.hit(r, interval(0.001, infinity), record)) {
      // 啥都沒打到，代表打到 skybox
      return hit_skybox(r, record);
    }

    // 打中了，record 有效，看看打到的物體的 material 是啥，用其 material
    // 計算下一個 ray 的方向以及碰到的顏色
    color emission = record.mat->emitted(record.u, record.v, record.p);
    ray scattered;
    color attenuation;
    double pdf_value;

    if (record.mat->scatter(r, record, attenuation, scattered, pdf_value)) {
      double scatter_pdf = record.mat->scattering_pdf(r, record, scattered);
      color color_from_scatter = (attenuation * scatter_pdf *
                                  ray_color(scattered, world, iteration - 1)) /
                                 pdf_value;

      return attenuation * ray_color(scattered, world, iteration - 1) +
             emission;
    } else {
      return emission;
    }
  }

  // 以某個方向、某個時間產生
  // ray，並且隨機稍為的偏移，不同的鏡頭在這會產生不同的 ray
  ray generate_ray(int y, int x, vec3 delta_u, vec3 delta_v) {
    if (mode_ == 1) {  // perspective
      point3 ray_dir00 = focal_length_ * dir_ - viewport_width_ / 2.0 * right_ +
                         viewport_height_ / 2.0 * up_ +
                         0.5 * (delta_u + delta_v);
      point3 ray_dir = ray_dir00 + x * delta_u + y * delta_v;
      vec3 offset = sample_square();
      point3 rand_ray_dir =
          ray_dir + offset.x() * delta_u + offset.y() * delta_v;
      double ray_time = random_double();
      return {pos_, rand_ray_dir, ray_time};
    } else if (mode_ == 2) {  // orthonormal
      point3 pos00 = pos_ - viewport_width_ / 2.0 * right_ +
                     viewport_height_ / 2.0 * up_ + 0.5 * (delta_u + delta_v);
      point3 pos = pos00 + x * delta_u + y * delta_v;
      vec3 offset = sample_square();
      point3 rand_pos = pos + offset.x() * delta_u + offset.y() * delta_v;
      double ray_time = random_double();
      return {rand_pos, dir_, ray_time};
    } else if (mode_ == 3) {  // stereographic fish eyes
      point3 ray_dir00 = focal_length_ * dir_ - viewport_width_ / 2.0 * right_ +
                         viewport_height_ / 2.0 * up_ +
                         0.5 * (delta_u + delta_v);
      point3 ray_dir = ray_dir00 + x * delta_u + y * delta_v;
      vec3 offset = sample_square();
      point3 rand_ray_dir =
          ray_dir + offset.x() * delta_u + offset.y() * delta_v;

      // https://www.politesi.polimi.it/retrieve/330f59c4-1a55-40fe-a197-bf0f412af6e6/2022_12_Abelli_02.pdf
      double r = (rand_ray_dir - dir_).length();
      double theta = std::asin(r / focal_length_);
      vec3 v1 = unit_vector(dir_);
      vec3 v2 = unit_vector(rand_ray_dir);
      double b_prime = std::sqrt(std::sin(theta) * std::sin(theta) /
                                 (1 - dot(v1, v2) * dot(v1, v2)));
      double a_prime = std::cos(theta) - b_prime * dot(v1, v2);
      vec3 v3 = a_prime * v1 + b_prime * v2;

      double ray_time = random_double();
      return {pos_, v3, ray_time};
    } else if (mode_ == 4) {  // defocus blur
      vec3 offset = sample_square();
      point3 focus_plane00 = pos_ - viewport_width_ / 2.0 * right_ +
                             viewport_height_ / 2.0 * up_ +
                             0.5 * (delta_u + delta_v);
      point3 ray_dir = focus_plane00 + x * delta_u + y * delta_v +
                       offset.x() * delta_u + offset.y() * delta_v +
                       focus_dist_ * dir_;
      point3 ray_origin = defocus_disk_sample();
      ray_dir -= ray_origin;
      return {ray_origin, ray_dir};
    }
  }

  // 從半徑為 r 的單位圓採樣出一個點，並且加上當前的相機位置
  point3 defocus_disk_sample() const {
    auto p = random_in_unit_disk();
    return pos_ + (p.x() * defocus_disk_u) + (p.y() * defocus_disk_v);
  }

  // 回傳 x,y 介於 [-.5,-.5]~[+.5,+.5] 的隨機 vec3
  vec3 sample_square() {
    return vec3(random_double() - 0.5, random_double() - 0.5, 0);
  }

  // 隨機產生半徑為 radius 的 2D 圓內的向量(z 固定為 0，只有 xy 變化)
  vec3 sample_disk(double radius) const {
    return radius * random_in_unit_disk();
  }

 public:
  int mode_ = 1;  // 1 = perspective, 2 = orthnormal
  double aspect_ratio_;
  int image_width_;
  int image_height_;

  float fovy_degree_;

  int max_recur_depth_ = 50;

  int samples_per_pixel_;

  double viewport_height_;
  double viewport_width_;

  double focal_length_;

  double focus_dist_ = 3.4;
  double defocus_angle_ = 10.0;

  int thread_count_ = 8;  // 要求 image 的高度必須是 thread_count 的倍數

  // 圓型薄鏡片的垂直以及水平向量，決定了鏡片的朝向
  vec3 defocus_disk_u;  // Defocus disk horizontal radius
  vec3 defocus_disk_v;  // Defocus disk vertical radius

  point3 pos_;
  vec3 dir_;
  vec3 world_up_;
  vec3 up_;
  vec3 right_;

  std::mutex cerr_mux_;

  std::vector<color> image_;
  std::shared_ptr<texture> background_;
};