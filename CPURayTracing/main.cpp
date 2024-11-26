#include <chrono>
#include <fstream>
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "aabb.h"
#include "bvh_node.h"
#include "camera.h"
#include "color.h"
#include "gltf_loader.h"
#include "hittable.h"
#include "hittable_list.h"
#include "onb.h"
#include "quad.h"
#include "ray.h"
#include "sphere.h"
#include "stb_image.h"
#include "triangle.h"
#include "utility.h"
#include "vec3.h"
#include "volumne.h"

void sphereflake(double radius, const vec3 &center, std::shared_ptr<material> mat, int iteration, hittable_list &world, double scale,
                 vec3 direction) {
  world.push_back(std::make_shared<sphere>(center, radius, mat));
  if (iteration == 0) return;

  onb uvw(direction);

  for (int i = 0; i < 6; ++i) {
    double angle = 2.0 * pi * i / 6.0;
    vec3 offset = uvw.transform(vec3(std::cos(angle), 0.0, std::sin(angle)));
    vec3 new_dir = offset;
    offset = offset * (radius + radius * scale);
    sphereflake(radius * scale, center + offset, mat, iteration - 1, world, scale, new_dir);
  }

  for (int i = 0; i < 3; ++i) {
    double angle = 2.0 * pi * i / 3.0;
    vec3 offset = uvw.transform(vec3(std::cos(angle) * std::cos(pi / 3.0), std::sin(pi / 3.0), std::sin(angle) * std::cos(pi / 3.0)));
    vec3 new_dir = offset;
    offset = offset * (radius + radius * scale);
    sphereflake(radius * scale, center + offset, mat, iteration - 1, world, scale, new_dir);
  }
}

void sphereflake_test() {
  auto skybox_and_fisheye = std::make_shared<picture_texture>(std::make_shared<image>("./assets/bathroom.exr"));

  hittable_list world;
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.5, 0.5, 0.5)));
  sphereflake(100, point3(0, 0, 0), metal_mat, 4, world, 0.25, vec3(0, 1, 0));

  bvh_node bvh(world);
  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(400, 1.0, point3(200), vec3(0, 0, 0), 1, 90.0, 50, 5);
  cam.background_ = skybox_and_fisheye;

  auto start = std::chrono::high_resolution_clock::now();
  cam.render(of, world);
  // cam.render(of, bvh); 
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Elapsed time: " << elapsed.count() << " seconds\n";
}

void three_mat_ball() {
  hittable_list world;
  auto ground_material = std::make_shared<lambertian>(std::make_shared<checker_texture>(color{1.0, 1.0, 1.0}, color{0.6, 0.6, 0.2}, 1.0));
  auto glass_mat = std::make_shared<dielectric>(std::make_shared<solid_color>(color{1.0, 1.0, 1.0}), 1.5);
  auto matte_mat = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.4, 0.2, 0.1)));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.7, 0.6, 0.5)), 0.0);

  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));
  world.push_back(std::make_shared<sphere>(point3(0, 1, 0), 1.0, glass_mat));
  world.push_back(std::make_shared<sphere>(point3(-4, 1, 0), 1.0, matte_mat));
  world.push_back(std::make_shared<sphere>(point3(4, 1, 0), 1.0, metal_mat));

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(400, 16.0 / 9.0, point3(13, 2, 3), vec3(0, 0, 0), 1, 20.0, 100, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void defocus_blur() {
  hittable_list world;
  auto ground_material = std::make_shared<lambertian>(std::make_shared<checker_texture>(color{1.0, 1.0, 1.0}, color{0.6, 0.6, 0.2}, 1.0));
  auto glass_mat = std::make_shared<dielectric>(std::make_shared<solid_color>(color{1.0, 1.0, 1.0}), 1.5);
  auto matte_mat = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.4, 0.2, 0.1)));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.7, 0.6, 0.5)), 0.0);

  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));
  world.push_back(std::make_shared<sphere>(point3(0, 1, 0), 1.0, glass_mat));
  world.push_back(std::make_shared<sphere>(point3(-4, 1, 0), 1.0, matte_mat));
  world.push_back(std::make_shared<sphere>(point3(4, 1, 0), 1.0, metal_mat));

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_lens(1280, 16.0 / 9.0, point3(13, 2, 3), vec3(1, 1, 1), 2.0, 15, 20.0, 1000, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void earth() {
  hittable_list world;
  auto earth_tex = std::make_shared<picture_texture>(std::make_shared<image>("./assets/earthmap.jpg"));
  world.push_back(std::make_shared<sphere>(point3(0, 0, -2), 1.0, std::make_shared<lambertian>(earth_tex)));

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(400, 16.0 / 9.0, point3(0, 0, 2), vec3(0, 0, -2), 1, 60.0, 10, 10);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void random_ball() {
  hittable_list world;
  auto ground_material = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.5, 0.5, 0.5)));
  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));
  for (int a = -11; a < 11; a++) {
    for (int b = -11; b < 11; b++) {
      auto choose_mat = random_double();
      point3 center(a + 0.9 * random_double(), 0.2, b + 0.9 * random_double());
      if ((center - point3(4, 0.2, 0)).length() > 0.9) {
        std::shared_ptr<material> sphere_material;
        if (choose_mat < 0.8) {
          // diffuse
          color albedo = random_vec() * random_vec();
          sphere_material = std::make_shared<lambertian>(std::make_shared<solid_color>(albedo));
          world.push_back(std::make_shared<sphere>(center, 0.2, sphere_material));
        } else if (choose_mat < 0.95) {
          // metal
          color albedo = random_vec(0.5, 1);
          auto fuzz = random_double(0, 0.5);
          sphere_material = std::make_shared<metal>(std::make_shared<solid_color>(albedo), fuzz);
          world.push_back(std::make_shared<sphere>(center, 0.2, sphere_material));
        } else {
          // glass
          sphere_material = std::make_shared<dielectric>(std::make_shared<solid_color>(color(0, 0, 0)), 1.5);
          world.push_back(std::make_shared<sphere>(center, 0.2, sphere_material));
        }
      }
    }
  }

  auto glass_mat = std::make_shared<dielectric>(std::make_shared<solid_color>(color(0, 0, 0)), 1.5);
  auto matte_mat = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.4, 0.2, 0.1)));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.7, 0.6, 0.5)), 0.0);
  world.push_back(std::make_shared<sphere>(point3(0, 1, 0), 1.0, glass_mat));
  world.push_back(std::make_shared<sphere>(point3(-4, 1, 0), 1.0, matte_mat));
  world.push_back(std::make_shared<sphere>(point3(4, 1, 0), 1.0, metal_mat));

  bvh_node bvh(world);

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(400, 16.0 / 9.0, point3(13, 2, 3), point3(0, 0, 0), 1, 20, 100, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, bvh);
}

void random_motion_ball() {
  hittable_list world;
  auto ground_material = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.5, 0.5, 0.5)));
  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));
  for (int a = -11; a < 11; a++) {
    for (int b = -11; b < 11; b++) {
      auto choose_mat = random_double();
      point3 center(a + 0.9 * random_double(), 0.2, b + 0.9 * random_double());
      point3 center2 = center + vec3(0, random_double(0, .3), 0);

      if ((center - point3(4, 0.2, 0)).length() > 0.9) {
        std::shared_ptr<material> sphere_material;
        if (choose_mat < 0.8) {
          // diffuse
          color albedo = random_vec() * random_vec();
          sphere_material = std::make_shared<lambertian>(std::make_shared<solid_color>(albedo));
          world.push_back(std::make_shared<sphere>(center, center2, 0.2, sphere_material));
        } else if (choose_mat < 0.95) {
          // metal
          color albedo = random_vec(0.5, 1);
          auto fuzz = random_double(0, 0.5);
          sphere_material = std::make_shared<metal>(std::make_shared<solid_color>(albedo), fuzz);
          world.push_back(std::make_shared<sphere>(center, center2, 0.2, sphere_material));
        } else {
          // glass
          sphere_material = std::make_shared<dielectric>(std::make_shared<solid_color>(color(0, 0, 0)), 1.5);
          world.push_back(std::make_shared<sphere>(center, center2, 0.2, sphere_material));
        }
      }
    }
  }

  auto glass_mat = std::make_shared<dielectric>(std::make_shared<solid_color>(color(0, 0, 0)), 1.5);
  auto matte_mat = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.4, 0.2, 0.1)));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.7, 0.6, 0.5)), 0.0);
  world.push_back(std::make_shared<sphere>(point3(0, 1, 0), 1.0, glass_mat));
  world.push_back(std::make_shared<sphere>(point3(-4, 1, 0), 1.0, matte_mat));
  world.push_back(std::make_shared<sphere>(point3(4, 1, 0), 1.0, metal_mat));

  bvh_node bvh(world);

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(400, 16.0 / 9.0, point3(13, 2, 3), point3(0, 0, 0), 1, 20, 200, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  // bvh 與 list 的差異比較 ?
  cam.render(of, bvh);
  // cam.render(of, world);
}

void marble_texture() {
  hittable_list world;
  auto perlin_tex = std::make_shared<perlin_texture>(4);
  world.push_back(std::make_shared<sphere>(point3(0, 2, 0), 2, std::make_shared<lambertian>(perlin_tex)));
  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, std::make_shared<lambertian>(perlin_tex)));

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(1080, 16.0 / 9.0, point3(18, 3, 4), vec3(0, 0, 0), 1, 30.0, 1000, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void cube_persp() {
  hittable_list world;
  auto left_red = std::make_shared<lambertian>(std::make_shared<solid_color>(color(1, 0.2, 0.2)));
  auto back_green = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 1.0, 0.2)));
  auto right_blue = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 0.2, 1.0)));
  auto upper_orange = std::make_shared<lambertian>(std::make_shared<solid_color>(color(1.0, 0.5, 0.0)));
  auto lower_teal = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 0.8, 0.8)));

  world.push_back(std::make_shared<quad>(point3(-2, -2, 4), vec3(0, 0, -4), vec3(0, 4, 0), left_red));
  world.push_back(std::make_shared<quad>(point3(-2, -2, 0), vec3(4, 0, 0), vec3(0, 4, 0), back_green));
  world.push_back(std::make_shared<quad>(point3(2, -2, 0), vec3(0, 0, 4), vec3(0, 4, 0), right_blue));
  world.push_back(std::make_shared<quad>(point3(-2, 2, 0), vec3(4, 0, 0), vec3(0, 0, 4), upper_orange));
  world.push_back(std::make_shared<quad>(point3(-2, -2, 4), vec3(4, 0, 0), vec3(0, 0, -4), lower_teal));

  camera cam;
  std::ofstream of("output.ppm");
  cam.initialize_perspective(400, 16.0 / 9.0, point3(1, 1, 9), point3(0, 0, 0), 1, 80);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void cube_orth() {
  hittable_list world;
  auto left_red = std::make_shared<lambertian>(std::make_shared<solid_color>(color(1, 0.2, 0.2)));
  auto back_green = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 1.0, 0.2)));
  auto right_blue = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 0.2, 1.0)));
  auto upper_orange = std::make_shared<lambertian>(std::make_shared<solid_color>(color(1.0, 0.5, 0.0)));
  auto lower_teal = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 0.8, 0.8)));

  world.push_back(std::make_shared<quad>(point3(-2, -2, 4), vec3(0, 0, -4), vec3(0, 4, 0), left_red));
  world.push_back(std::make_shared<quad>(point3(-2, -2, 0), vec3(4, 0, 0), vec3(0, 4, 0), back_green));
  world.push_back(std::make_shared<quad>(point3(2, -2, 0), vec3(0, 0, 4), vec3(0, 4, 0), right_blue));
  world.push_back(std::make_shared<quad>(point3(-2, 2, 0), vec3(4, 0, 0), vec3(0, 0, 4), upper_orange));
  world.push_back(std::make_shared<quad>(point3(-2, -2, 4), vec3(4, 0, 0), vec3(0, 0, -4), lower_teal));

  camera cam;
  std::ofstream of("output.ppm");
  cam.initialize_orthnormal(400, 10.0, 16.0 / 9.0, point3(1, 1, 9), point3(0, 0, 0), 100, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void cube_fisheye() {
  hittable_list world;
  auto left_red = std::make_shared<lambertian>(std::make_shared<solid_color>(color(1, 0.2, 0.2)));
  auto back_green = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 1.0, 0.2)));
  auto right_blue = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 0.2, 1.0)));
  auto upper_orange = std::make_shared<lambertian>(std::make_shared<solid_color>(color(1.0, 0.5, 0.0)));
  auto lower_teal = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.2, 0.8, 0.8)));

  world.push_back(std::make_shared<quad>(point3(-2, -2, 4), vec3(0, 0, -4), vec3(0, 4, 0), left_red));
  world.push_back(std::make_shared<quad>(point3(-2, -2, 0), vec3(4, 0, 0), vec3(0, 4, 0), back_green));
  world.push_back(std::make_shared<quad>(point3(2, -2, 0), vec3(0, 0, 4), vec3(0, 4, 0), right_blue));
  world.push_back(std::make_shared<quad>(point3(-2, 2, 0), vec3(4, 0, 0), vec3(0, 0, 4), upper_orange));
  world.push_back(std::make_shared<quad>(point3(-2, -2, 4), vec3(4, 0, 0), vec3(0, 0, -4), lower_teal));

  camera cam;
  std::ofstream of("output.ppm");
  cam.initialize_fisheye(400, 16.0 / 9.0, point3(1, 1, 9), point3(0, 0, 0), 1.0, 60);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void simple_light() {
  hittable_list world;
  auto pertext = std::make_shared<perlin_texture>(4);
  auto difflight = std::make_shared<diffuse_light>(std::make_shared<solid_color>(color(4, 4, 4)));

  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, std::make_shared<lambertian>(pertext)));
  world.push_back(std::make_shared<sphere>(point3(0, 2, 0), 2, std::make_shared<lambertian>(pertext)));
  world.push_back(std::make_shared<quad>(point3(3, 1, -2), vec3(2, 0, 0), vec3(0, 2, 0), difflight));

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(764, 16.0 / 9.0, point3(26, 3, 6), point3(0, 2, 0), 1, 20.0, 1000, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world);
}

void simple_light_earth() {
  hittable_list world;
  auto perlin_tex = std::make_shared<perlin_texture>(4);
  auto earth_tex = std::make_shared<picture_texture>(std::make_shared<image>("./assets/earthmap.jpg"));
  auto difflight = std::make_shared<diffuse_light>(std::make_shared<solid_color>(color(4, 4, 4)));

  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, std::make_shared<lambertian>(perlin_tex)));
  world.push_back(std::make_shared<sphere>(point3(0, 2, 0), 2, std::make_shared<lambertian>(earth_tex)));
  world.push_back(std::make_shared<quad>(point3(-1, 8, -1), vec3(2, 0, 0), vec3(0, 0, 2), difflight));

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(764, 16.0 / 9.0, point3(26, 3, 6), point3(0, 2, 0), 1, 20.0, 1000, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world);
}

void cornell_box() {
  hittable_list world;
  auto red = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.65, .05, .05}));
  auto white = std::make_shared<lambertian>(std::make_shared<solid_color>(color{0.73, 0.73, 0.73}));
  auto green = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.12, .45, .15}));
  auto light = std::make_shared<diffuse_light>(std::make_shared<solid_color>(color{15, 15, 15}));

  world.push_back(std::make_shared<quad>(point3(555, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), green));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), red));
  world.push_back(std::make_shared<quad>(point3(343, 554, 332), vec3(-130, 0, 0), vec3(0, 0, -105), light));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(555, 555, 555), vec3(-555, 0, 0), vec3(0, 0, -555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 555, 0), white));

  auto big_box = std::make_shared<translate>(vec3(100, 0, 100), box(point3(0), point3(165, 330, 165), white));
  auto small_box = std::make_shared<translate>(vec3(50, 0, 50), box(point3(0), point3(165, 165, 165), white));

  world.push_back(big_box);
  world.push_back(small_box);

  bvh_node bvh(world);

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(600, 16.0 / 9.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40.0, 40, 4);
  cam.background_ = solid_color::black;
  cam.render(of, bvh);
}

void skybox_and_fisheye() {
  auto skybox_and_fisheye = std::make_shared<picture_texture>(std::make_shared<image>("./assets/bathroom.exr"));
  hittable_list world;

  world.push_back(std::make_shared<sphere>(vec3(0), 1, std::make_shared<dielectric>(solid_color::white, 1.0)));

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_fisheye(720, 1, point3(1.1, 1.8, 1.1), point3(0, 0, 0), 1.0, 90, 100, 5);
  cam.background_ = skybox_and_fisheye;
  cam.render(of, world);
}

void smoke_cornell_box() {
  hittable_list world;
  auto red = std::make_shared<lambertian>(color(.65, .05, .05));
  auto white = std::make_shared<lambertian>(color(.73, .73, .73));
  auto green = std::make_shared<lambertian>(color(.12, .45, .15));
  auto light = std::make_shared<diffuse_light>(color(7, 7, 7));

  world.push_back(std::make_shared<quad>(point3(555, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), green));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), red));
  world.push_back(std::make_shared<quad>(point3(0, 555, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 555, 0), white));

  auto box1 = std::make_shared<translate>(vec3(265, 0, 285), std::make_shared<rotate_y>(box(point3(0), point3(150, 280, 150), white), 45));
  auto box2 = std::make_shared<translate>(vec3(130, 0, 65), std::make_shared<rotate_y>(box(point3(0), point3(140, 140, 140), white), -15));

  world.push_back(std::make_shared<volumne>(box1, 0.02, std::make_shared<solid_color>(color(0))));
  world.push_back(std::make_shared<volumne>(box2, 0.02, std::make_shared<solid_color>(color(1))));

  auto quad_light = std::make_shared<quad>(point3(113, 554, 127), vec3(330, 0, 0), vec3(0, 0, 305), light);
  world.push_back(quad_light);

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(512, 1.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40, 100, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world, quad_light);
}

void speculer_cornell_box() {
  hittable_list world;
  auto red = std::make_shared<lambertian>(color(.65, .05, .05));
  auto white = std::make_shared<lambertian>(color(.73, .73, .73));
  auto green = std::make_shared<lambertian>(color(.12, .45, .15));
  auto light = std::make_shared<diffuse_light>(color(7, 7, 7));
  auto met = std::make_shared<metal>(color(1.0), 0.0);

  world.push_back(std::make_shared<quad>(point3(555, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), green));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), red));
  world.push_back(std::make_shared<quad>(point3(0, 555, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 555, 0), white));

  auto box1 = std::make_shared<translate>(vec3(265, 0, 285), std::make_shared<rotate_y>(box(point3(0), point3(150, 280, 150), met), 20));
  auto box2 = std::make_shared<translate>(vec3(130, 0, 65), std::make_shared<rotate_y>(box(point3(0), point3(140, 140, 140), white), -15));

  world.push_back(box1);
  world.push_back(box2);

  auto quad_light = std::make_shared<quad>(point3(113, 554, 127), vec3(330, 0, 0), vec3(0, 0, 305), light);
  world.push_back(quad_light);

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(512, 1.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40, 100, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world, quad_light);
}

void rotate() {
  hittable_list world;
  auto red = std::make_shared<lambertian>(color(.65, .05, .05));
  auto white = std::make_shared<lambertian>(color(.73, .73, .73));
  auto green = std::make_shared<lambertian>(color(.12, .45, .15));
  auto light = std::make_shared<diffuse_light>(color(7, 7, 7));

  world.push_back(std::make_shared<quad>(point3(555, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), green));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), red));
  world.push_back(std::make_shared<quad>(point3(0, 555, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 555, 0), white));

  auto box1 = std::make_shared<rotate_z>(box(point3(265, 0, 295), point3(430, 330, 460), white), 15);
  world.push_back(box1);

  auto quad_light = std::make_shared<quad>(point3(113, 554, 127), vec3(330, 0, 0), vec3(0, 0, 305), light);
  world.push_back(quad_light);

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(600, 1.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40, 100, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world, quad_light);
}

void gltf_model_test() {
  gltf::GltfLoader model("./assets/Fox/glTF/Fox.gltf");
  auto &OutputPrimitives = model.getOutputPrimitives();

  auto skybox_and_fisheye = std::make_shared<picture_texture>(std::make_shared<image>("./assets/bathroom.exr"));

  // 加載三角形模型
  hittable_list world;
  for (int i = 0; i < OutputPrimitives.size(); i++) {
    auto &primitive = OutputPrimitives[i];

    std::vector<vec3> positions;
    std::vector<unsigned short> indices;

    // 根據 primitive position 的 type 轉成 vec3
    if (primitive.pos_type == DataType::kFloat) {
      for (int j = 0; j < primitive.positions.size(); j += 3 * sizeof(float)) {
        float x = 0, y = 0, z = 0;
        memcpy(&x, &primitive.positions[j], sizeof(float));
        memcpy(&y, &primitive.positions[j + 4], sizeof(float));
        memcpy(&z, &primitive.positions[j + 8], sizeof(float));
        positions.push_back(vec3(x, y, z));
      }
    }

    if (primitive.use_indices) {
      // 根據 primitive indices 的 type 轉成 unsigned short
      if (primitive.indices_type == DataType::kUnsignedShort) {
        for (int j = 0; j < primitive.indices.size(); j += sizeof(unsigned short)) {
          unsigned short index = 0;
          memcpy(&index, &primitive.indices[j], sizeof(unsigned short));
          indices.push_back(index);
        }
      }
    }

    if (primitive.use_indices) {
      // 根據 indices 把三角形加入到 world 中
      for (int i = 0; i < indices.size(); i += 3) {
        triangle tri(positions[indices[i]], positions[indices[i + 1]], positions[indices[i + 2]],
                     std::make_shared<lambertian>(std::make_shared<solid_color>(color(1.0f))));
        world.push_back(std::make_shared<triangle>(tri));
      }
    } else {
      for (int i = 0; i < positions.size(); i += 3) {
        triangle tri(positions[i], positions[i + 1], positions[i + 2],
                     std::make_shared<lambertian>(std::make_shared<solid_color>(color(1.0f))));
        world.push_back(std::make_shared<triangle>(tri));
      }
    }
  }

  std::ofstream of("output.ppm");
  camera cam;
  cam.initialize_perspective(400, 16.0 / 9.0, point3(0, 10000, -10000), point3(0, 0, 0), 1, 20.0, 20, 5);
  bvh_node bvh(world);
  cam.background_ = skybox_and_fisheye;
  cam.render(of, bvh);
}

void final_scene() {
  hittable_list boxes1;
  auto ground = std::make_shared<lambertian>(color(0.48, 0.83, 0.53));
  int boxes_per_side = 20;
  for (int i = 0; i < boxes_per_side; i++) {
    for (int j = 0; j < boxes_per_side; j++) {
      auto w = 100.0;
      auto x0 = -1000.0 + i * w;
      auto z0 = -1000.0 + j * w;
      auto y0 = 0.0;
      auto x1 = x0 + w;
      auto y1 = random_double(1, 101);
      auto z1 = z0 + w;
      boxes1.push_back(box(point3(x0, y0, z0), point3(x1, y1, z1), ground));
    }
  }

  hittable_list world;
  world.push_back(std::make_shared<bvh_node>(boxes1));

  auto light = std::make_shared<diffuse_light>(color(7, 7, 7));
  auto quad_light = std::make_shared<quad>(point3(123, 554, 147), vec3(300, 0, 0), vec3(0, 0, 265), light);
  world.push_back(quad_light);
  world.push_back(std::make_shared<sphere>(point3(260, 150, 45), 50, std::make_shared<dielectric>(1.5)));

  auto pertext = std::make_shared<perlin_texture>(8);
  world.push_back(std::make_shared<translate>(
      point3(180, 280, 400),
      std::make_shared<rotate_x>(std::make_shared<sphere>(point3(0), 80, std::make_shared<lambertian>(pertext)), -90)));

  camera cam;
  cam.initialize_perspective(512, 1.0, point3(478, 278, -600), point3(278, 278, 0), 1, 40.0, 100, 5);
  bvh_node bvh(world);
  std::ofstream of("output.ppm");
  cam.render(of, bvh);
}

int main() {
  switch (19) {
  case 1:
    three_mat_ball(); // OK
    break;
  case 14:
    defocus_blur(); // OK
    break;
  case 15:
    gltf_model_test(); // fix
    break;
  case 16:
    rotate(); // OK
    break;
  case 17:
    final_scene(); // OK
    break;
  case 2:
    earth(); // OK
    break;
  case 3:
    random_ball(); // OK
    break;
  case 4:
    random_motion_ball(); // OK
    break;
  case 5:
    marble_texture(); // OK
    break;
  case 6:
    cube_persp(); // OK
    break;
  case 12:
    cube_orth(); // OK
    break;
  case 7:
    simple_light();
    break;
  case 8:
    simple_light_earth();
    break;
  case 9:
    cornell_box(); // OK
    break;
  case 10:
    skybox_and_fisheye(); // OK
    break;
  case 11:
    smoke_cornell_box(); // OK
    break;
  case 18:
    speculer_cornell_box(); // OK
    break;
  case 19:
    sphereflake_test(); // OK
    break;
  default:
    break;
  }
}