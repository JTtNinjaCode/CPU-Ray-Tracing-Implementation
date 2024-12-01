#include <chrono>
#include <fstream>
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

std::ofstream of;
void sphereflake_recur(double radius, const vec3 &center, std::shared_ptr<material> mat, int iteration, hittable_list &world, double scale,
                       vec3 direction) {
  world.push_back(std::make_shared<sphere>(center, radius, mat));
  if (iteration == 0)
    return;

  onb uvw(direction);

  for (int i = 0; i < 6; ++i) {
    double angle = 2.0 * pi * i / 6.0;
    vec3 offset = uvw.transform(vec3(std::cos(angle), 0.0, std::sin(angle)));
    vec3 new_dir = offset;
    offset = offset * (radius + radius * scale);
    sphereflake_recur(radius * scale, center + offset, mat, iteration - 1, world, scale, new_dir);
  }

  for (int i = 0; i < 3; ++i) {
    double angle = 2.0 * pi * i / 3.0;
    vec3 offset = uvw.transform(vec3(std::cos(angle) * std::cos(pi / 3.0), std::sin(pi / 3.0), std::sin(angle) * std::cos(pi / 3.0)));
    vec3 new_dir = offset;
    offset = offset * (radius + radius * scale);
    sphereflake_recur(radius * scale, center + offset, mat, iteration - 1, world, scale, new_dir);
  }
}

void sphereflake() {
  auto skybox_and_fisheye = std::make_shared<picture_texture>(std::make_shared<image>("./assets/bathroom.exr"));

  hittable_list world;
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.5, 0.5, 0.5)));
  sphereflake_recur(100, point3(0, 0, 0), metal_mat, 4, world, 0.25, vec3(0, 1, 0));

  bvh_node bvh(world);

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

void three_material_ball() {
  hittable_list world;
  auto ground_material = std::make_shared<lambertian>(std::make_shared<checker_texture>(color{1.0, 1.0, 1.0}, color{0.6, 0.6, 0.2}, 1.0));
  auto glass_mat = std::make_shared<dielectric>(std::make_shared<solid_color>(color{1.0, 1.0, 1.0}), 1.5);
  auto matte_mat = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.4, 0.2, 0.1)));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.7, 0.6, 0.5)), 0.0);

  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));
  world.push_back(std::make_shared<sphere>(point3(0, 1, 0), 1.0, glass_mat));
  world.push_back(std::make_shared<sphere>(point3(-4, 1, 0), 1.0, matte_mat));
  world.push_back(std::make_shared<sphere>(point3(4, 1, 0), 1.0, metal_mat));

  camera cam;
  cam.initialize_perspective(1280, 16.0 / 9.0, point3(13, 2, 3), vec3(0, 0, 0), 1, 20.0, 100, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void three_material_ball_with_defocus_blur() {
  hittable_list world;
  auto ground_material = std::make_shared<lambertian>(std::make_shared<checker_texture>(color{1.0, 1.0, 1.0}, color{0.6, 0.6, 0.2}, 1.0));
  auto glass_mat = std::make_shared<dielectric>(std::make_shared<solid_color>(color{1.0, 1.0, 1.0}), 1.5);
  auto matte_mat = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.4, 0.2, 0.1)));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.7, 0.6, 0.5)), 0.0);

  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));
  world.push_back(std::make_shared<sphere>(point3(0, 1, 0), 1.0, glass_mat));
  world.push_back(std::make_shared<sphere>(point3(-4, 1, 0), 1.0, matte_mat));
  world.push_back(std::make_shared<sphere>(point3(4, 1, 0), 1.0, metal_mat));

  camera cam;
  cam.initialize_lens(1280, 16.0 / 9.0, point3(13, 2, 3), vec3(1, 1, 1), 2.0, 15, 20.0, 1000, 5);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, world);
}

void random_motion_ball() {
  hittable_list world;
  auto ground_material = std::make_shared<lambertian>(std::make_shared<checker_texture>(color{1.0, 1.0, 1.0}, color{0.6, 0.6, 0.2}, 1.0));
  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));

  for (int a = -11; a < 11; a++) {
    for (int b = -11; b < 11; b++) {
      auto choose_mat = random_double();
      point3 center1(a + 0.7 * random_double(), 0.2, b + 0.7 * random_double());
      point3 center2 = center1 + vec3(0, random_double(0, .15), 0);

      if ((center1 - point3(4, 0.2, 0)).length() > 0.9) {
        std::shared_ptr<material> sphere_material;
        if (choose_mat < 0.3) {
          // no ball
        } else if (choose_mat < 0.8) {
          // diffuse
          color albedo = random_vec() * random_vec();
          sphere_material = std::make_shared<lambertian>(std::make_shared<solid_color>(albedo));
          world.push_back(std::make_shared<sphere>(center1, center2, 0.2, sphere_material));
        } else if (choose_mat < 0.95) {
          // metal
          color albedo = random_vec(0.5, 1);
          // auto fuzz = random_double(0, 0.5); // TODO Fix
          sphere_material = std::make_shared<metal>(std::make_shared<solid_color>(albedo), 0.0);
          world.push_back(std::make_shared<sphere>(center1, center2, 0.2, sphere_material));
        } else {
          // glass
          sphere_material = std::make_shared<dielectric>(std::make_shared<solid_color>(color(1)), 1.5);
          world.push_back(std::make_shared<sphere>(center1, center2, 0.2, sphere_material));
        }
      }
    }
  }

  auto glass_mat = std::make_shared<dielectric>(std::make_shared<solid_color>(color(1)), 1.5);
  auto matte_mat = std::make_shared<lambertian>(std::make_shared<solid_color>(color(0.4, 0.2, 0.1)));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.7, 0.6, 0.5)), 0.0);
  world.push_back(std::make_shared<sphere>(point3(0, 1, 0), 1.0, glass_mat));
  world.push_back(std::make_shared<sphere>(point3(-4, 1, 0), 1.0, matte_mat));
  world.push_back(std::make_shared<sphere>(point3(4, 1, 0), 1.0, glass_mat));

  bvh_node bvh(world);

  camera cam;
  cam.initialize_perspective(1280, 16.0 / 9.0, point3(13, 2, 3), point3(0, 0, 0), 1, 20, 20, 50);
  cam.background_ = std::make_shared<solid_color>(color(0.7, 0.8, 1.0));
  cam.render(of, bvh);
}

void simple_light_earth() {
  hittable_list world;
  auto perlin_tex = std::make_shared<perlin_texture>(4);
  auto earth_tex = std::make_shared<picture_texture>(std::make_shared<image>("./assets/earthmap.jpg"));
  auto light = std::make_shared<diffuse_light>(std::make_shared<solid_color>(color(9)));

  world.push_back(std::make_shared<sphere>(point3(0, -1000, 0), 1000, std::make_shared<lambertian>(perlin_tex)));
  world.push_back(std::make_shared<sphere>(point3(0, 2, 0), 2, std::make_shared<gloss>(earth_tex, 1.0, 0.08)));

  auto quad_light = std::make_shared<quad>(point3(-2, 7, -2), vec3(4, 0, 0), vec3(0, 0, 4), light);
  world.push_back(quad_light);

  camera cam;
  cam.initialize_perspective(1280, 16.0 / 9.0, point3(26, 3, 6), point3(0, 2, 0), 1, 20.0, 500, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world, quad_light);
}

void skybox_and_fisheye() {
  auto skybox = std::make_shared<picture_texture>(std::make_shared<image>("./assets/bathroom.exr"));
  hittable_list world;

  world.push_back(std::make_shared<sphere>(vec3(0), 1, std::make_shared<dielectric>(solid_color::white, 1.0)));

  camera cam;
  cam.initialize_fisheye(600, 1, point3(1.1, 1.8, 1.1), point3(0, 0, 0), 1.0, 90, 500, 5);
  cam.background_ = skybox;
  cam.render(of, world);
}

void skybox_and_motion_blur() {
  auto skybox = std::make_shared<picture_texture>(std::make_shared<image>("./assets/bathroom.exr"));
  hittable_list world;
  auto earth_tex = std::make_shared<picture_texture>(std::make_shared<image>("./assets/earthmap.jpg"));

  world.push_back(std::make_shared<sphere>(vec3(-0.2, 0, 0), vec3(0.2, 0, 0), 1, std::make_shared<lambertian>(earth_tex)));

  camera cam;
  cam.initialize_perspective(600, 1, point3(0, 0, 4), point3(0, 0, 0), 1.0, 70, 500, 5);
  cam.background_ = skybox;
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
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(555, 555, 555), vec3(-555, 0, 0), vec3(0, 0, -555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 555, 0), white));

  auto big_box = std::make_shared<translate>(vec3(100, 0, 200), box(point3(0), point3(165, 330, 165), white));
  auto small_box = std::make_shared<translate>(vec3(50, 0, 100), box(point3(0), point3(165, 165, 165), white));
  auto quad_light = std::make_shared<quad>(point3(343, 554, 332), vec3(-130, 0, 0), vec3(0, 0, -105), light);

  world.push_back(big_box);
  world.push_back(small_box);
  world.push_back(quad_light);

  bvh_node bvh(world);

  camera cam;
  cam.initialize_perspective(600, 1.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40.0, 40, 4);
  cam.background_ = solid_color::black;
  cam.render(of, bvh, quad_light);
}

void cornell_box_with_volume() {
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

  camera cam;
  cam.initialize_perspective(600, 1.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40, 100, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world, quad_light);
}

void cornell_box_with_specular_box() {
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

  camera cam;
  cam.initialize_perspective(600, 1.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40, 500, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world, quad_light);
}

void cornell_box_with_rotated_box() {
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

  camera cam;
  cam.initialize_perspective(600, 1.0, point3(278, 278, -800), point3(278, 278, 0), 1, 40, 100, 5);
  cam.background_ = solid_color::black;
  cam.render(of, world, quad_light);
}

void cornell_box_with_glossy_ball() {
  hittable_list world;

  auto red = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.65, .05, .05}));
  auto white = std::make_shared<lambertian>(std::make_shared<solid_color>(color{0.73, 0.73, 0.73}));
  auto green = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.12, .45, .15}));
  auto light = std::make_shared<diffuse_light>(std::make_shared<solid_color>(color(8)));

  world.push_back(std::make_shared<quad>(point3(18, -4, -3), vec3(0, 8, 0), vec3(0, 0, 6), green));
  world.push_back(std::make_shared<quad>(point3(0, -4, -3), vec3(0, 8, 0), vec3(0, 0, 6), red));
  world.push_back(std::make_shared<quad>(point3(0, -4, -3), vec3(18, 0, 0), vec3(0, 0, 6), white));
  world.push_back(std::make_shared<quad>(point3(0, 4, -3), vec3(18, 0, 0), vec3(0, 0, 6), white));
  world.push_back(std::make_shared<quad>(point3(0, -4, -3), vec3(18, 0, 0), vec3(0, 10, 0), white));

  auto earth_tex = std::make_shared<picture_texture>(std::make_shared<image>("./assets/earthmap.jpg"));
  auto gloss_100 = std::make_shared<gloss>(earth_tex, 1.0, 1.00);
  auto gloss_40 = std::make_shared<gloss>(earth_tex, 1.0, 0.40);
  auto gloss_15 = std::make_shared<gloss>(earth_tex, 1.0, 0.15);
  auto gloss_02 = std::make_shared<gloss>(earth_tex, 1.0, 0.02);

  world.push_back(std::make_shared<sphere>(point3(3, 0, -0.5), 1.25, gloss_100));
  world.push_back(std::make_shared<sphere>(point3(7, 0, -0.5), 1.25, gloss_40));
  world.push_back(std::make_shared<sphere>(point3(11, 0, -0.5), 1.25, gloss_15));
  world.push_back(std::make_shared<sphere>(point3(15, 0, -0.5), 1.25, gloss_02));

  auto quad_light = std::make_shared<quad>(point3(5.5, 3.995, -1.25), vec3(7, 0, 0), vec3(0, 0, 2.5), light);
  world.push_back(quad_light);

  bvh_node bvh(world);

  camera cam;
  cam.initialize_perspective(760, 19.0 / 9.0, point3(9, 0, 15.2), point3(9, 0, 1), 1, 40.0, 1000, 10);
  cam.background_ = solid_color::black;
  cam.render(of, bvh, quad_light);
}

void glass_fox() {
  gltf::GltfLoader model("./assets/Fox/glTF/Fox.gltf");

  auto skybox = std::make_shared<picture_texture>(std::make_shared<image>("./assets/bathroom.exr"));

  // load model
  auto &OutputPrimitives = model.getOutputPrimitives();
  hittable_list world;
  for (int i = 0; i < OutputPrimitives.size(); i++) {
    auto &primitive = OutputPrimitives[i];

    std::vector<vec3> positions;
    std::vector<unsigned short> indices;

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
      if (primitive.indices_type == DataType::kUnsignedShort) {
        for (int j = 0; j < primitive.indices.size(); j += sizeof(unsigned short)) {
          unsigned short index = 0;
          memcpy(&index, &primitive.indices[j], sizeof(unsigned short));
          indices.push_back(index);
        }
      }
    }

    // use EBO or not.
    if (primitive.use_indices) {
      for (int i = 0; i < indices.size(); i += 3) {
        triangle tri(positions[indices[i]], positions[indices[i + 1]], positions[indices[i + 2]],
                     std::make_shared<dielectric>(std::make_shared<solid_color>(color(1.0f)), 1.5));
        world.push_back(std::make_shared<triangle>(tri));
      }
    } else {
      for (int i = 0; i < positions.size(); i += 3) {
        triangle tri(positions[i], positions[i + 1], positions[i + 2],
                     std::make_shared<dielectric>(std::make_shared<solid_color>(color(1.0f)), 1.5));
        world.push_back(std::make_shared<triangle>(tri));
      }
    }
  }

  camera cam;
  cam.initialize_perspective(600, 1.0, point3(220, 220, 220), point3(0, 20, 0), 1, 45.0, 200, 5);
  bvh_node bvh(world);
  cam.background_ = skybox;
  cam.render(of, bvh);
}

void perlin_texture_ball() {
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
  cam.initialize_perspective(600, 1.0, point3(478, 278, -600), point3(278, 278, 0), 1, 40.0, 500, 5);
  bvh_node bvh(world);

  cam.render(of, bvh);
}

void sponza() {
  gltf::GltfLoader model("./assets/Sponza/glTF/Sponza.gltf");

  // load model
  auto &OutputPrimitives = model.getOutputPrimitives();
  hittable_list world;
  for (int i = 0; i < OutputPrimitives.size(); i++) {
    auto &primitive = OutputPrimitives[i];

    std::vector<vec3> positions;
    std::vector<unsigned short> indices;

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
      if (primitive.indices_type == DataType::kUnsignedShort) {
        for (int j = 0; j < primitive.indices.size(); j += sizeof(unsigned short)) {
          unsigned short index = 0;
          memcpy(&index, &primitive.indices[j], sizeof(unsigned short));
          indices.push_back(index);
        }
      }
    }

    // use EBO or not.
    if (primitive.use_indices) {
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

  auto light = std::make_shared<diffuse_light>(color(10));
  auto quad_light = std::make_shared<quad>(point3(0, 1200, 0), vec3(500, 0, 0), vec3(0, 0, 500), light);
  world.push_back(quad_light);

  std::cout << "bvh building..." << std::endl;
  bvh_node bvh(world);
  std::cout << "bvh build done." << std::endl;

  camera cam;
  cam.initialize_perspective(200, 1.0, point3(500, 320, 90), point3(0, 280, 0), 1, 45.0, 30, 5);
  cam.render(of, bvh, quad_light);
}

void white_sphere() {
  hittable_list world;
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(1.0)), 0.1);
  // auto red = std::make_shared<lambertian>(std::make_shared<solid_color>(color(1.0, 0.0, 0.0)));
  world.push_back(std::make_shared<sphere>(point3(0, 0, 0), 1, metal_mat));

  camera cam;
  cam.initialize_perspective(400, 1.0, point3(13, 2, 3), point3(0, 0, 0), 1, 20, 100, 5);
  cam.background_ = std::make_shared<solid_color>(color(1.0));
  cam.render(of, world);
}

void different_fuzz_metal() {
  hittable_list world;

  auto red = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.65, .05, .05}));
  auto white = std::make_shared<lambertian>(std::make_shared<solid_color>(color{0.73, 0.73, 0.73}));
  auto green = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.12, .45, .15}));
  auto light = std::make_shared<diffuse_light>(std::make_shared<solid_color>(color(7)));

  world.push_back(std::make_shared<quad>(point3(18, -4, -3), vec3(0, 8, 0), vec3(0, 0, 6), green));
  world.push_back(std::make_shared<quad>(point3(0, -4, -3), vec3(0, 8, 0), vec3(0, 0, 6), red));
  world.push_back(std::make_shared<quad>(point3(0, -4, -3), vec3(18, 0, 0), vec3(0, 0, 6), white));
  world.push_back(std::make_shared<quad>(point3(0, 4, -3), vec3(18, 0, 0), vec3(0, 0, 6), white));
  world.push_back(std::make_shared<quad>(point3(0, -4, -3), vec3(18, 0, 0), vec3(0, 10, 0), white));

  auto fuzz_00 = std::make_shared<metal>(solid_color::white, 0.00);
  auto fuzz_25 = std::make_shared<metal>(solid_color::white, 0.25);
  auto fuzz_50 = std::make_shared<metal>(solid_color::white, 0.50);
  auto fuzz_75 = std::make_shared<metal>(solid_color::white, 0.75);
  auto fuzz_10 = std::make_shared<metal>(solid_color::white, 1.00); // same as lambertian

  world.push_back(std::make_shared<sphere>(point3(2, 0, -0.5), 1.25, fuzz_00));
  world.push_back(std::make_shared<sphere>(point3(5.5, 0, -0.5), 1.25, fuzz_25));
  world.push_back(std::make_shared<sphere>(point3(9, 0, -0.5), 1.25, fuzz_50));
  world.push_back(std::make_shared<sphere>(point3(12.5, 0, -0.5), 1.25, fuzz_75));
  world.push_back(std::make_shared<sphere>(point3(16, 0, -0.5), 1.25, fuzz_10));

  auto quad_light = std::make_shared<quad>(point3(5.5, 3.995, -1.25), vec3(7, 0, 0), vec3(0, 0, 2.5), light);
  world.push_back(quad_light);

  bvh_node bvh(world);

  camera cam;
  cam.initialize_perspective(760, 19.0 / 9.0, point3(9, 0, 15.2), point3(9, 0, 1), 1, 40.0, 1000, 10);
  cam.background_ = solid_color::black;
  cam.render(of, bvh, quad_light);
}

void infinite_reflection() {
  hittable_list world;
  auto red = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.65, .05, .05}));
  auto white = std::make_shared<lambertian>(std::make_shared<solid_color>(color{0.73, 0.73, 0.73}));
  auto green = std::make_shared<lambertian>(std::make_shared<solid_color>(color{.12, .45, .15}));
  auto metal_mat = std::make_shared<metal>(std::make_shared<solid_color>(color(0.80)), 0.0);
  auto light = std::make_shared<diffuse_light>(std::make_shared<solid_color>(color{5}));

  world.push_back(std::make_shared<quad>(point3(555, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), green));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), red));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.push_back(std::make_shared<quad>(point3(555, 555, 555), vec3(-555, 0, 0), vec3(0, 0, -555), white));
  world.push_back(std::make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 555, 0), metal_mat));
  world.push_back(std::make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 555, 0), metal_mat));

  auto earth_tex = std::make_shared<picture_texture>(std::make_shared<image>("./assets/earthmap.jpg"));
  world.push_back(std::make_shared<sphere>(point3(460, 80, 80), 60, std::make_shared<gloss>(earth_tex, 0.97, 0.18)));

  auto cube = std::make_shared<translate>(vec3(130, 0, 65), std::make_shared<rotate_y>(box(point3(0), point3(140, 140, 140), white), -15));
  world.push_back(cube);

  auto quad_light = std::make_shared<quad>(point3(113, 554, 127), vec3(330, 0, 0), vec3(0, 0, 305), light);
  world.push_back(quad_light);

  bvh_node bvh(world);

  camera cam;
  cam.initialize_perspective(600, 1.0, point3(500, 290, 550), point3(400, 278, 0), 1, 40.0, 1000, 30);
  cam.background_ = solid_color::black;
  cam.render(of, bvh, quad_light);
}

void test_perlin_noise() {
  hittable_list world;

  auto perlin_noise = std::make_shared<lambertian>(std::make_shared<perlin_texture>(1));
  auto perlin_quad = std::make_shared<quad>(point3(0, 0, 0), vec3(10, 0, 0), vec3(0, 10, 0), perlin_noise);
  world.push_back(perlin_quad);

  camera cam;
  cam.initialize_orthnormal(400, 1, 10, vec3(5, 5, 1), vec3(5, 5, 0), 10, 5);
  cam.background_ = solid_color::white;
  cam.render(of, world);
}

void test_value_noise() {
  hittable_list world;

  auto value_noise = std::make_shared<lambertian>(std::make_shared<value_texture>(40));
  auto value_quad = std::make_shared<quad>(point3(0, 0, 0), vec3(40, 0, 0), vec3(0, 40, 0), value_noise);
  world.push_back(value_quad);

  camera cam;
  cam.initialize_orthnormal(400, 1, 20, vec3(20, 20, 1), vec3(20, 20, 0), 10, 5);
  cam.background_ = solid_color::white;
  cam.render(of, world);
}

void test_worley_noise() {
  hittable_list world;

  auto worley_noise = std::make_shared<lambertian>(std::make_shared<worley_texture>());
  auto worley_quad = std::make_shared<quad>(point3(0, 0, 0), vec3(40, 0, 0), vec3(0, 40, 0), worley_noise);
  world.push_back(worley_quad);

  camera cam;
  cam.initialize_orthnormal(400, 1, 20, vec3(20, 20, 1), vec3(20, 20, 0), 10, 5);
  cam.background_ = solid_color::white;
  cam.render(of, world);
}

void test_voronoi_noise() {
  hittable_list world;

  auto voronoi_noise = std::make_shared<lambertian>(std::make_shared<voronoi_texture>());
  auto voronoi_quad = std::make_shared<quad>(point3(0, 0, 0), vec3(40, 0, 0), vec3(0, 40, 0), voronoi_noise);
  world.push_back(voronoi_quad);

  camera cam;
  cam.initialize_orthnormal(400, 1, 20, vec3(20, 20, 1), vec3(20, 20, 0), 10, 5);
  cam.background_ = solid_color::white;
  cam.render(of, world);
}

int main() {
  std::vector<std::pair<std::string, std::function<void()>>> test_cases = {
      {"Three Material Ball", three_material_ball},
      {"Three Material Ball with Defocus Blur", three_material_ball_with_defocus_blur},
      {"Random Motion Ball", random_motion_ball}, // FIX
      {"Simple Light Earth", simple_light_earth},
      {"Skybox and Fisheye", skybox_and_fisheye},
      {"Skybox and Motion Blur", skybox_and_motion_blur},
      {"Cornell Box", cornell_box},
      {"Cornell Box with Volume", cornell_box_with_volume},
      {"Cornell Box with Rotated Box", cornell_box_with_rotated_box},
      {"Cornell Box with Specular Box", cornell_box_with_specular_box},
      {"Glass Fox", glass_fox},
      {"Perlin Texture Ball", perlin_texture_ball},
      {"Sphereflake", sphereflake},
      {"Sponza", sponza},
      {"White Sphere", white_sphere},
      {"Different Fuzz Metal", different_fuzz_metal},
      {"Infinite Reflection", infinite_reflection},
      {"Cornell Box with Glossy Ball", cornell_box_with_glossy_ball},
      {"Test Perlin Noise", test_perlin_noise},
      {"Test Value Noise", test_value_noise},
      {"Test Worley Noise", test_worley_noise},
      {"Test Voronoi Noise", test_voronoi_noise},
  };

  // Prompt for output file name
  std::cout << "Input output file name (.ppm), or press Enter for default ('output.ppm'): ";
  std::string output_file;
  std::getline(std::cin, output_file);

  // If no input, set the default file name
  if (output_file.empty())
    output_file = "output.ppm";
  of.open(output_file);

  if (!of) {
    std::cout << "Failed to open file: " << output_file << std::endl;
    return 1;
  }

  // Display scene options
  std::cout << "Choose a scene to render:" << std::endl;
  for (size_t i = 0; i < test_cases.size(); ++i)
    std::cout << i + 1 << ". " << test_cases[i].first << std::endl;

  int which;
  std::cout << "Enter the number of the scene you want to render: ";
  std::cin >> which;

  // Render the selected scene
  if (which > 0 && which <= static_cast<int>(test_cases.size()))
    test_cases[which - 1].second();
  else
    std::cout << "Invalid selection. Please choose a valid number." << std::endl;

  return 0;
}