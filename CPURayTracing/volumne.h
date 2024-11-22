#pragma once
#include "aabb.h"
#include "hittable.h"
#include "utility"

// constant �۷��@�ӥ]�˪���A�L���̭��� hittable
// �ᤩ�@�ӱK�סA�åB�����|��L�Ӫ���A�]���O
// volumne�A�]���ҥ]�����󥲶��O�����~������
class volumne : public hittable {
 public:
  volumne(std::shared_ptr<hittable> boundary, double density,
          std::shared_ptr<texture> tex) {
    boundary_ = boundary;
    density_ = density;
    phase_function_ = std::make_shared<isotropic>(tex);
  }

  // �I�� volumne �H��A�I�쪺�I�|�b volumne ��
  bool hit(const ray& r, interval ray_t, hit_record& record) const {
    hit_record record1, record2;
    // record ���O�N��I�� volumne ������I
    if (!boundary_->hit(r, interval::universe, record1)) return false;
    if (!boundary_->hit(r, interval(record1.t + 0.0001, infinity), record2))
      return false;

    // ���� record �� clamp �� ray_t �d��
    if (record1.t < ray_t.min) record1.t = ray_t.min;
    if (record2.t > ray_t.max) record2.t = ray_t.max;

    if (record1.t >= record2.t) return false;
    if (record1.t < 0) record1.t = 0;

    // ray �b box �̭����h��
    auto ray_length = r.direction().length();
    auto distance_inside_boundary = (record2.t - record1.t) * ray_length;

    // ray �b box ��
    auto hit_distance = -1.0 / density_ * std::log(random_double());

    // false �N���������L��� hittable ����  ԣ���S����
    if (hit_distance > distance_inside_boundary) return false;
    record.t = record1.t + hit_distance / ray_length;
    record.p = r.at(record.t);
    record.normal = vec3(1, 0, 0);  // arbitrary
    record.front_face = true;       // arbitrary
    record.mat = phase_function_;
    return true;
  }

  aabb get_bounding_box() const { return boundary_->get_bounding_box(); }

 private:
  std::shared_ptr<hittable> boundary_;
  double density_;
  std::shared_ptr<material> phase_function_;

  // phase
  // function�]�ۦ��ơ^�O�@�Ӵy�z���u�b�ѻP����]�p���B�ϡB�G��B�ֽ����^���o�ʹ��g��A�u�ۤ��P��V�����g�j�ת��ƾǨ�ơC���Ψӭp����u�q�@�Ӥ�V�i�J�����A�H���P�������}���誺���v�M�j�פ��G�C
  // ����u��L�ѻP����ɡA���|�P���褤���L�ɬۤ��@�ΡA�o�جۤ��@�η|�ɭP���u���ܨ��V�]���g�^�C�ۦ��ƥΨӨM�w���u���g�᪺�s��V�P���V�������������Y�A�H�ΨC�Ӥ�V�W���g�����j�סC
  //
};