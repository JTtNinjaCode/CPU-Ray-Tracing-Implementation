#pragma once
#include "vec3.h"

class frame {
public:
  vec3 transform(vec3 v) const { return 0 + v.x() * x + v.y() * y + v.z() * z; }
  vec3 trans_to_conanical(vec3 v) const {}
  vec3 trans_to_onb(vec3 v) const {}
  vec3 x;
  vec3 y;
  vec3 z;
  vec3 o;
};

// use normal class to construct an orthnormal basis, select two orthogonal vectors in the plane perpendicular to normal and normal as the
// basis, only ensure that the y direction points to normal, x, z have infinite possibilities, but ensure that they are two orthogonal
// vectors in the plane
class onb : public frame {
public:
  onb(const vec3 &normal) {
    y = unit_vector(normal);
    // a ���`�ӻ��n�O vec3(1, 0, 0)(�ۤv��)�A���p�G normal
    // �Ӿa��Ӥ�V�A�K�L�k�i�}�X basis�A�]���אּ�t�@�Ӥ�V�A�b�o�̧ڨϥ�
    // vec3(0, 0, 1)
    vec3 a = (std::fabs(y.x()) > 0.9) ? vec3(0, 0, 1) : vec3(1, 0, 0);
    z = unit_vector(cross(y, a));
    x = cross(y, z);
  }
};