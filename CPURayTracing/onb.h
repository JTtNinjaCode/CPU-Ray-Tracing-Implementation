#pragma once
#include "vec3.h"

class frame {
 public:
  vec3 transform(vec3 v) { return 0 + v.x() * x + v.y() * y + v.z() * z; }
  vec3 trans_to_conanical(vec3 v) {}
  vec3 trans_to_onb(vec3 v) {}
  vec3 x = vec3(0.0);
  vec3 y = vec3(0.0);
  vec3 z = vec3(0.0);
  vec3 o = vec3(0.0);
};
// �ϥ� normal �غc�X�@�� orthnormal basis�A��ܫ����� normal
// ������������ӫ����V�q�H�� normal �@�� basis�A�u�O�� y direction ���V normal
// x, z ���i��ʦ��L���h�ءA���O�ҬO���������⫫���V�q

class onb : public frame {
 public:
  onb(const vec3& normal) {
    y = unit_vector(normal);
    // a ���`�ӻ��n�O vec3(1, 0, 0)(�ۤv��)�A���p�G normal
    // �Ӿa��Ӥ�V�A�K�L�k�i�}�X basis�A�]���אּ�t�@�Ӥ�V�A�b�o�̧ڨϥ�
    // vec3(0, 0, 1)
    vec3 a = (std::fabs(y.x()) > 0.9) ? vec3(0, 0, 1) : vec3(1, 0, 0);
    z = unit_vector(cross(y, a));
    x = cross(y, z);
  }
};