#pragma once
#include "color.h"
#include "utility.h"
#define SPECTRUM_WAVELENGTH_MAX 750
#define SPECTRUM_WAVELENGTH_MIN 380
#define SPECTRUM_WAVELENGTH_STEP 5
#define SPECTRUM_WAVELENGTH_COUNT                        \
  ((SPECTRUM_WAVELENGTH_MAX - SPECTRUM_WAVELENGTH_MIN) / \
       SPECTRUM_WAVELENGTH_STEP +                        \
   1)
#include <array>
#include <iostream>

// https://www.youtube.com/watch?v=gnUYoQ1pwes
//  �H���@���T�إ��ղӭM�A���O�������P�Ϭq���i���A�ϬM�X���P���j��
//  short cone �������Ϭq���� 380 �� 550 nm�Apeak �b 440 nm
//  middle cone �������Ϭq���� 400 �� 650 nm�Apeak �b 540 nm
//  long cone �������Ϭq���� 430 �� 700 nm�Apeak �b 570 nm

// �o�T�� cone �������j�׳̱j���O middle cone�A�ڭ̳]�̤p�� 0�A�̤j��
// 1�A�ۤ񤧤U�A��L cone ���ȳ����� 1.0 ���ڭ̳q�`�|��T�� cone
// ���ƭȥ��W�ơA�ܦ� 0 �� 1 �������ȡA�M���ܦ��@�ӤT���Ŷ��A�o�T���Ŷ��N�ٰ�
// LMS Space�ALMS ���O�N�� long�Bmiddle�Bshort cone ���j��

// ��ڭ̱N�U�Ӫi���b LMS
// �Ŷ����e�X�@���ѼƦ��u�ɨåB��䤤�u�ʴ��ȡA�i�H�o��@�ӥ��骺�Y�]

// XYZ color space ���N�O LMS space ���@�ӽu���ഫ�A������
// X = 0.4002L + 0.7076M - 0.0808S
// Y = 0.2263L + 0.7152M + 0.0581S
// Z = 0.0000L + 0.0000M + 0.8253S

// CIE 1931 XYZ �O LMS ���@�ӽu���ഫ�A�ϱo�q XYZ
// �����I�¥~�����N���u���P�˪��C��A�ڭ̦b x+y+z=1
// �������W�e�@�ӤT���ΡA�åB�H�G�����Ѽƨ��o�ӤT�����W�����N�I�A�M������I����I�����u�W�A�̰��G�ת��C��e������ϡA�o�N�O
// CIE �������

// CIE 1931 xy Chromaticity Diagram: �� xy �Ӫ�� XYZ �Ŷ�����׮y��
class spectrum {
 public:
  // ��}�C������������l�Ʊj�׬� 0.0
  spectrum(double v = 0.0) : values() {
      for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
	  values[i] = v;
	}
  }

  // ����w�i�������бj�׳]�w�����w�ȡA��L�i�������бj�׬�
  // 0�A�|���J���ȦV�U����
  spectrum(double wavelength, double intensity) : values() {
    int index =
        (wavelength - SPECTRUM_WAVELENGTH_MIN) / SPECTRUM_WAVELENGTH_STEP;
    values[index] = intensity;
  }

  void add(double wavelength, double intensity) {
    int index =
        (wavelength - SPECTRUM_WAVELENGTH_MIN) / SPECTRUM_WAVELENGTH_STEP;
    values[index] += intensity;
  }

  void set(double wavelength, double intensity) {
    int index =
        (wavelength - SPECTRUM_WAVELENGTH_MIN) / SPECTRUM_WAVELENGTH_STEP;
    values[index] = intensity;
  }

  void set_all(double intensity) {
    for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
      values[i] = intensity;
    }
  }

  double operator[](int i) const { return values[i]; }
  double& operator[](int i) { return values[i]; }
  spectrum operator*(double c) const {
    spectrum result;
    for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
      result[i] = values[i] * c;
    }
    return result;
  }
  spectrum operator*(const spectrum& s) const {
    spectrum result;
    for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
      result[i] = values[i] * s[i];
    }
    return result;
  }

  spectrum operator/(double c) const {
    spectrum result;
    for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
      result[i] = values[i] / c;
    }
    return result;
  }

  spectrum operator+(const spectrum& s) const {
    spectrum result;
    for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
      result[i] = values[i] + s[i];
    }
    return result;
  }

  spectrum& operator+=(const spectrum& s) {
    for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
      values[i] += s[i];
    }
    return *this;
  }

  spectrum& operator/=(double c) {
    for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
      values[i] /= c;
    }
    return *this;
  }
  static int size() { return SPECTRUM_WAVELENGTH_COUNT; }
  static double wavelength(int index) {
	return SPECTRUM_WAVELENGTH_MIN + index * SPECTRUM_WAVELENGTH_STEP;
  }
 private:
  // �x�s�U�i�������бj�סA�q�C�찪
  std::array<double, SPECTRUM_WAVELENGTH_COUNT> values;
};

inline std::ostream& operator<<(std::ostream& out, const spectrum& s) {
  for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
    out << "wavelength: "
        << SPECTRUM_WAVELENGTH_MIN + i * SPECTRUM_WAVELENGTH_STEP
        << " nm, intensity:" << s[i] << std::endl;
  }
  return out;
}

const double Gamma = 0.80;

color wavelengthToRGB(double wavelength) {
  double factor;
  double r, g, b;

  if (wavelength < 380.0 || wavelength > 780.0) return color(0, 0, 0);

  if (wavelength >= 380 && wavelength < 440) {
    r = -(wavelength - 440) / (440 - 380);
    g = 0.0;
    b = 1.0;
  } else if (wavelength >= 440 && wavelength < 490) {
    r = 0.0;
    g = (wavelength - 440) / (490 - 440);
    b = 1.0;
  } else if (wavelength >= 490 && wavelength < 510) {
    r = 0.0;
    g = 1.0;
    b = -(wavelength - 510) / (510 - 490);
  } else if (wavelength >= 510 && wavelength < 580) {
    r = (wavelength - 510) / (580 - 510);
    g = 1.0;
    b = 0.0;
  } else if (wavelength >= 580 && wavelength < 645) {
    r = 1.0;
    g = -(wavelength - 645) / (645 - 580);
    b = 0.0;
  } else if (wavelength >= 645 && wavelength < 780) {
    r = 1.0;
    g = 0.0;
    b = 0.0;
  }

  // Let the intensity fall off near the vision limits
  if (wavelength >= 380 && wavelength < 420) {
    factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
  } else if (wavelength >= 420 && wavelength < 701) {
    factor = 1.0;
  } else if (wavelength >= 701 && wavelength < 781) {
    factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 700);
  } else {
    factor = 0.0;
  }

  std::array<int, 3> rgb;

  // Apply gamma correction
  rgb[0] =
      r == 0.0
          ? 0
          : static_cast<int>(std::round(255 * std::pow(r * factor, Gamma)));
  rgb[1] =
      g == 0.0
          ? 0
          : static_cast<int>(std::round(255 * std::pow(g * factor, Gamma)));
  rgb[2] =
      b == 0.0
          ? 0
          : static_cast<int>(std::round(255 * std::pow(b * factor, Gamma)));

  return color(rgb[0], rgb[1], rgb[2]);
}

color spectrumToRGB(const spectrum& s) {
  double totalIntensity = 0.0;
  double r = 0.0, g = 0.0, b = 0.0;

  for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; ++i) {
    double wavelength = SPECTRUM_WAVELENGTH_MIN + i * SPECTRUM_WAVELENGTH_STEP;
    double intensity = s[i];

    // Convert each wavelength to RGB
    color rgb = wavelengthToRGB(wavelength);

    // Accumulate weighted RGB values
    r += rgb.x() * intensity;
    g += rgb.y() * intensity;
    b += rgb.z() * intensity;

    // Accumulate total intensity
    totalIntensity += intensity;
  }
  // Normalize RGB values based on total intensity
  if (totalIntensity > 0) {
    r /= totalIntensity;
    g /= totalIntensity;
    b /= totalIntensity;
  }

  // Return the final color
  return color(static_cast<int>(std::round(r)), static_cast<int>(std::round(g)),
               static_cast<int>(std::round(b)));
}