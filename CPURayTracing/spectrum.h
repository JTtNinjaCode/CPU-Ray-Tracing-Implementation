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
//  人眼共有三種光椎細胞，分別接收不同區段的波長，反映出不同的強度
//  short cone 反應的區段介於 380 到 550 nm，peak 在 440 nm
//  middle cone 反應的區段介於 400 到 650 nm，peak 在 540 nm
//  long cone 反應的區段介於 430 到 700 nm，peak 在 570 nm

// 這三種 cone 反應的強度最強的是 middle cone，我們設最小為 0，最大為
// 1，相比之下，其他 cone 的值都不及 1.0 但我們通常會把三個 cone
// 的數值正規化，變成 0 到 1 之間的值，然後變成一個三維空間，這三維空間就稱做
// LMS Space，LMS 分別代表 long、middle、short cone 的強度

// 當我們將各個波長在 LMS
// 空間內畫出一條參數曲線時並且對其中線性插值，可以得到一個立體的凸包

// XYZ color space 其實就是 LMS space 的一個線性轉換，公式為
// X = 0.4002L + 0.7076M - 0.0808S
// Y = 0.2263L + 0.7152M + 0.0581S
// Z = 0.0000L + 0.0000M + 0.8253S

// CIE 1931 XYZ 是 LMS 的一個線性轉換，使得從 XYZ
// 的原點朝外的任意直線都同樣的顏色，我們在 x+y+z=1
// 的平面上畫一個三角形，並且以二維的參數取得該三角型上的任意點，然後取原點到該點的直線上，最高亮度的顏色畫成馬蹄圖，這就是
// CIE 的馬蹄圖

// CIE 1931 xy Chromaticity Diagram: 用 xy 來表示 XYZ 空間的色度座標
class spectrum {
 public:
  // 把陣列內的元素都初始化強度為 0.0
  spectrum(double v = 0.0) : values() {
      for (int i = 0; i < SPECTRUM_WAVELENGTH_COUNT; i++) {
	  values[i] = v;
	}
  }

  // 把指定波長的光譜強度設定為指定值，其他波長的光譜強度為
  // 0，會把輸入的值向下取整
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
  // 儲存各波長的光譜強度，從低到高
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