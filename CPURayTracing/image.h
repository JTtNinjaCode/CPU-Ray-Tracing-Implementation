#pragma once

#include <cstdlib>
#include <iostream>

#include "stb_image.h"
#include "tinyexr/tinyexr.h"

class image {
 public:
  image() = default;

  // Loads image data from the specified file. If the RTW_IMAGES environment
  // variable is defined, looks only in that directory for the image file.
  // If the image was not found, searches for the specified image file first
  // from the current directory, then in the images/ subdirectory, then the
  // _parent's_ images/ subdirectory, and then _that_ parent, on so on, for
  // six levels up. If the image was not loaded successfully, width() and
  // height() will return 0.
  image(const char *image_filename) {
    auto filename = std::string(image_filename);

    if (load_stb(filename)) return;
    if (load_exr(filename)) return;
    std::cerr << "ERROR: Could not load image file '" << image_filename
              << "'.\n";
  }

  ~image() {
    delete[] bdata;
    STBI_FREE(fdata);
  }

  bool load_stb(const std::string &filename) {
    // Loads the linear (gamma=1) image data from the given file name. Returns
    // true if the load succeeded. The resulting data buffer contains the three
    // [0.0, 1.0] floating-point values for the first pixel (red, then green,
    // then blue). Pixels are contiguous, going left to right for the width of
    // the image, followed by the next row below, for the full height of the
    // image.

    // Dummy out parameter: original components per pixel
    bytes_per_pixel = 3;
    auto n = bytes_per_pixel;
    fdata = stbi_loadf(filename.c_str(), &image_width, &image_height, &n,
                       bytes_per_pixel);
    if (fdata == nullptr) return false;

    bytes_per_scanline = image_width * bytes_per_pixel;
    convert_to_bytes();
    return true;
  }

  bool load_exr(const std::string &filename) {
    const char *err = nullptr;
    int ret =
        LoadEXR(&fdata, &image_width, &image_height, filename.c_str(), &err);

    if (ret != TINYEXR_SUCCESS) {
      if (err) {
        std::cerr << "Error loading EXR file: " << err << std::endl;
        FreeEXRErrorMessage(err);  // release memory of error message.
      }
      return ret;
    }
    bytes_per_pixel = 4;
    bytes_per_scanline = image_width * bytes_per_pixel;
    convert_to_bytes();
    return true;
  }

  int width() const { return (fdata == nullptr) ? 0 : image_width; }
  int height() const { return (fdata == nullptr) ? 0 : image_height; }

  const unsigned char *pixel_data(int x, int y) const {
    // Return the address of the three RGB bytes of the pixel at x,y. If there
    // is no image data, returns magenta.
    static unsigned char magenta[] = {255, 0, 255};
    if (bdata == nullptr) return magenta;

    x = clamp(x, 0, image_width);
    y = clamp(y, 0, image_height);

    return bdata + y * bytes_per_scanline + x * bytes_per_pixel;
  }

 private:
  int bytes_per_pixel;
  float *fdata = nullptr;          // Linear floating point pixel data
  unsigned char *bdata = nullptr;  // Linear 8-bit pixel data
  int image_width = 0;             // Loaded image width
  int image_height = 0;            // Loaded image height
  int bytes_per_scanline = 0;

  static int clamp(int x, int low, int high) {
    // Return the value clamped to the range [low, high).
    if (x < low) return low;
    if (x < high) return x;
    return high - 1;
  }

  static unsigned char float_to_byte(float value) {
    if (value <= 0.0) return 0;
    if (1.0 <= value) return 255;
    return static_cast<unsigned char>(256.0 * value);
  }

  // Convert the linear floating point pixel data to bytes, storing the
  // resulting byte data in the `bdata` member.
  void convert_to_bytes() {
    int total_bytes = image_width * image_height * bytes_per_pixel;
    bdata = new unsigned char[total_bytes];

    // Iterate through all pixel components, converting from [0.0, 1.0] float
    // values to unsigned [0, 255] byte values.

    auto *bptr = bdata;
    auto *fptr = fdata;
    for (auto i = 0; i < total_bytes; i++, fptr++, bptr++)
      *bptr = float_to_byte(*fptr);
  }
};
