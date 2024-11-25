# Ray Tracing in C++

## Introduction
This project is a personal implementation of the ray tracing renderer inspired by *Ray Tracing in One Weekend* by Peter Shirley. It extends the foundational concepts of ray tracing to produce visually realistic images, offering customizable features such as camera modes, materials, and lighting.

## Features
- Camera Modes:
  - Perspective
  - Orthographic
  - Fisheye
  - Lens with Depth of Field
- Physically Based Rendering:
  - Support for diffuse, reflective, and refractive materials.
  - Ray bounces to simulate global illumination.
- Multithreaded Rendering:
  - Leverages modern C++ features like `std::execution` for parallelization.
- Sampling:
  - Implements anti-aliasing using stratified sampling.
- Extensibility:
  - Easy to add new objects, materials, and features.

## Examples
Here are some sample renderings produced by this ray tracer:

1. Cornell Box
   ![Cornell Box Example](#)
2. Reflective Sphere
   ![Reflective Sphere Example](#)
3. Fisheye
   ![Fisheye Lens Example](#)

## How It Works
1. Ray Generation:
   Rays are cast from a virtual camera into the scene based on the selected camera mode.
2. Intersection Testing:
   Each ray is tested against objects in the scene to find intersections.
3. Shading:
   Lighting and material interactions (e.g., reflection, refraction, or absorption) are computed at each intersection.
4. Recursive Ray Tracing:
   For reflective or refractive surfaces, recursive rays are traced until a maximum depth is reached.
5. Parallel Processing:
   The image is rendered in parallel to fully utilize CPU cores.

## Getting Started
### Prerequisites
- Compiler: A C++17 or later compatible compiler (e.g., GCC, Clang, MSVC).
- Libraries: No external libraries are required; the project uses the C++ standard library.

### Building
1. Clone this repository:

```bash
   git clone https://github.com/your-username/.git
   cd raytracer
```

## Usage
The configuration file allows you to adjust rendering parameters such as:

- Image resolution
- Samples per pixel
- Maximum recursion depth
- Scene setup (objects, materials, lighting)

## Performance
This project uses multi-threading and parallel (std::execution::par) to significantly reduce rendering time. Depending on the complexity of the scene and the hardware used, rendering performance scales effectively across multiple CPU cores.

## Acknowledgments
This project was heavily inspired by the Ray Tracing in One Weekend series by Peter Shirley. Special thanks to the ray tracing community for their guidance and resources.