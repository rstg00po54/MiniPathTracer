# MiniPathTracer

A from-scratch **C++17 CPU path tracer** rendering a Cornell Box with **multi-bounce global illumination, soft shadows, and red/green color bleeding**. This is a new project, separate from `opengltest/SoftMini3D`.

No GPU shading tricks: every pixel launches rays into the scene, samples an area light, and randomly traces diffuse bounces.

## Features (v0.1)

- Cornell Box scene: red/green walls, ceiling area light, two rotated boxes
- Monte Carlo path tracing, cosine-weighted hemisphere sampling, Russian roulette
- Next-event estimation for direct area-light sampling and realistic soft shadows
- Multi-bounce indirect illumination and color bleeding
- BVH traversal over triangle geometry (including boxes)
- Progressive sampling, CPU tile parallelism, deterministic per-pixel random seeds
- ACES-inspired tonemapping and sRGB output; binary PPM image file
- **Optional SDL2 preview** (disabled automatically when SDL2 isn't installed)
- Unit tests; standard CMake project on Windows / Linux

**Not yet included:** ImGui controls, glass/metal BSDFs, OBJ import, importance-sampling MIS, GPU acceleration, denoising. The initial implementation supports **diffuse surfaces** and one rectangular area emitter. These are future steps, not present features.

## Build

Install a C++17 compiler and CMake >= 3.16. SDL2 is optional (vcpkg/apt/homebrew are all fine). On Ubuntu, `sudo apt install libsdl2-dev` enables the preview. On Windows, use vcpkg's SDL2 package or set `SDL2_DIR` to your SDL2 CMake package.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
```

### Render

```bash
# Opens the live preview window if SDL2 was available at build time
./build/MiniPathTracer --width 480 --height 480 --spp 128 --out cornell.ppm

# CPU-only, no window or external dependencies
./build/MiniPathTracer --headless --width 480 --height 480 --spp 128 --threads 8 --out cornell.ppm

# Very quick test
./build/MiniPathTracer --headless --width 128 --height 128 --spp 8 --out quick.ppm
```

Windows multi-config generators commonly put the executable at `build/Release/MiniPathTracer.exe`. The PPM is a standard image file: GIMP, ImageMagick, IrfanView, and many other viewers can open it. To convert with ImageMagick: `magick cornell.ppm cornell.png`.

**SDL2 preview keys:** `+`/`-` exposure, `S` save (after completion), `Esc` close. The final render is saved when the preview window closes after rendering. Closing during a render cancels it without writing a misleading partial output.

### Tests

```bash
ctest --test-dir build --output-on-failure -C Release
```

## Renderer overview

`src/pathtracer.h/.cpp` implements ray / triangle intersections, BVH, geometry and the integrator. `src/main.cpp` provides CLI flags and an optional SDL2 preview. The camera faces the open side of the Cornell Box.

For diffuse materials, each path bounce adds a separately sampled light contribution (`NEE`) and continues in a cosine-sampled random direction. The unbiased Lambertian throughput update is simply the surface albedo. For this initial NEE-only diffuse integrator, indirect hits on an emissive triangle are intentionally not added a second time, avoiding double counting (camera rays still see the light). A later MIS integrator can combine both light and BSDF sampling.

The software path tracer is entirely independent from OpenGL. SDL2 is used only to *display* its pixel buffer, not calculate lighting.