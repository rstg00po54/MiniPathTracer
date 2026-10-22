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
- ACES-inspired tonemapping and sRGB output; BMP/PPM image files
- **Interactive SDL2 path-traced preview**: orbit/pan/zoom camera, adjustable light position, light power and sampling quality, fast low-resolution updates during motion and progressive refinement when still (only enabled when SDL2 is installed)
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
./build/MiniPathTracer --width 480 --height 480 --spp 128 --out cornell.bmp

# Lamp can also be moved in headless renders (offsets in scene units):
./build/MiniPathTracer --headless --width 320 --height 320 --spp 64 \
  --light-x 0.35 --light-y -0.20 --light-z 0.25 --out moved_light.bmp

# CPU-only, no window or external dependencies
./build/MiniPathTracer --headless --width 480 --height 480 --spp 128 --threads 8 --out cornell.bmp

# Very quick test
./build/MiniPathTracer --headless --width 128 --height 128 --spp 8 --out quick.bmp
```

Windows multi-config generators commonly put the executable at `build/Release/MiniPathTracer.exe`. BMP files open directly in Windows Photos/Paint. You can still output a binary PPM by choosing a `.ppm` filename.

**SDL2 live preview controls:**
- **Left mouse drag:** orbit around the target; **mouse wheel:** zoom
- **WASD:** move horizontally, **Q/E:** up/down, **R:** reset camera
- **Arrow Left/Right:** move rectangular lamp horizontally (X)
- **Arrow Up/Down:** move lamp toward rear/front wall (Z)
- **Page Up/Page Down:** raise/lower the lamp (Y); constrained below the ceiling and above the tall box
- **Home:** reset the lamp to its original position
- **[ / ]:** adjust actual emitter power (restarts path tracing)
- Moving the lamp updates its visible triangles, light sampling and BVH together; the window title displays its XYZ center.
- **1 / 2 / 3:** set stationary quality to 16 / 64 / 256 samples per pixel
- **+ / -:** exposure; **F12** or **Ctrl+S:** save the currently displayed BMP; **Esc:** close

Movement triggers a fast 160-pixel-wide (maximum dimension), 1-sample preview; stopping for about 0.2 s switches back to full-resolution progressive accumulation. The first full-resolution sample is displayed immediately, then additional samples refine the image. **This is responsive CPU interaction, not guaranteed 60 FPS**: actual update rate depends on resolution, CPU, and whether the app runs inside a VM.

Ubuntu: install SDL2 **before** configuring CMake:
```bash
sudo apt install libsdl2-dev
cmake -S . -B build -DMPT_ENABLE_SDL2=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
./build/MiniPathTracer --width 320 --height 320 --spp 32
```
CMake must print `SDL2 preview enabled`. If SDL2 is absent it silently falls back to headless, which only saves an image. `--headless` intentionally disables the window.

### Tests

```bash
ctest --test-dir build --output-on-failure -C Release
```

## Renderer overview

`src/pathtracer.h/.cpp` implements ray / triangle intersections, BVH, geometry and the integrator. `src/main.cpp` provides CLI flags and an optional SDL2 preview. The camera faces the open side of the Cornell Box.

For diffuse materials, each path bounce adds a separately sampled light contribution (`NEE`) and continues in a cosine-sampled random direction. The unbiased Lambertian throughput update is simply the surface albedo. For this initial NEE-only diffuse integrator, indirect hits on an emissive triangle are intentionally not added a second time, avoiding double counting (camera rays still see the light). A later MIS integrator can combine both light and BSDF sampling.

The software path tracer is entirely independent from OpenGL. SDL2 is used only to *display* its pixel buffer, not calculate lighting.