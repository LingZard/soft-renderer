# soft-renderer (learning project)

This is a CPU-only, learning-oriented software renderer. The goal is to make a clear and readable implementation of a classic rasterization pipeline and the core bits of PBR.

Heads-up: PBRShader is still work-in-progress, and there are known bugs to fix. This is intentional for learning and iteration.

## How to run

1) Update example asset paths to match your machine (or make them relative to the project root `assets/`):
- `examples/pbr_chandelier.cpp`
- `examples/african_head.cpp`

2) Build and run with xmake:
```sh
xmake -y
xmake run pointcloud
# others:
# xmake run african_head
# xmake run triangles
# xmake run wireframe
# xmake run pointcloud
```

## What’s included

- Core pipeline
  - Vertex processing → clipping → perspective divide → rasterization → fragment shading → output
  - Triangle rasterizer using bounding-box scan and edge functions
  - Perspective-correct interpolation for attributes and depth
  - Viewport transform and depth buffer testing

- Clipping
  - Lines: Liang–Barsky
  - Triangles: Sutherland–Hodgman (to clip space, then retriangulate)

- Shaders
  - `FlatShader`: solid vertex color
  - `TexturedShader`: textured with UVs
  - `PBRShader` (WIP): physically based BRDF with metallic-roughness inputs


- Texturing
  - SamplerState: address (Clamp/Repeat/Mirror/Border), filtering (Nearest/Bilinear), mip filter (None/Nearest/Linear)
  - Mipmap generation: simple 2×2 box downsampling
  - UV origin configurable (TopLeft / BottomLeft)
  - Per-texture color space: sRGB vs Linear
  - Screen-space UV gradients for LOD selection (finite differences on neighbors)

- Color & output
  - Internal linear space processing
  - Exposure (EV) + tone mapping: Linear→sRGB, Reinhard, ACES

- IO & assets
  - glTF (TinyGLTF): loads positions / normals / texcoord0 (+ optional tangent/texcoord1), factors, texture paths, and `KHR_texture_transform`
  - Images via stb_image
  - For simplicity, the glTF loader takes the first mesh/primitive

- Framework
  - Window/input (minifb), Orbit camera controller, Perspective/Orthographic cameras
  - Ground plane and XYZ axes helpers for scene orientation

- Examples
  - `pbr_chandelier.cpp` (PBR, glTF)
  - `african_head.cpp` (textured OBJ)
  - `triangles.cpp`, `wireframe.cpp`, `pointcloud.cpp`

## Status
- PBRShader: WIP (features are being filled in and tuned)
- Some known bugs remain;
- Extras like IBL/shadows/clearcoat are intentionally left out for now
