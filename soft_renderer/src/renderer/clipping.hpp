#pragma once

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "../core/math/vec.hpp"
#include "pipeline_types.hpp"

namespace soft_renderer {
namespace renderer {

inline const std::array<math::Vec4f, 6> CanonicalViewVolumePlanes = {
    {{1, 0, 0, 1},
     {-1, 0, 0, 1},
     {0, 1, 0, 1},
     {0, -1, 0, 1},
     {0, 0, 1, 1},
     {0, 0, -1, 1}}};

enum class LineClipAlgorithm {
  CohenSutherland,
  LiangBarsky,
};

enum class TriangleClipAlgorithm {
  SutherlandHodgman,
};

template <Varying TVaryings>
class ILineClipper {
 public:
  virtual ~ILineClipper() = default;

  virtual std::optional<std::pair<TVaryings, TVaryings>> clip(
      const TVaryings& v0, const TVaryings& v1,
      const std::array<math::Vec4f, 6>& planes =
          CanonicalViewVolumePlanes) const = 0;
};

template <Varying TVaryings>
class ITriangleClipper {
 public:
  virtual ~ITriangleClipper() = default;

  virtual std::vector<TVaryings> clip(const TVaryings& v0, const TVaryings& v1,
                                      const TVaryings& v2,
                                      const std::array<math::Vec4f, 6>& planes =
                                          CanonicalViewVolumePlanes) const = 0;
};

template <Varying TVaryings>
class CohenSutherlandLineClipper : public ILineClipper<TVaryings> {
 private:
  enum OutCode {
    INSIDE = 0,  // 000000
    LEFT = 1,    // 000001
    RIGHT = 2,   // 000010
    BOTTOM = 4,  // 000100
    TOP = 8,     // 001000
    NEAR = 16,   // 010000
    FAR = 32     // 100000
  };

  OutCode compute_out_code(const math::Vec4f& point,
                           const std::array<math::Vec4f, 6>& planes) const {
    OutCode code = INSIDE;

    float w = point.w();

    if (point.x() < -w) code = static_cast<OutCode>(code | LEFT);
    if (point.x() > w) code = static_cast<OutCode>(code | RIGHT);
    if (point.y() < -w) code = static_cast<OutCode>(code | BOTTOM);
    if (point.y() > w) code = static_cast<OutCode>(code | TOP);
    if (point.z() < -w) code = static_cast<OutCode>(code | NEAR);
    if (point.z() > w) code = static_cast<OutCode>(code | FAR);

    return code;
  }

  TVaryings compute_intersection(const TVaryings& v0, const TVaryings& v1,
                                 OutCode out_code) const {
    float t = 0.0f;
    math::Vec4f p0 = v0.clip_pos;
    math::Vec4f p1 = v1.clip_pos;
    math::Vec4f delta = p1 - p0;

    if (out_code & TOP) {
      t = (p0.w() - p0.y()) / (delta.w() - delta.y());
    } else if (out_code & BOTTOM) {
      t = (-p0.w() - p0.y()) / (delta.w() - delta.y());
    } else if (out_code & RIGHT) {
      t = (p0.w() - p0.x()) / (delta.w() - delta.x());
    } else if (out_code & LEFT) {
      t = (-p0.w() - p0.x()) / (delta.w() - delta.x());
    } else if (out_code & FAR) {
      t = (p0.w() - p0.z()) / (delta.w() - delta.z());
    } else if (out_code & NEAR) {
      t = (-p0.w() - p0.z()) / (delta.w() - delta.z());
    }

    return v0 + (v1 - v0) * t;
  }

 public:
  std::optional<std::pair<TVaryings, TVaryings>> clip(
      const TVaryings& v0, const TVaryings& v1,
      const std::array<math::Vec4f, 6>& planes) const override {
    TVaryings p0 = v0;
    TVaryings p1 = v1;

    for (int iter = 0; iter < 6; ++iter) {
      OutCode out_code0 = compute_out_code(p0.clip_pos, planes);
      OutCode out_code1 = compute_out_code(p1.clip_pos, planes);

      if ((out_code0 | out_code1) == 0) {
        return std::make_pair(p0, p1);
      } else if ((out_code0 & out_code1) != 0) {
        return std::nullopt;
      } else {
        OutCode out_code_out = out_code0 ? out_code0 : out_code1;
        TVaryings intersection = compute_intersection(p0, p1, out_code_out);
        if (out_code_out == out_code0) {
          p0 = intersection;
        } else {
          p1 = intersection;
        }
      }
    }

    return std::nullopt;
  }
};

template <Varying TVaryings>
class LiangBarskyLineClipper : public ILineClipper<TVaryings> {
 public:
  std::optional<std::pair<TVaryings, TVaryings>> clip(
      const TVaryings& v0, const TVaryings& v1,
      const std::array<math::Vec4f, 6>& planes) const override {
    Vec4f p0 = v0.clip_pos;
    Vec4f p1 = v1.clip_pos;

    float t0 = 0.0f;
    float t1 = 1.0f;
    Vec4f delta = p1 - p0;
    for (const auto& plane : planes) {
      float dist_p0 = p0.dot(plane);
      float dist_delta = delta.dot(plane);

      if (std::abs(dist_delta) < math::Vec4f::epsilon()) {
        // 'delta' on the plane, which means the line is parallel to the plane
        if (dist_p0 < 0.0f) {
          return std::nullopt;
        }
        continue;
      }

      float t = -dist_p0 / dist_delta;

      if (dist_delta > 0.0f) {
        t0 = std::max(t0, t);
      } else {
        t1 = std::min(t1, t);
      }
    }

    if (t0 < t1 - math::Vec4f::epsilon()) {
      return std::make_pair(v0 + (v1 - v0) * t0, v0 + (v1 - v0) * t1);
    }

    return std::nullopt;
  }
};

template <Varying TVaryings>
class SutherlandHodgmanTriangleClipper : public ITriangleClipper<TVaryings> {
 private:
  bool is_inside(const TVaryings& vertex, const Vec4f& plane) const {
    return vertex.clip_pos.dot(plane) >= 0.0;
  }

  TVaryings compute_intersection(const TVaryings& inside_vertex,
                                 const TVaryings& outside_vertex,
                                 const Vec4f& plane) const {
    Vec4f p0 = inside_vertex.clip_pos;
    Vec4f p1 = outside_vertex.clip_pos;

    float d0 = p0.dot(plane);
    float d1 = p1.dot(plane);

    if (std::abs(d0 - d1) < Vec4f::epsilon()) {
      return inside_vertex;
    }

    float t = d0 / (d0 - d1);
    return inside_vertex + (outside_vertex - inside_vertex) * t;
  }

  std::vector<TVaryings> clip_against_plane(
      const std::vector<TVaryings>& input_vertices, const Vec4f& plane) const {
    std::vector<TVaryings> output_vertices;

    if (input_vertices.empty()) {
      return output_vertices;
    }

    for (size_t i = 0; i < input_vertices.size(); ++i) {
      const TVaryings& current = input_vertices[i];
      const TVaryings& previous =
          input_vertices[(i - 1 + input_vertices.size()) %
                         input_vertices.size()];

      bool current_inside = is_inside(current, plane);
      bool previous_inside = is_inside(previous, plane);

      if (current_inside) {
        if (!previous_inside) {
          TVaryings intersection =
              compute_intersection(current, previous, plane);
          output_vertices.push_back(intersection);
        }
        output_vertices.push_back(current);
      } else if (previous_inside) {
        TVaryings intersection = compute_intersection(previous, current, plane);
        output_vertices.push_back(intersection);
      }
    }

    return output_vertices;
  }

 public:
  std::vector<TVaryings> clip(
      const TVaryings& v0, const TVaryings& v1, const TVaryings& v2,
      const std::array<math::Vec4f, 6>& planes) const override {
    std::vector<TVaryings> vertices = {v0, v1, v2};

    for (const auto& plane : planes) {
      vertices = clip_against_plane(vertices, plane);

      if (vertices.size() < 3) {
        vertices.clear();
        break;
      }
    }

    return vertices;
  }
};

template <Varying TVaryings>
std::unique_ptr<ILineClipper<TVaryings>> create_line_clipper(
    LineClipAlgorithm type) {
  switch (type) {
    case LineClipAlgorithm::CohenSutherland:
      return std::make_unique<CohenSutherlandLineClipper<TVaryings>>();
    case LineClipAlgorithm::LiangBarsky:
      return std::make_unique<LiangBarskyLineClipper<TVaryings>>();
  }
  return nullptr;
}

template <Varying TVaryings>
std::unique_ptr<ITriangleClipper<TVaryings>> create_triangle_clipper(
    TriangleClipAlgorithm type) {
  switch (type) {
    case TriangleClipAlgorithm::SutherlandHodgman:
      return std::make_unique<SutherlandHodgmanTriangleClipper<TVaryings>>();
  }
  return nullptr;
}

}  // namespace renderer
}  // namespace soft_renderer