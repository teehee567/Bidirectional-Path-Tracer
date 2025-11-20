#ifndef BIDIRECTIONAL_PATH_TRACER_H
#define BIDIRECTIONAL_PATH_TRACER_H

#include "camera.h"
#include "objects/hittable.h"
#include "objects/primatives/triangle.h"

#include <vector>

class BidirectionalPathTracer {
  public:
    BidirectionalPathTracer(camera& cam, const hittable& world, const triangle_collection& lights);

    void render();

  private:
    struct PathVertex {
        hit_record rec;
        color throughput;
        vec3 wi;
        color emission;
        bool delta = false;
        bool is_light = false;
        double pdf_fwd = 0.0; // pdf of sampling next direction (solid angle)
        double pdf_rev = 0.0; // pdf of sampling previous direction (solid angle)
    };

    color render_pixel(int i, int j) const;
    color trace_ray(const ray& r) const;
    color path_trace_color(const ray& r, int depth) const;
    color bidirectional_color(const ray& r, int depth) const;

    void trace_subpath(
        ray current,
        int depth,
        std::vector<PathVertex>& path,
        color throughput,
        color* background_contrib
    ) const;
    bool build_light_path(int depth, std::vector<PathVertex>& path) const;
    color connect_subpaths(
        const std::vector<PathVertex>& camera_path,
        const std::vector<PathVertex>& light_path,
        int s,
        int t
    ) const;
    color connect_vertices(const PathVertex& cam_v, const PathVertex& light_v) const;
    bool visible(const point3& a, const point3& b) const;
    vec3 sample_cosine_hemisphere(const vec3& normal) const;

    camera& cam;
    const hittable& world;
    const triangle_collection& lights;
};

#endif

