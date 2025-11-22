#ifndef BIDIRECTIONAL_PATH_TRACER_H
#define BIDIRECTIONAL_PATH_TRACER_H

#include "camera.h"
#include "objects/hittable.h"
#include "core/color.h"
#include "objects/primatives/triangle.h"
#include <vector>

// Define PathVertex before the class so it can be used in std::vector members
struct PathVertex {
    hit_record rec;
    color beta;      // Throughput
    vec3 wi;         // Incident direction (pointing away from surface)
    double pdf_fwd;  // Probability of generating this vertex from the previous one
    double pdf_rev;  // Probability of generating the previous vertex from this one
    bool is_delta;   // Specular/Dirac material
    bool is_light;   // Is this vertex on a light source?
};

class BidirectionalPathTracer {
public:
    BidirectionalPathTracer(
        camera& cam,
        const hittable& world,
        const triangle_collection& lights
    );

    void render();

private:
    camera& cam;
    const hittable& world;
    const triangle_collection& lights;

    // Renders a single pixel
    color render_pixel(int i, int j) const;

    // Generates a subpath starting from the camera
    void generate_camera_path(ray r, std::vector<PathVertex>& path) const;

    // Generates a subpath starting from a light source
    void generate_light_path(std::vector<PathVertex>& path) const;

    // Connects a camera vertex (t) and a light vertex (s)
    color evaluate_connection(
        const std::vector<PathVertex>& cam_path, 
        const std::vector<PathVertex>& light_path, 
        int s, 
        int t
    ) const;

    // Calculates Multiple Importance Sampling weight
    double mis_weight(
        const std::vector<PathVertex>& cam_path, 
        const std::vector<PathVertex>& light_path, 
        int s, 
        int t
    ) const;

    // Helper to sample cosine hemisphere
    vec3 sample_cosine_hemisphere(const vec3& normal) const;
};

#endif