#ifndef CAMERA_H
#define CAMERA_H

#include "main.h"

#include <algorithm>
#include <string>

class camera {
  public:
    double aspect_ratio = 1.0;   // Ratio of image width over height
    int    image_width  = 100;   // Rendered image width in pixel count
    int    samples_per_pixel = 4;   // Count of random samples for each pixel
    int    max_depth         = 10;  // Maximum number of ray bounces into scene
    color  background = color(0,0,0);

    double vfov = 90;  // Vertical view angle (field of view)
    point3 lookfrom = point3(0,0,0);   // Point camera is looking from
    point3 lookat   = point3(0,0,-1);  // Point camera is looking at
    vec3   vup      = vec3(0,1,0);     // Camera-relative "up" direction

    double defocus_angle = 0;  // Variation angle of rays through each pixel
    double focus_dist = 10;    // Distance from camera lookfrom point to plane of perfect focus

    int progress_bar_length = 30;
    std::string file_name = "image.png";

    camera() = default;

    void initialize();
    ray get_ray(int i, int j, int s_i, int s_j) const;

    int image_height_px() const { return image_height; }
    int sqrt_samples_per_pixel() const { return sqrt_spp; }
    int actual_samples_per_pixel() const { return std::max(1, sqrt_spp * sqrt_spp); }
    double recip_sqrt_samples() const { return recip_sqrt_spp; }

  private:
    int    image_height = 1;
    int    sqrt_spp = 1;
    double recip_sqrt_spp = 1.0;
    point3 center;
    point3 pixel00_loc;
    vec3   pixel_delta_u;
    vec3   pixel_delta_v;
    vec3   u, v, w;
    vec3   defocus_disk_u;
    vec3   defocus_disk_v;

    vec3 sample_square_stratified(int s_i, int s_j) const;
    vec3 sample_square() const;
    point3 defocus_disk_sample() const;
};

inline void camera::initialize() {
    image_height = int(image_width / aspect_ratio);
    image_height = (image_height < 1) ? 1 : image_height;

    sqrt_spp = std::max(1, int(std::sqrt(std::max(1, samples_per_pixel))));
    recip_sqrt_spp = 1.0 / sqrt_spp;

    center = lookfrom;

    const auto theta = degrees_to_radians(vfov);
    const auto h = std::tan(theta / 2);
    const auto viewport_height = 2 * h * focus_dist;
    const auto viewport_width = viewport_height * (double(image_width) / image_height);

    w = unit_vector(lookfrom - lookat);
    u = unit_vector(cross(vup, w));
    v = cross(w, u);

    const vec3 viewport_u = viewport_width * u;
    const vec3 viewport_v = viewport_height * -v;

    pixel_delta_u = viewport_u / image_width;
    pixel_delta_v = viewport_v / image_height;

    const auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
    pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

    const auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
    defocus_disk_u = u * defocus_radius;
    defocus_disk_v = v * defocus_radius;
}

inline ray camera::get_ray(int i, int j, int s_i, int s_j) const {
    const auto offset = sample_square_stratified(s_i, s_j);
    const auto pixel_sample = pixel00_loc
        + ((i + offset.x()) * pixel_delta_u)
        + ((j + offset.y()) * pixel_delta_v);

    const auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
    const auto ray_direction = pixel_sample - ray_origin;
    const auto ray_time = random_double();

    return ray(ray_origin, ray_direction, ray_time);
}

inline vec3 camera::sample_square_stratified(int s_i, int s_j) const {
    const auto px = ((s_i + random_double()) * recip_sqrt_spp) - 0.5;
    const auto py = ((s_j + random_double()) * recip_sqrt_spp) - 0.5;
    return vec3(px, py, 0);
}

inline vec3 camera::sample_square() const {
    return vec3(random_double() - 0.5, random_double() - 0.5, 0);
}

inline point3 camera::defocus_disk_sample() const {
    auto p = random_in_unit_disk();
    return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
}

#endif