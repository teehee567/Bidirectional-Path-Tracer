#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include "pdf.h"
#include "main.h"
#include "material.h"
#include "wpng.h"

#include <chrono>
#include <string>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <cmath>
#include <string>

class camera {
  public:
    double aspect_ratio = 1.0;  // Ratio of image width over height
    int    image_width  = 100;  // Rendered image width in pixel count
    int    samples_per_pixel = 10;   // Count of random samples for each pixel
    int    max_depth         = 10;   // Maximum number of ray bounces into scene
    color  background;               // Scene background color

    double vfov = 90;  // Vertical view angle (field of view)
    point3 lookfrom = point3(0,0,0);   // Point camera is looking from
    point3 lookat   = point3(0,0,-1);  // Point camera is looking at
    vec3   vup      = vec3(0,1,0);     // Camera-relative "up" direction

    double defocus_angle = 0;  // Variation angle of rays through each pixel
    double focus_dist = 10;    // Distance from camera lookfrom point to plane of perfect focus
    
    int progress_bar_length = 30; // Amount of characters in the progress bar
    std::string file_name = "image.png";

    void render(const hittable& world, const hittable& lights) {
        initialize();

        using clock = std::chrono::steady_clock;
        auto t0 = clock::now();

        const int W = image_width;
        const int H = image_height;
        const int total_rows = H;

        std::vector<color> framebuffer(W * H, color(0,0,0));

        std::atomic<int> next_row{0};
        std::atomic<int> rows_done{0};

        int thread_count = int(std::thread::hardware_concurrency());
        if (thread_count <= 0) thread_count = 4;

        std::atomic<bool> done_flag{false};
        const int barWidth = progress_bar_length;

        // Progress Bar
        std::thread reporter([&]{
            while (!done_flag.load(std::memory_order_relaxed)) {
                int done = rows_done.load(std::memory_order_relaxed);
                double progress = (total_rows > 0) ? double(done) / total_rows : 1.0;
                int filled = int(progress * barWidth + 0.5);

                double dt = std::chrono::duration<double>(clock::now() - t0).count();
                double rps = dt > 0.0 ? done / dt : 0.0;
                double sec_left = (rps > 0.0) ? (total_rows - done) / rps : 0.0;

                int rem  = int(std::ceil(sec_left));
                int hrs  = rem / 3600;
                int mins = (rem % 3600) / 60;
                int secs = rem % 60;

                std::clog << '\r' << '['
                          << std::string(filled, '#')
                          << std::string(barWidth - filled, ' ')
                          << "] " << std::setw(3) << int(progress * 100) << "% | "
                          << done << "/" << total_rows
                          << " | ETA: "
                          << (hrs > 0 ? (std::to_string(hrs) + ":") : "")
                          << std::setfill('0') << std::setw(2) << mins << ':'
                          << std::setw(2) << secs
                          << std::setfill(' ')
                          << std::flush;

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            double dt = std::chrono::duration<double>(clock::now() - t0).count();
            int rem  = int(std::ceil(dt));
                int hrs  = rem / 3600;
                int mins = (rem % 3600) / 60;
                int secs = rem % 60;

            // Final line
            std::clog << "\r[" << std::string(barWidth, '#') << "] 100% | "
                      << total_rows << "/" << total_rows
                      << " | Runtime: "<< (hrs > 0 ? (std::to_string(hrs) + ":") : "")
                          << std::setfill('0') << std::setw(2) << mins << ':'
                          << std::setw(2) << secs << "\n";
        });

        auto worker = [&]{
            for (;;) {
                int j = next_row.fetch_add(1, std::memory_order_relaxed);
                if (j >= H) break;

                for (int i = 0; i < W; ++i) {
                    color pixel_color(0,0,0);
                    for (int s_j = 0; s_j < sqrt_spp; s_j++) {
                        for (int s_i = 0; s_i < sqrt_spp; s_i++) {
                            ray r = get_ray(i, j, s_i, s_j);
                            pixel_color += ray_color(r, max_depth, world, lights);
                        }
                    }
                    framebuffer[j * W + i] = pixel_color;
                }

                rows_done.fetch_add(1, std::memory_order_relaxed);
            }
        };

        std::vector<std::thread> pool;
        pool.reserve(thread_count);
        for (int t = 0; t < thread_count; ++t) pool.emplace_back(worker);
        for (auto& th : pool) th.join();

        done_flag.store(true, std::memory_order_relaxed);
        reporter.join();

        std::vector<uint8_t> rgb;
        colors_to_rgb8(framebuffer, image_width, image_height, samples_per_pixel, rgb);
        write_png(file_name.c_str(), rgb, image_width, image_height);
    }

  private:
    int    image_height;         // Rendered image height
    double pixel_samples_scale;  // Color scale factor for a sum of pixel samples
    int    sqrt_spp;             // Square root of number of samples per pixel
    double recip_sqrt_spp;       // 1 / sqrt_spp
    point3 center;               // Camera center
    point3 pixel00_loc;          // Location of pixel 0, 0
    vec3   pixel_delta_u;        // Offset to pixel to the right
    vec3   pixel_delta_v;        // Offset to pixel below
    vec3   u, v, w;              // Camera frame basis vectors
    vec3   defocus_disk_u;       // Defocus disk horizontal radius
    vec3   defocus_disk_v;       // Defocus disk vertical radius

    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        sqrt_spp = int(std::sqrt(samples_per_pixel));
        pixel_samples_scale = 1.0 / (sqrt_spp * sqrt_spp);
        recip_sqrt_spp = 1.0 / sqrt_spp;

        center = lookfrom;

        // Determine viewport dimensions.
        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta/2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (double(image_width)/image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        vec3 viewport_u = viewport_width * u;    // Vector across viewport horizontal edge
        vec3 viewport_v = viewport_height * -v;  // Vector down viewport vertical edge

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
        auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j, int s_i, int s_j) const {
        // Construct a camera ray originating from the defocus disk and directed at a randomly
        // sampled point around the pixel location i, j for stratified sample square s_i, s_j.

        auto offset = sample_square_stratified(s_i, s_j);
        auto pixel_sample = pixel00_loc
                          + ((i + offset.x()) * pixel_delta_u)
                          + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_double();

        return ray(ray_origin, ray_direction, ray_time);
    }

    vec3 sample_square_stratified(int s_i, int s_j) const {
        // Returns the vector to a random point in the square sub-pixel specified by grid
        // indices s_i and s_j, for an idealized unit square pixel [-.5,-.5] to [+.5,+.5].

        auto px = ((s_i + random_double()) * recip_sqrt_spp) - 0.5;
        auto py = ((s_j + random_double()) * recip_sqrt_spp) - 0.5;

        return vec3(px, py, 0);
    }

    vec3 sample_square() const {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }

    point3 defocus_disk_sample() const {
        // Returns a random point in the camera defocus disk.
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, int depth, const hittable& world, const hittable& lights)
    const {
        // If we've exceeded the ray bounce limit, no more light is gathered.
        if (depth <= 0)
            return color(0,0,0);

        hit_record rec;

        // If the ray hits nothing, return the background color.
        if (!world.hit(r, interval(0.001, infinity), rec))
            return background;

        scatter_record srec;
        color color_from_emission = rec.mat->emitted(r, rec, rec.u, rec.v, rec.p);

        if (!rec.mat->scatter(r, rec, srec))
            return color_from_emission;

        if (srec.skip_pdf) {
            return srec.attenuation * ray_color(srec.skip_pdf_ray, depth-1, world, lights);
        }
        
        auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
        mixture_pdf p(light_ptr, srec.pdf_ptr);

        ray scattered = ray(rec.p, p.generate(), r.time());
        auto pdf_value = p.value(scattered.direction());

        double scattering_pdf = rec.mat->scattering_pdf(r, rec, scattered);

        color sample_color = ray_color(scattered, depth-1, world, lights);
        color color_from_scatter =
            (srec.attenuation * scattering_pdf * sample_color) / pdf_value;

        return color_from_emission + color_from_scatter;
    }
};

#endif