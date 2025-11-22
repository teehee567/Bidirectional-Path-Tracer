#include "renderers/unidirectional_path_tracer.h"

#include "acceleration/pdf.h"
#include "core/stats.h"
#include "image/wpng.h"
#include "main.h"
#include "materials/material.h"
#include "objects/primatives/triangle.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace {
    using clock = std::chrono::steady_clock;
}

UnidirectionalPathTracer::UnidirectionalPathTracer(
    camera& cam,
    const hittable& world,
    const triangle_collection& lights
) : cam(cam), world(world), lights(lights) {}

void UnidirectionalPathTracer::render() {
    cam.initialize();

    auto t0 = clock::now();
    reset_bvh_stats();

    const int width = cam.image_width;
    const int height = cam.image_height_px();
    const int total_rows = height;

    std::vector<color> framebuffer(static_cast<size_t>(width) * static_cast<size_t>(height), color(0,0,0));

    std::atomic<int> next_row{0};
    std::atomic<int> rows_done{0};

    int thread_count = static_cast<int>(std::thread::hardware_concurrency());
    if (thread_count <= 0) thread_count = 4;

    std::atomic<bool> done_flag{false};
    const int bar_width = cam.progress_bar_length;

    std::thread reporter([&]{
        while (!done_flag.load(std::memory_order_relaxed)) {
            int done = rows_done.load(std::memory_order_relaxed);
            double progress = (total_rows > 0) ? double(done) / total_rows : 1.0;
            int filled = int(progress * bar_width + 0.5);

            double dt = std::chrono::duration<double>(clock::now() - t0).count();
            double rps = dt > 0.0 ? done / dt : 0.0;
            double sec_left = (rps > 0.0) ? (total_rows - done) / rps : 0.0;

            int rem  = int(std::ceil(sec_left));
            int hrs  = rem / 3600;
            int mins = (rem % 3600) / 60;
            int secs = rem % 60;

            std::clog << '\r' << '['
                      << std::string(filled, '#')
                      << std::string(bar_width - filled, ' ')
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

        std::clog << "\r[" << std::string(bar_width, '#') << "] 100% | "
                  << total_rows << "/" << total_rows
                  << " | Runtime: " << (hrs > 0 ? (std::to_string(hrs) + ":") : "")
                  << std::setfill('0') << std::setw(2) << mins << ':'
                  << std::setw(2) << secs << "\n";
    });

    auto worker = [&]{
        for (;;) {
            int j = next_row.fetch_add(1, std::memory_order_relaxed);
            if (j >= height) break;

            for (int i = 0; i < width; ++i) {
                const size_t idx = static_cast<size_t>(j) * static_cast<size_t>(width)
                    + static_cast<size_t>(i);
                framebuffer[idx] = render_pixel(i, j);
            }

            rows_done.fetch_add(1, std::memory_order_relaxed);
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(static_cast<size_t>(thread_count));
    for (int t = 0; t < thread_count; ++t) pool.emplace_back(worker);
    for (auto& th : pool) th.join();

    done_flag.store(true, std::memory_order_relaxed);
    reporter.join();

    std::vector<uint8_t> rgb;
    const int used_samples_per_pixel = cam.actual_samples_per_pixel();
    colors_to_rgb8(framebuffer, width, height, used_samples_per_pixel, rgb);
    write_png(cam.file_name.c_str(), rgb, width, height);

    print_bvh_stats(std::clog);
}

color UnidirectionalPathTracer::render_pixel(int i, int j) const {
    color pixel_color(0,0,0);
    const int sqrt_spp = cam.sqrt_samples_per_pixel();

    for (int s_j = 0; s_j < sqrt_spp; ++s_j) {
        for (int s_i = 0; s_i < sqrt_spp; ++s_i) {
            ray r = cam.get_ray(i, j, s_i, s_j);
            pixel_color += trace_ray(r);
        }
    }

    return pixel_color;
}

color UnidirectionalPathTracer::trace_ray(const ray& r) const {
    return path_trace(r, cam.max_depth);
}

color UnidirectionalPathTracer::path_trace(const ray& r, int depth) const {
    bvh_stats().rays_traced.fetch_add(1, std::memory_order_relaxed);
    
    // Base case: maximum depth reached
    if (depth <= 0)
        return color(0,0,0);

    // Intersect ray with scene
    hit_record rec;
    if (!world.hit(r, interval(0.001, infinity), rec))
        return cam.background;

    // Get emission from hit point
    scatter_record srec;
    color emission = rec.mat->emitted(r, rec, rec.u, rec.v, rec.p);

    // Check if material scatters
    if (!rec.mat->scatter(r, rec, srec))
        return emission;

    // Handle delta materials (mirrors, glass) - no PDF needed
    if (srec.skip_pdf) {
        return emission + srec.attenuation * path_trace(srec.skip_pdf_ray, depth - 1);
    }

    // Create mixture PDF: 50% BSDF sampling, 50% light sampling (if lights exist)
    shared_ptr<pdf> light_pdf;
    shared_ptr<pdf> mixture;
    
    if (!lights.empty()) {
        light_pdf = make_shared<hittable_pdf>(lights, rec.p);
        mixture = make_shared<mixture_pdf>(light_pdf, srec.pdf_ptr);
    } else {
        mixture = srec.pdf_ptr;
    }

    // Sample direction using mixture PDF
    vec3 scattered_dir = mixture->generate();
    vec3 scattered_dir_unit = unit_vector(scattered_dir);
    double pdf_value = mixture->value(scattered_dir_unit);
    
    if (pdf_value <= 0.0)
        return emission;

    // Offset ray origin along scattered direction to avoid self-intersection
    point3 offset_origin = rec.p + (0.001 * scattered_dir_unit);
    ray scattered(offset_origin, scattered_dir_unit, r.time());

    // Recursively trace scattered ray
    color sample_color = path_trace(scattered, depth - 1);

    // Evaluate BSDF
    color f = rec.mat->evaluate_bsdf(
        rec,
        unit_vector(-r.direction()),  // wi: incoming direction (toward previous vertex)
        scattered_dir_unit  // wo: outgoing direction (toward next vertex)
    );

    // Compute cosine term - use absolute value to match evaluate_bsdf which divides by fabs
    // This is critical for transmission where the direction can be opposite the normal
    double cos_theta = std::fabs(dot(rec.normal, scattered_dir_unit));
    
    // Monte Carlo estimate: (f * cos_theta * L) / pdf
    // If BSDF is zero or cosine is zero, this will naturally contribute zero
    color scatter_contrib = (f * cos_theta * sample_color) / pdf_value;

    return emission + scatter_contrib;
}

