#include "renderers/bidirectional_path_tracer.h"

#include "acceleration/pdf.h"
#include "core/stats.h"
#include "image/wpng.h"
#include "materials/material.h"
#include "objects/primatives/triangle.h"
#include "onb.h"

#include <algorithm>
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

BidirectionalPathTracer::BidirectionalPathTracer(
    camera& cam,
    const hittable& world,
    const triangle_collection& lights
) : cam(cam), world(world), lights(lights) {}

void BidirectionalPathTracer::render() {
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

color BidirectionalPathTracer::render_pixel(int i, int j) const {
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

color BidirectionalPathTracer::trace_ray(const ray& r) const {
    if (lights.empty())
        return path_trace_color(r, cam.max_depth);
    return bidirectional_color(r, cam.max_depth);
}

color BidirectionalPathTracer::path_trace_color(const ray& r, int depth) const {
    bvh_stats().rays_traced.fetch_add(1, std::memory_order_relaxed);
    if (depth <= 0)
        return color(0,0,0);

    hit_record rec;
    if (!world.hit(r, interval(0.001, infinity), rec))
        return cam.background;

    scatter_record srec;
    color emission = rec.mat->emitted(r, rec, rec.u, rec.v, rec.p);

    if (!rec.mat->scatter(r, rec, srec))
        return emission;

    if (srec.skip_pdf) {
        return srec.attenuation * path_trace_color(srec.skip_pdf_ray, depth - 1);
    }

    auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
    mixture_pdf p(light_ptr, srec.pdf_ptr);

    ray scattered(rec.p, p.generate(), r.time());
    auto pdf_value = p.value(scattered.direction());
    if (pdf_value <= 0)
        return emission;

    color sample_color = path_trace_color(scattered, depth - 1);
    color f = rec.mat->evaluate_bsdf(rec, unit_vector(-r.direction()), unit_vector(scattered.direction()));
    double cos_theta = std::max(0.0, dot(rec.normal, unit_vector(scattered.direction())));

    color scatter = (f * cos_theta * sample_color) / pdf_value;
    return emission + scatter;
}

color BidirectionalPathTracer::bidirectional_color(const ray& r, int depth) const {
    std::vector<PathVertex> camera_path;
    camera_path.reserve(depth);
    color background_contrib(0,0,0);
    camera_path.clear();
    trace_subpath(r, depth, camera_path, color(1,1,1), &background_contrib);

    color result = background_contrib;
    for (const auto& vertex : camera_path) {
        if (!vertex.delta && vertex.emission.length_squared() > 0)
            result += vertex.throughput * vertex.emission;
    }

    std::vector<PathVertex> light_path;
    light_path.reserve(depth);
    if (!build_light_path(depth, light_path))
        return result;

    for (int s = 1; s <= static_cast<int>(camera_path.size()); ++s) {
        for (int t = 1; t <= static_cast<int>(light_path.size()); ++t) {
            result += connect_subpaths(camera_path, light_path, s, t);
        }
    }

    return result;
}

void BidirectionalPathTracer::trace_subpath(
    ray current,
    int depth,
    std::vector<PathVertex>& path,
    color throughput,
    color* background_contrib
) const {
    for (int bounce = 0; bounce < depth; ++bounce) {
        bvh_stats().rays_traced.fetch_add(1, std::memory_order_relaxed);

        hit_record rec;
        if (!world.hit(current, interval(0.001, infinity), rec)) {
            if (background_contrib)
                *background_contrib += throughput * cam.background;
            break;
        }

        PathVertex vertex;
        vertex.rec = rec;
        vertex.throughput = throughput;
        vertex.wi = unit_vector(-current.direction());
        vertex.emission = rec.mat->emitted(current, rec, rec.u, rec.v, rec.p);
        vertex.delta = rec.mat->is_delta();
        vertex.is_light = rec.mat->is_emissive();
        path.push_back(vertex);

        scatter_record srec;
        if (!rec.mat->scatter(current, rec, srec))
            break;

        if (srec.skip_pdf) {
            throughput = throughput * srec.attenuation;
            current = srec.skip_pdf_ray;
            continue;
        }

        ray scattered(rec.p, srec.pdf_ptr->generate(), current.time());
        auto pdf_value = srec.pdf_ptr->value(scattered.direction());
        if (pdf_value <= 0)
            break;

        color f = rec.mat->evaluate_bsdf(rec, unit_vector(-current.direction()),
                                         unit_vector(scattered.direction()));
        double cos_theta = std::max(0.0, dot(rec.normal, unit_vector(scattered.direction())));
        throughput = throughput * (f * (cos_theta / pdf_value));
        path.back().pdf_fwd = pdf_value;
        path.back().pdf_rev = rec.mat->scattering_pdf(
            scattered, rec, ray(rec.p, -current.direction(), current.time())
        );
        current = scattered;
    }
}

bool BidirectionalPathTracer::build_light_path(int depth, std::vector<PathVertex>& path) const {
    path.clear();
    if (depth <= 0 || lights.empty())
        return false;

    surface_sample sample;
    if (!lights.sample_surface(sample))
        return false;

    hit_record light_rec;
    light_rec.p = sample.position;
    light_rec.normal = sample.normal;
    light_rec.mat = sample.mat;
    light_rec.front_face = true;
    light_rec.u = 0;
    light_rec.v = 0;

    ray light_ray_origin(sample.position, sample.normal, 0.0);
    color emission = sample.mat->emitted(light_ray_origin, light_rec, 0, 0, sample.position);
    if (emission.length_squared() <= 0)
        return false;

    PathVertex emitter_vertex;
    emitter_vertex.rec = light_rec;
    emitter_vertex.throughput = color(1.0, 1.0, 1.0) / std::max(sample.pdf, 1e-8);
    emitter_vertex.wi = light_rec.normal;
    emitter_vertex.emission = emission;
    emitter_vertex.delta = false;
    emitter_vertex.is_light = true;
    emitter_vertex.pdf_fwd = 0.0;
    emitter_vertex.pdf_rev = 0.0;
    path.push_back(emitter_vertex);

    vec3 dir = sample_cosine_hemisphere(light_rec.normal);
    vec3 dir_unit = unit_vector(dir);
    double cos_theta = std::max(0.0, dot(light_rec.normal, dir_unit));
    if (cos_theta <= 0)
        return true;

    double pdf_dir = std::max(cos_theta / pi, 1e-8);
    path.back().pdf_fwd = pdf_dir;
    color throughput = emitter_vertex.throughput * emission * (cos_theta / pdf_dir);
    ray light_ray(light_rec.p + (0.001 * light_rec.normal), dir_unit, 0.0);
    trace_subpath(light_ray, depth - 1, path, throughput, nullptr);
    return true;
}

color BidirectionalPathTracer::connect_subpaths(
    const std::vector<PathVertex>& camera_path,
    const std::vector<PathVertex>& light_path,
    int s,
    int t
) const {
    if (s <= 0 || t <= 0)
        return color(0,0,0);
    if (s > static_cast<int>(camera_path.size()) || t > static_cast<int>(light_path.size()))
        return color(0,0,0);

    const PathVertex& cam_v = camera_path[s - 1];
    const PathVertex& light_v = light_path[t - 1];
    return connect_vertices(cam_v, light_v);
}

color BidirectionalPathTracer::connect_vertices(const PathVertex& cam_v, const PathVertex& light_v) const {
    if (cam_v.delta || light_v.delta)
        return color(0,0,0);

    vec3 dir = light_v.rec.p - cam_v.rec.p;
    double dist2 = dir.length_squared();
    if (dist2 <= 0)
        return color(0,0,0);

    vec3 dir_unit = unit_vector(dir);
    double cos_cam = std::fabs(dot(cam_v.rec.normal, dir_unit));
    double cos_light = std::fabs(dot(light_v.rec.normal, -dir_unit));
    if (cos_cam <= 0 || cos_light <= 0)
        return color(0,0,0);

    if (!visible(cam_v.rec.p, light_v.rec.p))
        return color(0,0,0);

    color f_cam = cam_v.rec.mat->evaluate_bsdf(cam_v.rec, cam_v.wi, dir_unit);
    if (f_cam.length_squared() <= 0)
        return color(0,0,0);

    color f_light = light_v.is_light
        ? light_v.emission
        : light_v.rec.mat->evaluate_bsdf(light_v.rec, light_v.wi, -dir_unit);
    if (f_light.length_squared() <= 0)
        return color(0,0,0);

    color contrib = cam_v.throughput * f_cam;
    contrib = contrib * light_v.throughput * f_light;
    contrib = contrib * (cos_cam * cos_light) / dist2;
    return contrib;
}

bool BidirectionalPathTracer::visible(const point3& a, const point3& b) const {
    vec3 dir = b - a;
    double dist = dir.length();
    if (dist <= 0)
        return false;

    vec3 dir_unit = dir / dist;
    ray shadow_ray(a + (0.001 * dir_unit), dir_unit, 0.0);
    hit_record tmp;
    double max_t = dist - 0.001;
    if (max_t <= 0)
        return false;
    return !world.hit(shadow_ray, interval(0.001, max_t), tmp);
}

vec3 BidirectionalPathTracer::sample_cosine_hemisphere(const vec3& normal) const {
    onb basis(normal);
    return basis.transform(random_cosine_direction());
}

