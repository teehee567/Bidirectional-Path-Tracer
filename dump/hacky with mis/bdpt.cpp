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
        double cos_theta = std::fabs(dot(rec.normal, unit_vector(scattered.direction())));
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
    color throughput = emitter_vertex.throughput * (cos_theta / pdf_dir);
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

    return connect_vertices(camera_path, light_path, s, t);
}

color BidirectionalPathTracer::connect_vertices(
    const std::vector<PathVertex>& camera_path,
    const std::vector<PathVertex>& light_path,
    int s,
    int t
) const {
    const PathVertex& cam_v = camera_path[s - 1];
    const PathVertex& light_v = light_path[t - 1];

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

    // Compute MIS weight for strategy (s,t)
    double mis_weight = compute_mis_weight(camera_path, light_path, s, t);
    
    color contrib = cam_v.throughput * f_cam;
    contrib = contrib * light_v.throughput * f_light;
    contrib = contrib * (cos_cam * cos_light) / dist2;
    contrib = contrib * mis_weight;
    return contrib;
}

double BidirectionalPathTracer::compute_mis_weight(
    const std::vector<PathVertex>& camera_path,
    const std::vector<PathVertex>& light_path,
    int s,
    int t
) const {
    if (s <= 0 || t <= 0 || s > static_cast<int>(camera_path.size()) || t > static_cast<int>(light_path.size()))
        return 0.0;

    const PathVertex& cam_v = camera_path[s - 1];
    const PathVertex& light_v = light_path[t - 1];

    if (cam_v.delta || light_v.delta)
        return 0.0;

    vec3 dir = light_v.rec.p - cam_v.rec.p;
    double dist2 = dir.length_squared();
    if (dist2 <= 0)
        return 0.0;

    vec3 dir_unit = unit_vector(dir);
    
    // Compute the PDF of the connection edge from both perspectives
    double pdf_connect_cam = compute_connection_pdf(cam_v, light_v, dir_unit, dist2);
    double pdf_connect_light = compute_connection_pdf(light_v, cam_v, -dir_unit, dist2);
    
    if (pdf_connect_cam <= 0 || pdf_connect_light <= 0)
        return 0.0;

    // Compute numerator: PDF for strategy (s,t)
    double pdf_num = 1.0;
    
    // Camera path PDFs
    for (int i = 0; i < s - 1; ++i) {
        if (i < static_cast<int>(camera_path.size()) && camera_path[i].pdf_fwd > 0) {
            pdf_num *= camera_path[i].pdf_fwd;
        } else {
            pdf_num = 0;
            break;
        }
    }
    pdf_num *= pdf_connect_cam;
    
    // Light path PDFs
    for (int j = 0; j < t - 1; ++j) {
        if (j < static_cast<int>(light_path.size()) && light_path[j].pdf_fwd > 0) {
            pdf_num *= light_path[j].pdf_fwd;
        } else {
            pdf_num = 0;
            break;
        }
    }
    pdf_num *= pdf_connect_light;

    if (pdf_num <= 0)
        return 0.0;

    // Compute denominator: sum of PDFs for all strategies
    double sum_pdf = pdf_num; // Start with strategy (s,t)
    
    // Consider strategies (i,t) for i from 0 to s-1
    for (int i = 0; i < s; ++i) {
        double pdf_i_t = compute_connection_pdf_strategy(camera_path, light_path, i, t);
        if (pdf_i_t > 0) {
            sum_pdf += pdf_i_t;
        }
    }
    
    // Consider strategies (s,j) for j from 0 to t-1
    for (int j = 0; j < t; ++j) {
        double pdf_s_j = compute_connection_pdf_strategy(camera_path, light_path, s, j);
        if (pdf_s_j > 0) {
            sum_pdf += pdf_s_j;
        }
    }

    if (sum_pdf <= 0)
        return 0.0;

    // Balance heuristic: w = pdf_s_t / sum_pdf
    return pdf_num / sum_pdf;
}

double BidirectionalPathTracer::compute_connection_pdf_strategy(
    const std::vector<PathVertex>& camera_path,
    const std::vector<PathVertex>& light_path,
    int s,
    int t
) const {
    // Strategy (0,t): connecting camera ray directly to light vertex t - not handled
    if (s == 0) {
        return 0.0; // Would need camera ray direction
    }

    // Strategy (s,0): connecting camera vertex s directly to light - handled by direct emission
    if (t == 0) {
        return 0.0; // Handled separately
    }

    if (s > static_cast<int>(camera_path.size()) || t > static_cast<int>(light_path.size()))
        return 0.0;

    const PathVertex& cam_v = camera_path[s - 1];
    const PathVertex& light_v = light_path[t - 1];

    if (cam_v.delta || light_v.delta)
        return 0.0;

    vec3 dir = light_v.rec.p - cam_v.rec.p;
    double dist2 = dir.length_squared();
    if (dist2 <= 0)
        return 0.0;

    vec3 dir_unit = unit_vector(dir);
    double cos_cam = std::fabs(dot(cam_v.rec.normal, dir_unit));
    double cos_light = std::fabs(dot(light_v.rec.normal, -dir_unit));
    if (cos_cam <= 0 || cos_light <= 0)
        return 0.0;

    // Compute PDF for this connection strategy
    double pdf_cam = compute_connection_pdf(cam_v, light_v, dir_unit, dist2);
    double pdf_light = compute_connection_pdf(light_v, cam_v, -dir_unit, dist2);
    
    if (pdf_cam <= 0 || pdf_light <= 0)
        return 0.0;

    // Product of PDFs along the path
    double strategy_pdf = 1.0;
    
    // Camera path: multiply PDFs from camera to vertex s
    for (int i = 0; i < s - 1; ++i) {
        if (i < static_cast<int>(camera_path.size()) && camera_path[i].pdf_fwd > 0) {
            strategy_pdf *= camera_path[i].pdf_fwd;
        } else {
            return 0.0;
        }
    }
    strategy_pdf *= pdf_cam;
    
    // Light path: multiply PDFs from light to vertex t
    for (int j = 0; j < t - 1; ++j) {
        if (j < static_cast<int>(light_path.size()) && light_path[j].pdf_fwd > 0) {
            strategy_pdf *= light_path[j].pdf_fwd;
        } else {
            return 0.0;
        }
    }
    strategy_pdf *= pdf_light;

    return strategy_pdf;
}

double BidirectionalPathTracer::compute_connection_pdf(
    const PathVertex& from_v,
    const PathVertex& to_v,
    const vec3& dir_unit,
    double dist2
) const {
    // Compute PDF of sampling direction dir_unit from from_v to to_v
    // This should match what the unidirectional path tracer would use
    
    // First, compute BSDF PDF (solid angle)
    ray dummy_ray(from_v.rec.p, dir_unit, 0.0);
    ray prev_ray(from_v.rec.p, -from_v.wi, 0.0);
    double pdf_bsdf = from_v.rec.mat->scattering_pdf(dummy_ray, from_v.rec, prev_ray);
    
    // If this is a light vertex emitting toward a non-light vertex, handle specially
    if (from_v.is_light && !to_v.is_light) {
        // Light emission PDF: convert area PDF to solid angle
        double pdf_area = lights.pdf_value(from_v.rec.p, dir_unit);
        if (pdf_area <= 0)
            return 0.0;
        double cos_theta = std::fabs(dot(from_v.rec.normal, dir_unit));
        if (cos_theta <= 0)
            return 0.0;
        // Convert from area PDF to solid angle PDF: pdf_solid = pdf_area * dist^2 / cos_theta
        return pdf_area * dist2 / cos_theta;
    }
    
    // For regular vertices, use mixture PDF matching unidirectional tracer
    if (!lights.empty()) {
        // Compute light PDF (area PDF converted to solid angle)
        double pdf_area = lights.pdf_value(from_v.rec.p, dir_unit);
        double pdf_light = 0.0;
        if (pdf_area > 0) {
            double cos_theta = std::fabs(dot(from_v.rec.normal, dir_unit));
            if (cos_theta > 0) {
                // Convert from area PDF to solid angle PDF
                pdf_light = pdf_area * dist2 / cos_theta;
            }
        }
        
        // Use 50/50 mixture like unidirectional path tracer
        // Note: hittable_pdf already returns area PDF, but mixture_pdf.value() expects
        // solid angle PDFs, so we need to convert manually here
        if (pdf_light > 0) {
            return 0.5 * pdf_bsdf + 0.5 * pdf_light;
        } else {
            return pdf_bsdf;
        }
    }
    
    // No lights, just use BSDF PDF
    return pdf_bsdf;
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

