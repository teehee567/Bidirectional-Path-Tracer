#include "renderers/bidirectional_path_tracer.h"

#include "acceleration/pdf.h"
#include "core/stats.h"
#include "image/wpng.h"
#include "materials/material.h"
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

    // Utility to convert solid angle PDF to area PDF
    inline double pdf_solid_to_area(double pdf_solid, double dist_sq, double cos_theta) {
        if (dist_sq <= 0 || cos_theta <= 1e-9) return 0.0;
        return pdf_solid * std::abs(cos_theta) / dist_sq;
    }

    // Geometry term G(p1 <-> p2)
    // PathVertex is now defined in the header, so it is visible here.
    double geometry_term(const PathVertex& v1, const PathVertex& v2) {
        vec3 d = v1.rec.p - v2.rec.p;
        double dist_sq = d.length_squared();
        if (dist_sq <= 0) return 0.0;
        
        double dist = std::sqrt(dist_sq);
        vec3 dir = d / dist;
        
        double cos1 = std::abs(dot(v1.rec.normal, -dir));
        double cos2 = std::abs(dot(v2.rec.normal, dir));
        
        return (cos1 * cos2) / dist_sq;
    }
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
    std::vector<color> framebuffer(static_cast<size_t>(width) * static_cast<size_t>(height), color(0,0,0));

    std::atomic<int> next_row{0};
    std::atomic<int> rows_done{0};
    int thread_count = static_cast<int>(std::thread::hardware_concurrency());
    if (thread_count <= 0) thread_count = 4;

    auto worker = [&]{
        for (;;) {
            int j = next_row.fetch_add(1, std::memory_order_relaxed);
            if (j >= height) break;

            for (int i = 0; i < width; ++i) {
                size_t idx = static_cast<size_t>(j) * width + i;
                framebuffer[idx] = render_pixel(i, j);
            }
            rows_done.fetch_add(1, std::memory_order_relaxed);
        }
    };


    const int total_rows = height;
    const int bar_width = cam.progress_bar_length;
    std::atomic<bool> done_flag{false};
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

    std::vector<std::thread> pool;
    pool.reserve(thread_count);
    for (int t = 0; t < thread_count; ++t) pool.emplace_back(worker);
    for (auto& th : pool) th.join();
    
    done_flag = true;
    reporter.join();

    std::vector<uint8_t> rgb;
    colors_to_rgb8(framebuffer, width, height, cam.actual_samples_per_pixel(), rgb);
    write_png(cam.file_name.c_str(), rgb, width, height);
    print_bvh_stats(std::clog);
}

color BidirectionalPathTracer::render_pixel(int i, int j) const {
    bvh_stats().rays_traced.fetch_add(1, std::memory_order_relaxed);

    color pixel_acc(0,0,0);
    int sqrt_spp = cam.sqrt_samples_per_pixel();

    for (int s_j = 0; s_j < sqrt_spp; ++s_j) {
        for (int s_i = 0; s_i < sqrt_spp; ++s_i) {
            ray cam_ray = cam.get_ray(i, j, s_i, s_j);
            
            std::vector<PathVertex> cam_path, light_path;
            cam_path.reserve(cam.max_depth + 1);
            light_path.reserve(cam.max_depth + 1);

            generate_camera_path(cam_ray, cam_path);
            generate_light_path(light_path);

            color sample_L(0,0,0);
            int n_cam = static_cast<int>(cam_path.size());
            int n_light = static_cast<int>(light_path.size());

            for (int t = 1; t <= n_cam; ++t) {
                for (int s = 0; s <= n_light; ++s) {
                    sample_L += evaluate_connection(cam_path, light_path, s, t);
                }
            }
            pixel_acc += sample_L;
        }
    }
    return pixel_acc;
}

void BidirectionalPathTracer::generate_camera_path(ray r, std::vector<PathVertex>& path) const {
    color beta(1,1,1);
    double pdf_fwd = 1.0; 

    for (int depth = 0; depth < cam.max_depth; ++depth) {
        hit_record rec;
        if (!world.hit(r, interval(0.001, infinity), rec)) {
            break;
        }

        PathVertex v;
        v.rec = rec;
        v.beta = beta;
        v.wi = -unit_vector(r.direction());
        v.is_delta = rec.mat->is_delta();
        v.is_light = rec.mat->is_emissive();
        v.pdf_fwd = pdf_fwd;
        v.pdf_rev = 0.0; // Initialized to 0, updated below

        path.push_back(v);

        scatter_record srec;
        if (!rec.mat->scatter(r, rec, srec)) {
            break;
        }

        if (srec.skip_pdf) {
            // Fix: Use explicit multiplication if *= is not overloaded for color * color
            beta = beta * srec.attenuation;
            r = srec.skip_pdf_ray;
            pdf_fwd = 0.0; 
            path.back().pdf_rev = 0.0;
        } else {
            auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
            mixture_pdf p(light_ptr, srec.pdf_ptr);

            vec3 scatter_dir = p.generate();
            double pdf_val = p.value(scatter_dir);

            if (pdf_val <= 0) break;

            color f = rec.mat->evaluate_bsdf(rec, v.wi, scatter_dir);
            double cos_theta = std::abs(dot(rec.normal, scatter_dir));
            
            // Fix: Explicit multiplication
            beta = beta * (f * cos_theta / pdf_val);
            
            // Use simplified ray constructor to avoid ambiguity
            // Assuming ray(origin, direction, time)
            ray scatter_ray(rec.p, scatter_dir, r.time());
            
            // IMPORTANT: Ensure scattering_pdf arguments match your material.h
            // Standard is usually (scattered, rec, incoming)
            ray incoming_ray(rec.p, -v.wi, r.time());
            path.back().pdf_rev = rec.mat->scattering_pdf(scatter_ray, rec, incoming_ray);

            r = ray(rec.p + (0.001*scatter_dir), scatter_dir, r.time());
            pdf_fwd = pdf_val;
        }
    }
}

void BidirectionalPathTracer::generate_light_path(std::vector<PathVertex>& path) const {
    if (lights.empty()) return;

    surface_sample light_sample;
    if (!lights.sample_surface(light_sample)) return;

    hit_record light_rec;
    light_rec.p = light_sample.position;
    light_rec.normal = light_sample.normal;
    light_rec.mat = light_sample.mat;
    light_rec.front_face = true; // Assume front face for emission
    
    PathVertex v0;
    v0.rec = light_rec;
    v0.is_light = true;
    v0.is_delta = false;
    
    ray dummy_in(light_sample.position, light_sample.normal, 0.0);
    color Le = light_sample.mat->emitted(dummy_in, light_rec, 0, 0, light_sample.position);
    
    double light_pdf_area = light_sample.pdf; 
    if (light_pdf_area <= 1e-8) return;

    v0.beta = Le / light_pdf_area;
    v0.pdf_fwd = light_pdf_area; 
    v0.pdf_rev = 0.0; 
    
    path.push_back(v0);

    vec3 dir = sample_cosine_hemisphere(light_rec.normal);
    double cos_theta = dot(light_rec.normal, dir);
    if (cos_theta <= 0) return;

    double pdf_dir = cos_theta / pi; 
    
    color beta = v0.beta * (cos_theta / pdf_dir); 
    
    double pdf_fwd = pdf_dir;
    ray r(light_rec.p + (0.001 * dir), dir, 0.0);

    for (int depth = 0; depth < cam.max_depth - 1; ++depth) {
        hit_record rec;
        if (!world.hit(r, interval(0.001, infinity), rec)) break;

        PathVertex v;
        v.rec = rec;
        v.beta = beta;
        v.wi = -unit_vector(r.direction());
        v.is_delta = rec.mat->is_delta();
        v.is_light = rec.mat->is_emissive();
        v.pdf_fwd = pdf_fwd;
        v.pdf_rev = 0.0; // Placeholder

        path.push_back(v);

        scatter_record srec;
        if (!rec.mat->scatter(r, rec, srec)) break;

        if (srec.skip_pdf) {
            beta = beta * srec.attenuation;
            r = srec.skip_pdf_ray;
            pdf_fwd = 0.0;
            path.back().pdf_rev = 0.0;
        } else {
            auto bsdf_pdf = srec.pdf_ptr; 
            vec3 scatter_dir = bsdf_pdf->generate();
            double pdf_val = bsdf_pdf->value(scatter_dir);
            
            if (pdf_val <= 0) break;

            color f = rec.mat->evaluate_bsdf(rec, v.wi, scatter_dir);
            double cos_scat = std::abs(dot(rec.normal, scatter_dir));

            beta = beta * (f * cos_scat / pdf_val);
            
            ray scatter_ray(rec.p, scatter_dir, r.time());
            ray incoming_ray(rec.p, -v.wi, r.time());
            path.back().pdf_rev = rec.mat->scattering_pdf(scatter_ray, rec, incoming_ray);

            r = ray(rec.p + (0.001*scatter_dir), scatter_dir, r.time());
            pdf_fwd = pdf_val;
        }
    }
}

color BidirectionalPathTracer::evaluate_connection(
    const std::vector<PathVertex>& cam_path,
    const std::vector<PathVertex>& light_path,
    int s, int t
) const {
    color L(0,0,0);

    if (s == 0) {
        const PathVertex& cam_v = cam_path[t-1];
        if (cam_v.is_light) {
            ray view_ray(cam_v.rec.p + cam_v.wi, -cam_v.wi, 0.0);
            color Le = cam_v.rec.mat->emitted(view_ray, cam_v.rec, cam_v.rec.u, cam_v.rec.v, cam_v.rec.p);
            L = cam_v.beta * Le;
        }
    } 
    else if (t == 1 && s > 0) {
        // Optional: Handle connections directly to camera (usually handled by splatting)
        // For this simplified tracer, we might skip t=1 if the camera model isn't physically 
        // integrated into the scene geometry.
    }
    else {
        const PathVertex& cam_v = cam_path[t-1];
        const PathVertex& light_v = light_path[s-1];

        if (cam_v.is_delta || light_v.is_delta) return color(0,0,0);

        vec3 diff = light_v.rec.p - cam_v.rec.p;
        double dist2 = diff.length_squared();
        double dist = std::sqrt(dist2);
        vec3 dir = diff / dist;

        ray shadow_ray(cam_v.rec.p + (0.001*dir), dir, 0.0);
        hit_record obs;
        if (world.hit(shadow_ray, interval(0.001, dist - 0.002), obs)) {
            return color(0,0,0);
        }

        double G = geometry_term(cam_v, light_v);
        if (G <= 0) return color(0,0,0);

        color f_cam = cam_v.rec.mat->evaluate_bsdf(cam_v.rec, cam_v.wi, dir);
        color f_light = light_v.rec.mat->evaluate_bsdf(light_v.rec, light_v.wi, -dir);

        L = cam_v.beta * f_cam * G * f_light * light_v.beta;
    }

    if (L.length_squared() <= 0) return color(0,0,0);

    double weight = mis_weight(cam_path, light_path, s, t);
    return L * weight;
}

double BidirectionalPathTracer::mis_weight(
    const std::vector<PathVertex>& cam_path,
    const std::vector<PathVertex>& light_path,
    int s, int t
) const {
    double sum_ratios = 1.0;
    
    const PathVertex* qs = (s > 0) ? &light_path[s-1] : nullptr;
    const PathVertex* pt = (t > 0) ? &cam_path[t-1] : nullptr;

    // 1. Light Path Strategies (reducing t, increasing s)
    {
        double pdf_rev_s = 0.0; 
        if (pt && qs) {
            double dist2 = (pt->rec.p - qs->rec.p).length_squared();
            vec3 dir_to_light = unit_vector(qs->rec.p - pt->rec.p);
            
            ray outgoing(pt->rec.p, dir_to_light, 0.0);
            ray incoming(pt->rec.p, -pt->wi, 0.0);
            
            double pdf_solid = pt->rec.mat->scattering_pdf(outgoing, pt->rec, incoming);
            pdf_rev_s = pdf_solid_to_area(pdf_solid, dist2, dot(qs->rec.normal, -dir_to_light));
        } else if (pt && s==0) {
             pdf_rev_s = lights.pdf_value(pt->rec.p, vec3(0,1,0));
        }

        double ri = 1.0;
        for (int i = t - 1; i > 0; --i) {
            double p_fwd = cam_path[i].pdf_fwd;
            double p_rev = cam_path[i].pdf_rev;
            
            double dist2 = (cam_path[i].rec.p - cam_path[i-1].rec.p).length_squared();
            double cos_curr = std::abs(dot(cam_path[i].rec.normal, -cam_path[i].wi));
            double cos_prev = std::abs(dot(cam_path[i-1].rec.normal, cam_path[i].wi)); 
            
            double p_fwd_area = pdf_solid_to_area(p_fwd, dist2, cos_curr);
            double p_rev_area = pdf_solid_to_area(p_rev, dist2, cos_prev);

            if (i == t-1) p_rev_area = pdf_rev_s;
            if (p_fwd_area <= 1e-20) { ri = 0; break; }
            
            ri *= (p_rev_area / p_fwd_area);
            if (!cam_path[i].is_delta && !cam_path[i-1].is_delta) sum_ratios += ri;
        }
    }

    // 2. Camera Path Strategies (reducing s, increasing t)
    {
        double pdf_rev_t = 0.0;
        if (qs && pt) {
            double dist2 = (qs->rec.p - pt->rec.p).length_squared();
            vec3 dir_to_cam = unit_vector(pt->rec.p - qs->rec.p);
            
            ray outgoing(qs->rec.p, dir_to_cam, 0.0);
            ray incoming(qs->rec.p, -qs->wi, 0.0);

            double pdf_solid = qs->rec.mat->scattering_pdf(outgoing, qs->rec, incoming);
            pdf_rev_t = pdf_solid_to_area(pdf_solid, dist2, dot(pt->rec.normal, -dir_to_cam));
        }

        double ri = 1.0;
        for (int i = s - 1; i >= 0; --i) {
            double p_fwd_area = 0.0;
            double p_rev_area = 0.0;

            if (i == 0) {
                p_fwd_area = light_path[0].pdf_fwd;
            } else {
                double dist2 = (light_path[i].rec.p - light_path[i-1].rec.p).length_squared();
                double cos_curr = std::abs(dot(light_path[i].rec.normal, -light_path[i].wi));
                p_fwd_area = pdf_solid_to_area(light_path[i].pdf_fwd, dist2, cos_curr);
            }

            if (i == s-1) {
                p_rev_area = pdf_rev_t;
            } else {
                 double dist2 = (light_path[i].rec.p - light_path[i+1].rec.p).length_squared();
                 double cos_i = std::abs(dot(light_path[i].rec.normal, unit_vector(light_path[i+1].rec.p - light_path[i].rec.p)));
                 p_rev_area = pdf_solid_to_area(light_path[i+1].pdf_rev, dist2, cos_i);
            }

            if (p_fwd_area <= 1e-20) { ri = 0; break; }
            
            ri *= (p_rev_area / p_fwd_area);
            
            bool curr_delta = light_path[i].is_delta;
            bool prev_delta = (i > 0) ? light_path[i-1].is_delta : false;
            
            if (!curr_delta && !prev_delta) sum_ratios += ri;
        }
    }

    return 1.0 / sum_ratios;
}

vec3 BidirectionalPathTracer::sample_cosine_hemisphere(const vec3& normal) const {
    onb basis(normal);
    return basis.transform(random_cosine_direction());
}