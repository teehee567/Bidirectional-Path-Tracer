#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "main.h"
#include "hittable.h"
#include "hittable_list.h"
#include "stats.h"

struct surface_sample {
    point3 position;
    vec3 normal;
    shared_ptr<material> mat;
    double pdf;
};

#include <array>
#include <vector>

class triangle : public hittable {
  public:
    triangle(const point3& v0, const point3& v1, const point3& v2, shared_ptr<material> mat)
      : v0(v0), v1(v1), v2(v2), mat(mat)
    {
        edge1 = v1 - v0;
        edge2 = v2 - v0;
        auto n = cross(edge1, edge2);
        normal = unit_vector(n);
        double double_area = n.length();
        area = 0.5 * double_area;

        auto min_x = std::fmin(v0.x(), std::fmin(v1.x(), v2.x()));
        auto max_x = std::fmax(v0.x(), std::fmax(v1.x(), v2.x()));
        auto min_y = std::fmin(v0.y(), std::fmin(v1.y(), v2.y()));
        auto max_y = std::fmax(v0.y(), std::fmax(v1.y(), v2.y()));
        auto min_z = std::fmin(v0.z(), std::fmin(v1.z(), v2.z()));
        auto max_z = std::fmax(v0.z(), std::fmax(v1.z(), v2.z()));

        bbox = aabb(point3(min_x, min_y, min_z), point3(max_x, max_y, max_z));
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        bvh_stats().triangle_tests.fetch_add(1, std::memory_order_relaxed);
        const double epsilon = 1e-8;

        vec3 pvec = cross(r.direction(), edge2);
        double det = dot(edge1, pvec);

        if (std::fabs(det) < epsilon)
            return false;

        double inv_det = 1.0 / det;
        vec3 tvec = r.origin() - v0;
        double u = dot(tvec, pvec) * inv_det;
        if (u < 0.0 || u > 1.0)
            return false;

        vec3 qvec = cross(tvec, edge1);
        double v = dot(r.direction(), qvec) * inv_det;
        if (v < 0.0 || u + v > 1.0)
            return false;

        double t = dot(edge2, qvec) * inv_det;
        if (!ray_t.contains(t))
            return false;

        rec.t = t;
        rec.p = r.at(t);
        rec.u = u;
        rec.v = v;
        rec.mat = mat;
        rec.set_face_normal(r, normal);
        bvh_stats().triangle_hits.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    aabb bounding_box() const override { return bbox; }

    double pdf_value(const point3& origin, const vec3& direction) const override {
        hit_record rec;
        if (!this->hit(ray(origin, direction), interval(0.001, infinity), rec))
            return 0.0;

        auto distance_squared = rec.t * rec.t * direction.length_squared();
        auto cosine = std::fabs(dot(direction, rec.normal) / direction.length());

        if (area <= 0.0 || cosine <= 0.0)
            return 0.0;

        return distance_squared / (cosine * area);
    }

    vec3 random(const point3& origin) const override {
        double u = random_double();
        double v = random_double();

        if (u + v > 1.0) {
            u = 1.0 - u;
            v = 1.0 - v;
        }

        auto p = v0 + (u * edge1) + (v * edge2);
        return p - origin;
    }

    double surface_area() const { return area; }

    void sample(surface_sample& sample) const {
        double u = random_double();
        double v = random_double();

        if (u + v > 1.0) {
            u = 1.0 - u;
            v = 1.0 - v;
        }

        sample.position = v0 + (u * edge1) + (v * edge2);
        sample.normal = normal;
        sample.mat = mat;
    }

    shared_ptr<material> material_ptr() const { return mat; }

  private:
    point3 v0;
    point3 v1;
    point3 v2;
    vec3 edge1;
    vec3 edge2;
    vec3 normal;
    shared_ptr<material> mat;
    aabb bbox;
    double area;
};

class triangle_collection : public hittable {
  public:
    triangle_collection() = default;

    void clear() {
        tris.clear();
        bbox = aabb();
    }

    void add(const triangle& tri) {
        tris.push_back(tri);
        bbox = aabb(bbox, tri.bounding_box());
    }

    bool empty() const { return tris.empty(); }
    size_t size() const { return tris.size(); }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = ray_t.max;

        for (const auto& tri : tris) {
            if (tri.hit(r, interval(ray_t.min, closest_so_far), temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        return hit_anything;
    }

    aabb bounding_box() const override { return bbox; }

    double pdf_value(const point3& origin, const vec3& direction) const override {
        if (tris.empty())
            return 0.0;

        auto weight = 1.0 / tris.size();
        double sum = 0.0;

        for (const auto& tri : tris)
            sum += weight * tri.pdf_value(origin, direction);

        return sum;
    }

    vec3 random(const point3& origin) const override {
        if (tris.empty())
            return vec3(1,0,0);

        auto idx = random_int(0, static_cast<int>(tris.size()) - 1);
        return tris[idx].random(origin);
    }

    hittable_list to_hittable_list() const {
        hittable_list list;
        for (const auto& tri : tris) {
            list.add(make_shared<triangle>(tri));
        }
        return list;
    }

    bool sample_surface(surface_sample& sample) const {
        if (tris.empty())
            return false;

        double total_area = 0.0;
        for (const auto& tri : tris)
            total_area += tri.surface_area();

        if (total_area <= 0.0)
            return false;

        double pick = random_double(0.0, total_area);
        const triangle* chosen = &tris.back();
        double accum = 0.0;
        for (const auto& tri : tris) {
            accum += tri.surface_area();
            if (pick <= accum) {
                chosen = &tri;
                break;
            }
        }

        chosen->sample(sample);
        sample.pdf = 1.0 / total_area;
        return true;
    }


  private:
    std::vector<triangle> tris;
    aabb bbox;
};

inline void add_quad_triangles(
    triangle_collection& collection,
    const point3& q,
    const vec3& u,
    const vec3& v,
    shared_ptr<material> mat
) {
    collection.add(triangle(q, q + u, q + v, mat));
    collection.add(triangle(q + u, q + u + v, q + v, mat));
}

inline point3 rotate_y_point(const point3& p, double sin_theta, double cos_theta) {
    return point3(
        (cos_theta * p.x()) + (sin_theta * p.z()),
        p.y(),
        (-sin_theta * p.x()) + (cos_theta * p.z())
    );
}

inline void add_box_triangles(
    triangle_collection& collection,
    const point3& a,
    const point3& b,
    shared_ptr<material> mat,
    double rotate_y_degrees = 0.0,
    const vec3& translate = vec3(0,0,0)
) {
    auto min_pt = point3(
        std::fmin(a.x(), b.x()),
        std::fmin(a.y(), b.y()),
        std::fmin(a.z(), b.z())
    );
    auto max_pt = point3(
        std::fmax(a.x(), b.x()),
        std::fmax(a.y(), b.y()),
        std::fmax(a.z(), b.z())
    );

    point3 v000(min_pt.x(), min_pt.y(), min_pt.z());
    point3 v001(min_pt.x(), min_pt.y(), max_pt.z());
    point3 v010(min_pt.x(), max_pt.y(), min_pt.z());
    point3 v011(min_pt.x(), max_pt.y(), max_pt.z());
    point3 v100(max_pt.x(), min_pt.y(), min_pt.z());
    point3 v101(max_pt.x(), min_pt.y(), max_pt.z());
    point3 v110(max_pt.x(), max_pt.y(), min_pt.z());
    point3 v111(max_pt.x(), max_pt.y(), max_pt.z());

    std::vector<std::array<point3,3>> faces = {
        {v001, v101, v111}, {v001, v111, v011}, // +Z
        {v000, v010, v110}, {v000, v110, v100}, // -Z
        {v000, v001, v011}, {v000, v011, v010}, // -X
        {v101, v100, v110}, {v101, v110, v111}, // +X
        {v011, v111, v110}, {v011, v110, v010}, // +Y
        {v000, v100, v101}, {v000, v101, v001}  // -Y
    };

    double radians = degrees_to_radians(rotate_y_degrees);
    double sin_theta = std::sin(radians);
    double cos_theta = std::cos(radians);

    for (const auto& verts : faces) {
        point3 p0 = verts[0];
        point3 p1 = verts[1];
        point3 p2 = verts[2];

        if (rotate_y_degrees != 0.0) {
            p0 = rotate_y_point(p0, sin_theta, cos_theta);
            p1 = rotate_y_point(p1, sin_theta, cos_theta);
            p2 = rotate_y_point(p2, sin_theta, cos_theta);
        }

        p0 += translate;
        p1 += translate;
        p2 += translate;

        collection.add(triangle(p0, p1, p2, mat));
    }
}

#endif
