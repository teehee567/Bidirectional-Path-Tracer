#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include "main.h"
#include "camera.h"
#include "triangle.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

struct scene_load_result {
    camera cam;
    triangle_collection world;
    triangle_collection lights;
};

scene_load_result load_scene_from_yaml(const std::string& path);

namespace scene_loader_detail {

inline std::string node_to_string(const YAML::Node& node, const std::string& def = {}) {
    if (!node || !node.IsScalar())
        return def;
    try {
        return node.as<std::string>();
    } catch (...) {
        return def;
    }
}

inline double node_to_double(const YAML::Node& node, double def = 0.0) {
    if (!node || !node.IsScalar())
        return def;
    try {
        return node.as<double>();
    } catch (...) {
        return def;
    }
}

inline int node_to_int(const YAML::Node& node, int def = 0) {
    if (!node || !node.IsScalar())
        return def;
    try {
        return node.as<int>();
    } catch (...) {
        return def;
    }
}

inline std::vector<double> node_to_double_list(const YAML::Node& node) {
    std::vector<double> values;
    if (!node || !node.IsSequence())
        return values;
    values.reserve(node.size());
    for (const auto& element : node) {
        values.push_back(node_to_double(element, 0.0));
    }
    return values;
}

inline color read_color_node(const YAML::Node& node, const color& fallback) {
    auto values = node_to_double_list(node);
    if (values.size() < 3)
        return fallback;
    return color(values[0], values[1], values[2]);
}

inline vec3 read_vec3_node(const YAML::Node& node, const vec3& fallback) {
    auto values = node_to_double_list(node);
    if (values.size() < 3)
        return fallback;
    return vec3(values[0], values[1], values[2]);
}

inline std::shared_ptr<material> build_material(const YAML::Node& node) {
    if (!node || !node.IsMap())
        throw std::runtime_error("Material must be a mapping");

    const color default_color(0.0, 0.0, 0.0);

    color base_color = read_color_node(node["base_color"], default_color);
    color emission   = read_color_node(node["emission"], default_color);

    if (emission.length_squared() > 0.0)
        return std::make_shared<diffuse_light>(emission);

    double transmission = node_to_double(node["transmission"], 0.0);
    double ior = node_to_double(node["ior"], 1.5);
    if (transmission > 0.0)
        return std::make_shared<dielectric>(ior > 0.0 ? ior : 1.5);

    double metallic = node_to_double(node["metallic"], 0.0);
    double roughness = std::clamp(node_to_double(node["roughness"], 0.0), 0.0, 1.0);

    if (metallic > 0.5)
        return std::make_shared<metal>(base_color, roughness);

    return std::make_shared<lambertian>(base_color);
}

inline void add_triangle_with_lights(
    triangle_collection& world,
    triangle_collection& lights,
    const point3& v0,
    const point3& v1,
    const point3& v2,
    const std::shared_ptr<material>& mat
) {
    triangle tri(v0, v1, v2, mat);
    world.add(tri);
    if (std::dynamic_pointer_cast<diffuse_light>(mat))
        lights.add(tri);
}

inline point3 sphere_point(double theta, double phi, const point3& center, double radius) {
    double sin_theta = std::sin(theta);
    double x = radius * sin_theta * std::cos(phi);
    double y = radius * std::cos(theta);
    double z = radius * sin_theta * std::sin(phi);
    return center + vec3(x, y, z);
}

inline void add_uv_sphere(
    triangle_collection& world,
    triangle_collection& lights,
    const point3& center,
    double radius,
    const std::shared_ptr<material>& mat,
    int lat_steps = 16,
    int lon_steps = 32
) {
    const double pi = 3.14159265358979323846;

    for (int lat = 0; lat < lat_steps; ++lat) {
        double theta0 = pi * (static_cast<double>(lat) / lat_steps);
        double theta1 = pi * (static_cast<double>(lat + 1) / lat_steps);

        for (int lon = 0; lon < lon_steps; ++lon) {
            double phi0 = 2.0 * pi * (static_cast<double>(lon) / lon_steps);
            double phi1 = 2.0 * pi * (static_cast<double>(lon + 1) / lon_steps);

            point3 p00 = sphere_point(theta0, phi0, center, radius);
            point3 p01 = sphere_point(theta0, phi1, center, radius);
            point3 p10 = sphere_point(theta1, phi0, center, radius);
            point3 p11 = sphere_point(theta1, phi1, center, radius);

            if (lat > 0)
                add_triangle_with_lights(world, lights, p00, p10, p11, mat);
            if (lat < lat_steps - 1)
                add_triangle_with_lights(world, lights, p00, p11, p01, mat);
        }
    }
}

inline void load_tri_mesh(
    const YAML::Node& mesh,
    triangle_collection& world,
    triangle_collection& lights
) {
    const YAML::Node data = mesh["data"];
    if (!data || !data.IsMap())
        throw std::runtime_error("Mesh missing data field");

    const YAML::Node vertices_node = data["vertices"];
    if (!vertices_node || !vertices_node.IsSequence())
        throw std::runtime_error("Missing vertices");

    std::vector<double> vertices = node_to_double_list(vertices_node);
    if (vertices.size() % 9 != 0)
        throw std::runtime_error("Vertices length not a multiple of 9");

    const YAML::Node material_node = mesh["material"];
    if (!material_node)
        throw std::runtime_error("Missing material field");
    auto mat = build_material(material_node);

    for (size_t i = 0; i < vertices.size(); i += 9) {
        point3 v0(vertices[i + 0], vertices[i + 1], vertices[i + 2]);
        point3 v1(vertices[i + 3], vertices[i + 4], vertices[i + 5]);
        point3 v2(vertices[i + 6], vertices[i + 7], vertices[i + 8]);
        add_triangle_with_lights(world, lights, v0, v1, v2, mat);
    }
}

inline void load_sphere(
    const YAML::Node& mesh,
    triangle_collection& world,
    triangle_collection& lights
) {
    const YAML::Node material_node = mesh["material"];
    if (!material_node)
        throw std::runtime_error("Missing material field");
    auto mat = build_material(material_node);

    const YAML::Node data = mesh["data"];
    if (!data || !data.IsMap())
        throw std::runtime_error("Missing data field");

    vec3 center_vec = read_vec3_node(data["center"], vec3(0, 0, 0));
    double radius = node_to_double(data["radius"], 0.0);
    if (radius <= 0.0)
        throw std::runtime_error("Missing or invalid radius field");

    add_uv_sphere(world, lights, point3(center_vec), radius, mat);
}

inline void load_camera_from_yaml(const YAML::Node& node, camera& cam) {
    if (!node || !node.IsMap())
        throw std::runtime_error("Camera section must be a mapping");

    const YAML::Node resolution_node = node["resolution"];
    if (!resolution_node || !resolution_node.IsSequence() || resolution_node.size() < 2)
        throw std::runtime_error("Camera missing resolution");

    auto res_values = node_to_double_list(resolution_node);
    if (res_values.size() < 2)
        throw std::runtime_error("Camera missing resolution");

    int width = static_cast<int>(res_values[0]);
    int height = static_cast<int>(res_values[1]);
    if (width <= 0 || height <= 0)
        throw std::runtime_error("Resolution values must be positive");

    cam.image_width = width;
    cam.aspect_ratio = static_cast<double>(width) / static_cast<double>(height);

    cam.vfov = node_to_double(node["focal_length"], cam.vfov);
    cam.focus_dist = node_to_double(node["focus_distance"], cam.focus_dist);

    double aperture_radius = node_to_double(node["aperture_radius"], 0.0);
    if (cam.focus_dist > 0.0 && aperture_radius > 0.0) {
        double angle = std::atan(aperture_radius / cam.focus_dist);
        cam.defocus_angle = 2.0 * angle * (180.0 / 3.14159265358979323846);
    } else {
        cam.defocus_angle = 0.0;
    }

    cam.lookfrom = point3(read_vec3_node(node["location"], vec3(cam.lookfrom.x(), cam.lookfrom.y(), cam.lookfrom.z())));
    cam.lookat   = point3(read_vec3_node(node["look_at"], vec3(cam.lookat.x(), cam.lookat.y(), cam.lookat.z())));

    cam.vup = read_vec3_node(node["up"], cam.vup);
    cam.background = read_color_node(node["background"], cam.background);

    cam.samples_per_pixel = node_to_int(node["samples_per_pixel"], cam.samples_per_pixel);
    cam.max_depth = node_to_int(node["max_depth"], cam.max_depth);
    std::string output = node_to_string(node["output"]);
    if (!output.empty())
        cam.file_name = output;
}

} // namespace scene_loader_detail

inline scene_load_result load_scene_from_yaml(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    if (!root || !root.IsMap())
        throw std::runtime_error("Scene root must be a mapping");

    scene_load_result result;
    scene_loader_detail::load_camera_from_yaml(root["camera"], result.cam);

    const YAML::Node scene_node = root["scene"];
    if (!scene_node || !scene_node.IsSequence())
        throw std::runtime_error("Scene field missing or not a sequence");

    for (const auto& mesh : scene_node) {
        if (!mesh.IsMap())
            throw std::runtime_error("Scene entries must be mappings");

        std::string mesh_type = scene_loader_detail::node_to_string(mesh["type"]);
        if (mesh_type.empty())
            throw std::runtime_error("Mesh missing type field");

        if (mesh_type == "TriMesh") {
            scene_loader_detail::load_tri_mesh(mesh, result.world, result.lights);
        } else if (mesh_type == "Sphere") {
            scene_loader_detail::load_sphere(mesh, result.world, result.lights);
        } else {
            std::cerr << "Unknown mesh type: " << mesh_type << "\n";
        }
    }

    std::cout << "Triangles: " << result.world.size() << "\n";
    return result;
}

#endif
