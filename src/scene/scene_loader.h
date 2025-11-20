#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include "main.h"
#include "camera.h"
#include "triangle.h"
#include "bxdf_material.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <filesystem>

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

// Reads a color, allowing 0-1, 0-255 ranges. If any component > 1 and <= 255, scales by 1/255.
inline color read_color_node_scaled(const YAML::Node& node, const color& fallback) {
    auto values = node_to_double_list(node);
    if (values.size() < 3)
        return fallback;
    double r = values[0], g = values[1], b = values[2];
    double maxc = std::max({std::fabs(r), std::fabs(g), std::fabs(b)});
    if (maxc > 1.0 && maxc <= 255.0) {
        const double s = 1.0 / 255.0;
        r *= s; g *= s; b *= s;
    }
    return color(r, g, b);
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

    // Simple schema: type: lambertian|metal|dielectric|light with minimal params
    // - lambertian: color/albedo
    // - metal: color/albedo, roughness
    // - dielectric: ior
    // - light/diffuse_light: emission/color
    // NEW BXDF TYPES:
    // - lambertian_bxdf: albedo
    // - specular_reflection: reflectance
    // - specular_transmission: transmittance, eta_a, eta_b
    // - ggx: ior, roughness
    // - biggx: ior, roughness
    // - mtl_ggx: diffuse, specular, roughness, ior, dissolve
    // - mtl_ext_ggx: base_color, metallic, roughness, ior, dissolve
    const std::string type_str = node_to_string(node["type"], "");
    if (!type_str.empty()) {
        // Normalize expected keys across synonyms
        const color color_value = read_color_node_scaled(
            node["color"],
            read_color_node_scaled(node["albedo"],
                read_color_node_scaled(node["base_color"],
                    read_color_node_scaled(node["base_colour"], default_color)))
        );

        // NEW BXDF MATERIAL TYPES
        if (type_str == "lambertian_bxdf") {
            auto bxdf = std::make_shared<LambertianBxDF>(color_value);
            return std::make_shared<BxDFMaterial>(bxdf);
        } else if (type_str == "specular_reflection") {
            auto bxdf = std::make_shared<SpecularReflectionBxDF>(color_value);
            return std::make_shared<BxDFMaterial>(bxdf);
        } else if (type_str == "specular_transmission") {
            double eta_a = node_to_double(node["eta_a"], 1.0);
            double eta_b = node_to_double(node["eta_b"], 1.5);
            // Also accept 'ior' as eta_b for convenience
            if (node["ior"]) {
                eta_b = node_to_double(node["ior"], 1.5);
            }
            auto bxdf = std::make_shared<SpecularTransmissionBxDF>(color_value, eta_a, eta_b);
            return std::make_shared<BxDFMaterial>(bxdf);
        } else if (type_str == "biggx") {
            double ior = node_to_double(node["ior"], 1.5);
            double roughness = std::clamp(node_to_double(node["roughness"], 0.1), 1e-4, 1.0);
            auto bxdf = std::make_shared<BiGGX>(ior, roughness);
            return std::make_shared<BxDFMaterial>(bxdf);
        } else if (type_str == "ggx") {
            // GGX takes F0 (Fresnel reflectance at normal incidence) and roughness
            color f0 = read_color_node_scaled(node["f0"], color_value);
            if (f0.length_squared() == 0.0) {
                // If no F0 specified, use color/albedo as F0
                f0 = color_value.length_squared() > 0 ? color_value : color(0.04, 0.04, 0.04);
            }
            double roughness = std::clamp(node_to_double(node["roughness"], 0.1), 1e-4, 1.0);
            auto bxdf = std::make_shared<GGX>(f0, roughness);
            return std::make_shared<BxDFMaterial>(bxdf);
        } else if (type_str == "mtl_ggx") {
            auto diffuse_tex = std::make_shared<solid_color>(
                read_color_node_scaled(node["diffuse"], color_value)
            );
            auto specular_tex = std::make_shared<solid_color>(
                read_color_node_scaled(node["specular"], color(1,1,1))
            );
            double roughness = std::clamp(node_to_double(node["roughness"], 0.1), 1e-4, 1.0);
            double ior = node_to_double(node["ior"], 1.5);
            double dissolve = std::clamp(node_to_double(node["dissolve"], 1.0), 0.0, 1.0);
            auto bxdf = std::make_shared<MtlGGX>(diffuse_tex, specular_tex, roughness, ior, dissolve);
            return std::make_shared<BxDFMaterial>(bxdf);
        } else if (type_str == "mtl_ext_ggx") {
            auto base_color_tex = std::make_shared<solid_color>(color_value);
            double metallic_val = node_to_double(node["metallic"], 0.0);
            auto metallic_tex = std::make_shared<solid_color>(
                color(metallic_val, metallic_val, metallic_val)
            );
            double roughness_val = std::clamp(node_to_double(node["roughness"], 0.1), 1e-4, 1.0);
            auto roughness_tex = std::make_shared<solid_color>(
                color(roughness_val, roughness_val, roughness_val)
            );
            double ior = node_to_double(node["ior"], 1.5);
            double dissolve = std::clamp(node_to_double(node["dissolve"], 1.0), 0.0, 1.0);
            auto bxdf = std::make_shared<MtlExtGGX>(base_color_tex, metallic_tex, roughness_tex, ior, dissolve);
            return std::make_shared<BxDFMaterial>(bxdf);
        }
        // OLD MATERIAL TYPES (for backward compatibility)
        else if (type_str == "light" || type_str == "diffuse_light") {
            // For lights, treat emission as linear HDR. No 0–255 scaling.
            color emission = read_color_node(node["emission"], default_color);
            return std::make_shared<diffuse_light>(emission);
        } else if (type_str == "lambertian") {
            return std::make_shared<lambertian>(color_value);
        } else if (type_str == "metal") {
            double roughness = std::clamp(node_to_double(node["roughness"], 0.0), 0.0, 1.0);
            return std::make_shared<metal>(color_value, roughness);
        } else if (type_str == "dielectric" || type_str == "glass") {
            double ior = node_to_double(node["ior"], 1.5);
            return std::make_shared<dielectric>(ior > 0.0 ? ior : 1.5);
        }
        // Unknown type: fall through to legacy mapping below
    }

    // Legacy mapping: interpret common PBR-ish keys but produce simple materials
    // Accept both base_color and base_colour, and scale 0-255 if necessary
    color base_color = read_color_node_scaled(node["base_color"], default_color);
    if (node["base_colour"]) {
        base_color = read_color_node_scaled(node["base_colour"], base_color);
    }
    // Emission: if provided in 0-255 range, scale; otherwise accept absolute intensities
    color emission   = read_color_node_scaled(node["emission"], default_color);

    if (emission.length_squared() > 0.0) {
        double maxc = std::max({std::fabs(emission.x()), std::fabs(emission.y()), std::fabs(emission.z())});
        if (maxc > 50.0) {
            emission *= (50.0 / maxc);
        }
        return std::make_shared<diffuse_light>(emission);
    }

    // Map common PBR-ish keys to simple materials
    double transmission = node_to_double(node["transmission"], 0.0);
    if (transmission == 0.0)
        transmission = node_to_double(node["spec_trans"], 0.0);
    double ior = node_to_double(node["ior"], 1.5);
    if (transmission > 0.0)
        return std::make_shared<dielectric>(ior > 0.0 ? ior : 1.5);

    double metallic = node_to_double(node["metallic"], 0.0);
    double roughness = std::clamp(node_to_double(node["roughness"], 0.0), 0.0, 1.0);
    if (metallic > 0.5)
        return std::make_shared<metal>(base_color, roughness);

    // Default: diffuse
    return std::make_shared<lambertian>(base_color);
}

// Build materials map from a YAML mapping of name -> material_def
inline std::unordered_map<std::string, std::shared_ptr<material>> load_materials(const YAML::Node& materials_node) {
    std::unordered_map<std::string, std::shared_ptr<material>> materials;
    if (!materials_node || !materials_node.IsMap())
        return materials;

    for (auto it = materials_node.begin(); it != materials_node.end(); ++it) {
        const std::string name = it->first.as<std::string>();
        const YAML::Node def = it->second;
        try {
            materials[name] = build_material(def);
        } catch (...) {
            // Skip invalid material entries
        }
    }
    return materials;
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

// Load indexed mesh: vertices: [[x,y,z], ...], triangles: [[i,j,k], ...], material: "Name"
inline void load_indexed_mesh(
    const YAML::Node& mesh,
    triangle_collection& world,
    triangle_collection& lights,
    const std::unordered_map<std::string, std::shared_ptr<material>>& materials
) {
    const YAML::Node verts_node = mesh["vertices"];
    const YAML::Node tris_node  = mesh["triangles"];
    if (!verts_node || !verts_node.IsSequence())
        throw std::runtime_error("Indexed mesh missing vertices");
    if (!tris_node || !tris_node.IsSequence())
        throw std::runtime_error("Indexed mesh missing triangles");

    std::vector<point3> verts;
    verts.reserve(verts_node.size());
    for (const auto& v : verts_node) {
        auto vals = node_to_double_list(v);
        if (vals.size() < 3) continue;
        verts.emplace_back(vals[0], vals[1], vals[2]);
    }

    std::shared_ptr<material> mat;
    const YAML::Node mat_node = mesh["material"];
    if (mat_node) {
        if (mat_node.IsScalar()) {
            const std::string mat_name = node_to_string(mat_node);
            auto it = materials.find(mat_name);
            if (it != materials.end()) mat = it->second;
        } else if (mat_node.IsMap()) {
            mat = build_material(mat_node);
        }
    }
    if (!mat) mat = std::make_shared<lambertian>(color(0.8,0.8,0.8));

    for (const auto& tri : tris_node) {
        std::vector<int> idx;
        if (tri.IsSequence()) {
            for (const auto& t : tri) idx.push_back(node_to_int(t, 0));
        }
        if (idx.size() < 3) continue;
        // Assume indices are 0-based in YAML (as provided). If they are 1-based, adjust here.
        const point3& v0 = verts.at(static_cast<size_t>(idx[0]));
        const point3& v1 = verts.at(static_cast<size_t>(idx[1]));
        const point3& v2 = verts.at(static_cast<size_t>(idx[2]));
        add_triangle_with_lights(world, lights, v0, v1, v2, mat);
    }
}

// Very small OBJ loader: supports lines beginning with 'v' and 'f'.
inline void load_obj_file(
    const std::filesystem::path& obj_path,
    const std::shared_ptr<material>& mat,
    triangle_collection& world,
    triangle_collection& lights
) {
    std::ifstream in(obj_path);
    if (!in)
        throw std::runtime_error("Failed to open OBJ file: " + obj_path.string());

    std::vector<point3> verts;
    verts.reserve(1024);

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") {
            double x, y, z;
            if (ss >> x >> y >> z) {
                verts.emplace_back(x, y, z);
            }
        } else if (tag == "f") {
            std::vector<int> fidx;
            std::string tok;
            while (ss >> tok) {
                // token forms: vi | vi/vt | vi/vt/vn | vi//vn
                size_t slash = tok.find('/');
                std::string vi_str = (slash == std::string::npos) ? tok : tok.substr(0, slash);
                try {
                    int vi = std::stoi(vi_str);
                    // OBJ indices are 1-based; handle negative indices too
                    int idx = (vi > 0) ? (vi - 1) : (static_cast<int>(verts.size()) + vi);
                    fidx.push_back(idx);
                } catch (...) {
                    // skip malformed
                }
            }
            if (fidx.size() >= 3) {
                // fan triangulation
                for (size_t k = 2; k < fidx.size(); ++k) {
                    const point3& v0 = verts.at(static_cast<size_t>(fidx[0]));
                    const point3& v1 = verts.at(static_cast<size_t>(fidx[k-1]));
                    const point3& v2 = verts.at(static_cast<size_t>(fidx[k]));
                    add_triangle_with_lights(world, lights, v0, v1, v2, mat);
                }
            }
        }
    }
}

inline void load_object(
    const YAML::Node& node,
    const std::filesystem::path& yaml_dir,
    triangle_collection& world,
    triangle_collection& lights,
    const std::unordered_map<std::string, std::shared_ptr<material>>& materials
) {
    const std::string file_rel = node_to_string(node["file"]);
    if (file_rel.empty())
        throw std::runtime_error("Object missing file field");
    const std::filesystem::path obj_path = yaml_dir / file_rel;

    std::shared_ptr<material> mat;
    const YAML::Node mat_node = node["material"];
    if (mat_node) {
        if (mat_node.IsScalar()) {
            const std::string mat_name = node_to_string(mat_node);
            auto it = materials.find(mat_name);
            if (it != materials.end()) mat = it->second;
        } else if (mat_node.IsMap()) {
            mat = build_material(mat_node);
        }
    }
    if (!mat) mat = std::make_shared<lambertian>(color(0.8,0.8,0.8));

    load_obj_file(obj_path, mat, world, lights);
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

    // Field of view parsing:
    // Prefer explicit fov/vfov (in degrees). If absent, derive vfov from focal_length using
    // a default 35mm stills camera vertical sensor size (24mm), adjusted for aspect ratio if provided.
    double vfov_deg = cam.vfov;
    const bool has_vfov = static_cast<bool>(node["vfov"]) || static_cast<bool>(node["fov"]);

    if (has_vfov) {
        vfov_deg = node_to_double(node["vfov"], node_to_double(node["fov"], vfov_deg));
    }

    // Clamp to a sane range
    vfov_deg = std::clamp(vfov_deg, 1.0, 179.0);
    cam.vfov = vfov_deg;

    cam.focus_dist = node_to_double(node["focus_distance"], cam.focus_dist);
    // Disable defocus blur regardless of any aperture settings provided
    cam.defocus_angle = 0.0;

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

    // Materials map (optional)
    auto materials = scene_loader_detail::load_materials(root["materials"]);

    // Prefer 'surfaces' (coffee_machine.yaml), fallback to 'scene' (legacy)
    YAML::Node surfaces_node = root["surfaces"];
    if (!surfaces_node)
        surfaces_node = root["scene"];
    if (!surfaces_node || !surfaces_node.IsSequence())
        throw std::runtime_error("Scene/surfaces field missing or not a sequence");

    const std::filesystem::path yaml_dir = std::filesystem::path(path).parent_path();

    for (const auto& mesh : surfaces_node) {
        if (!mesh.IsMap())
            throw std::runtime_error("Scene entries must be mappings");

        std::string mesh_type = scene_loader_detail::node_to_string(mesh["type"]);
        if (mesh_type.empty())
            throw std::runtime_error("Mesh missing type field");

        if (mesh_type == "TriMesh") {
            scene_loader_detail::load_tri_mesh(mesh, result.world, result.lights);
        } else if (mesh_type == "Sphere") {
            scene_loader_detail::load_sphere(mesh, result.world, result.lights);
        } else if (mesh_type == "mesh") {
            scene_loader_detail::load_indexed_mesh(mesh, result.world, result.lights, materials);
        } else if (mesh_type == "object") {
            scene_loader_detail::load_object(mesh, yaml_dir, result.world, result.lights, materials);
        } else {
            std::cerr << "Unknown mesh type: " << mesh_type << "\n";
        }
    }

    std::cout << "Triangles: " << result.world.size() << "\n";
    return result;
}

#endif
