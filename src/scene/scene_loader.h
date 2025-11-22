#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include "camera.h"
#include "main.h"
#include "material.h"
#include "triangle.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

struct scene_load_result
{
    camera cam;
    triangle_collection world;
    triangle_collection lights;
};

scene_load_result load_scene_from_yaml(const std::string &path);

namespace scene_loader_detail
{

    // -----------------------------------------------------------------------------
    // Small YAML helpers
    // -----------------------------------------------------------------------------

    inline std::string
    as_string(const YAML::Node &node, const std::string &def = {})
    {
        if(!node || !node.IsScalar())
            return def;
        try
            {
                return node.as<std::string>();
            }
        catch(...)
            {
                return def;
            }
    }

    inline double as_double(const YAML::Node &node, double def = 0.0)
    {
        if(!node || !node.IsScalar())
            return def;
        try
            {
                return node.as<double>();
            }
        catch(...)
            {
                return def;
            }
    }

    inline int as_int(const YAML::Node &node, int def = 0)
    {
        if(!node || !node.IsScalar())
            return def;
        try
            {
                return node.as<int>();
            }
        catch(...)
            {
                return def;
            }
    }

    inline bool as_bool(const YAML::Node &node, bool def = false)
    {
        if(!node)
            return def;
        try
            {
                return node.as<bool>();
            }
        catch(...)
            {
                return def;
            }
    }

    inline std::vector<double> as_double_list(const YAML::Node &node)
    {
        std::vector<double> v;
        if(!node || !node.IsSequence())
            return v;
        v.reserve(node.size());
        for(const auto &e : node)
            v.push_back(as_double(e, 0.0));
        return v;
    }

    inline vec3 as_vec3(const YAML::Node &node, const vec3 &def)
    {
        auto values = as_double_list(node);
        if(values.size() < 3)
            return def;
        return vec3(values[0], values[1], values[2]);
    }

    inline color as_color_01(const YAML::Node &node, const color &def)
    {
        auto values = as_double_list(node);
        if(values.size() < 3)
            return def;
        return color(values[0], values[1], values[2]);
    }

    // Reads a color, allowing either [0,1] or [0,255] ranges.
    // If any absolute component is in (1, 255], we assume 0-255 and scale.
    inline color as_color_scaled(const YAML::Node &node, const color &def)
    {
        auto values = as_double_list(node);
        if(values.size() < 3)
            return def;

        double r = values[0], g = values[1], b = values[2];
        double maxc = std::max({std::fabs(r), std::fabs(g), std::fabs(b)});

        if(maxc > 1.0 && maxc <= 255.0)
            {
                const double s = 1.0 / 255.0;
                r *= s;
                g *= s;
                b *= s;
            }

        return color(r, g, b);
    }

    // -----------------------------------------------------------------------------
    // Materials
    // -----------------------------------------------------------------------------

    inline std::shared_ptr<material> build_material(const YAML::Node &node)
    {
        if(!node || !node.IsMap())
            throw std::runtime_error("Material must be a mapping");

        DisneyMaterialParams p;

        // Base color: accept a few synonyms, scaled to [0,1] when needed.
        color base = as_color_scaled(node["base_color"], color(0.8, 0.8, 0.8));
        base = as_color_scaled(node["base_colour"], base);
        base = as_color_scaled(node["albedo"], base);
        base = as_color_scaled(node["color"], base);
        p.base_color = base;

        p.emission = as_color_01(node["emission"], p.emission);
        p.transmittance_color = as_color_scaled(node["transmittance_color"],
                                                p.transmittance_color);

        p.metallic = as_double(node["metallic"], p.metallic);
        p.roughness = as_double(node["roughness"], p.roughness);
        p.specular_tint = as_double(node["specular_tint"], p.specular_tint);
        p.sheen = as_double(node["sheen"], p.sheen);
        p.sheen_tint = as_double(node["sheen_tint"], p.sheen_tint);
        p.clearcoat = as_double(node["clearcoat"], p.clearcoat);
        p.clearcoat_gloss
          = as_double(node["clearcoat_gloss"], p.clearcoat_gloss);
        p.anisotropic = as_double(node["anisotropic"], p.anisotropic);
        p.flatness = as_double(node["flatness"], p.flatness);
        p.scatter_distance
          = as_double(node["scatter_distance"], p.scatter_distance);
        p.relative_ior = as_double(node["relative_ior"], p.relative_ior);
        p.ior = as_double(node["ior"], p.ior);
        p.specular_transmission
          = as_double(node["spec_trans"], as_double(node["transmission"],
                                                    p.specular_transmission));
        p.diffuse_transmission = as_double(
          node["diff_trans"],
          as_double(node["diffuse_transmission"], p.diffuse_transmission));
        p.thin = as_bool(node["thin"], p.thin);

        const std::string type = as_string(node["type"], "disney");

        // Handle glass material type
        if (type == "glass") {
            GlassMaterialParams glass_params;
            glass_params.ior = as_double(node["ior"], 1.5);
            glass_params.tint = as_color_scaled(node["tint"], color(1.0, 1.0, 1.0));
            glass_params.tint = as_color_scaled(node["color"], glass_params.tint);
            return std::make_shared<GlassMaterial>(glass_params);
        }

        return std::make_shared<DisneyMaterial>(p);
    }

    // name -> material definition
    inline std::unordered_map<std::string, std::shared_ptr<material>>
    load_materials(const YAML::Node &materials_node)
    {
        std::unordered_map<std::string, std::shared_ptr<material>> mats;
        if(!materials_node || !materials_node.IsMap())
            return mats;

        for(auto it = materials_node.begin(); it != materials_node.end(); ++it)
            {
                const std::string name = as_string(it->first, "");
                if(name.empty())
                    continue;
                try
                    {
                        mats[name] = build_material(it->second);
                    }
                catch(...)
                    {
                        // Skip invalid entries; keep going.
                    }
            }
        return mats;
    }

    // -----------------------------------------------------------------------------
    // Transform matrix helpers
    // -----------------------------------------------------------------------------

    // Parse a 4x4 transform matrix from YAML (16 values in row-major order)
    inline std::vector<double> parse_transform_matrix(const YAML::Node &node)
    {
        std::vector<double> mat;
        if(!node || !node.IsSequence())
            return mat;

        auto values = as_double_list(node);
        if(values.size() == 16)
            return values;
        else if(values.size() > 0)
            throw std::runtime_error("Transform matrix must have exactly 16 values");

        return mat; // Empty matrix means identity (no transform)
    }

    // Apply a 4x4 transform matrix to a point (row-major order)
    inline point3 transform_point(const point3 &p, const std::vector<double> &m)
    {
        if(m.empty() || m.size() != 16)
            return p; // Identity transform

        // Matrix is in row-major order: m[0-3] is row 0, m[4-7] is row 1, etc.
        double x = p.x(), y = p.y(), z = p.z();
        double w = 1.0;

        double x_new = m[0] * x + m[1] * y + m[2] * z + m[3] * w;
        double y_new = m[4] * x + m[5] * y + m[6] * z + m[7] * w;
        double z_new = m[8] * x + m[9] * y + m[10] * z + m[11] * w;
        double w_new = m[12] * x + m[13] * y + m[14] * z + m[15] * w;

        // Homogeneous divide
        if(std::abs(w_new) > 1e-10)
            {
                return point3(x_new / w_new, y_new / w_new, z_new / w_new);
            }
        else
            {
                return point3(x_new, y_new, z_new);
            }
    }

    // -----------------------------------------------------------------------------
    // Geometry helpers
    // -----------------------------------------------------------------------------

    inline void
    add_triangle_with_lights(triangle_collection &world,
                             triangle_collection &lights, const point3 &v0,
                             const point3 &v1, const point3 &v2,
                             const std::shared_ptr<material> &mat)
    {
        triangle tri(v0, v1, v2, mat);
        world.add(tri);
        if(mat && mat->is_emissive())
            lights.add(tri);
    }

    inline point3
    sphere_point(double theta, double phi, const point3 &center, double radius)
    {
        double sin_theta = std::sin(theta);
        double x = radius * sin_theta * std::cos(phi);
        double y = radius * std::cos(theta);
        double z = radius * sin_theta * std::sin(phi);
        return center + vec3(x, y, z);
    }

    // Simple UV-sphere tesselation
    inline void
    add_uv_sphere(triangle_collection &world, triangle_collection &lights,
                  const point3 &center, double radius,
                  const std::shared_ptr<material> &mat, int lat_steps = 16,
                  int lon_steps = 32)
    {
        constexpr double pi = 3.14159265358979323846;

        for(int lat = 0; lat < lat_steps; ++lat)
            {
                double theta0 = pi * (static_cast<double>(lat) / lat_steps);
                double theta1
                  = pi * (static_cast<double>(lat + 1) / lat_steps);

                for(int lon = 0; lon < lon_steps; ++lon)
                    {
                        double phi0
                          = 2.0 * pi * (static_cast<double>(lon) / lon_steps);
                        double phi1
                          = 2.0 * pi
                            * (static_cast<double>(lon + 1) / lon_steps);

                        point3 p00
                          = sphere_point(theta0, phi0, center, radius);
                        point3 p01
                          = sphere_point(theta0, phi1, center, radius);
                        point3 p10
                          = sphere_point(theta1, phi0, center, radius);
                        point3 p11
                          = sphere_point(theta1, phi1, center, radius);

                        if(lat > 0)
                            add_triangle_with_lights(world, lights, p00, p10,
                                                     p11, mat);
                        if(lat < lat_steps - 1)
                            add_triangle_with_lights(world, lights, p00, p11,
                                                     p01, mat);
                    }
            }
    }

    // -----------------------------------------------------------------------------
    // Mesh loaders
    // -----------------------------------------------------------------------------

    // "TriMesh": vertices is a flat list [x0,y0,z0, x1,y1,z1, ...] in groups
    // of 9.
    inline void
    load_tri_mesh(const YAML::Node &mesh, triangle_collection &world,
                  triangle_collection &lights)
    {
        const YAML::Node data = mesh["data"];
        if(!data || !data.IsMap())
            throw std::runtime_error("TriMesh missing data field");

        auto vertices_node = data["vertices"];
        if(!vertices_node || !vertices_node.IsSequence())
            throw std::runtime_error("TriMesh missing vertices");

        std::vector<double> vertices = as_double_list(vertices_node);
        if(vertices.size() % 9 != 0)
            throw std::runtime_error(
              "TriMesh: vertices length must be multiple of 9");

        auto mat = build_material(mesh["material"]);
        if(!mat)
            throw std::runtime_error("TriMesh missing or invalid material");

        for(size_t i = 0; i < vertices.size(); i += 9)
            {
                point3 v0(vertices[i + 0], vertices[i + 1], vertices[i + 2]);
                point3 v1(vertices[i + 3], vertices[i + 4], vertices[i + 5]);
                point3 v2(vertices[i + 6], vertices[i + 7], vertices[i + 8]);
                add_triangle_with_lights(world, lights, v0, v1, v2, mat);
            }
    }

    // "Sphere": data.center, data.radius, and a material.
    inline void load_sphere(
      const YAML::Node &mesh, triangle_collection &world,
      triangle_collection &lights,
      const std::unordered_map<std::string, std::shared_ptr<material>>
        &materials)
    {
        std::shared_ptr<material> mat;
        const YAML::Node mat_node = mesh["material"];
        if(mat_node)
            {
                if(mat_node.IsScalar())
                    {
                        const std::string mat_name = as_string(mat_node);
                        auto it = materials.find(mat_name);
                        if(it != materials.end())
                            mat = it->second;
                    }
                else if(mat_node.IsMap())
                    {
                        mat = build_material(mat_node);
                    }
            }
        if(!mat)
            mat = std::make_shared<DisneyMaterial>(DisneyMaterialParams{});

        vec3 center_vec = as_vec3(mesh["center"], vec3(0, 0, 0));
        double radius = as_double(mesh["radius"], 0.0);
        if(radius <= 0.0)
            throw std::runtime_error("Sphere missing or invalid radius");

        add_uv_sphere(world, lights, point3(center_vec), radius, mat);
    }

    // "mesh": indexed triangles, plus material reference or inline definition.
    inline void load_indexed_mesh(
      const YAML::Node &mesh, triangle_collection &world,
      triangle_collection &lights,
      const std::unordered_map<std::string, std::shared_ptr<material>>
        &materials)
    {
        const YAML::Node verts_node = mesh["vertices"];
        const YAML::Node tris_node = mesh["triangles"];
        if(!verts_node || !verts_node.IsSequence())
            throw std::runtime_error("Indexed mesh missing vertices");
        if(!tris_node || !tris_node.IsSequence())
            throw std::runtime_error("Indexed mesh missing triangles");

        std::vector<point3> verts;
        verts.reserve(verts_node.size());
        for(const auto &v : verts_node)
            {
                auto vals = as_double_list(v);
                if(vals.size() < 3)
                    continue;
                verts.emplace_back(vals[0], vals[1], vals[2]);
            }

        std::shared_ptr<material> mat;
        const YAML::Node mat_node = mesh["material"];
        if(mat_node)
            {
                if(mat_node.IsScalar())
                    {
                        const std::string mat_name = as_string(mat_node);
                        auto it = materials.find(mat_name);
                        if(it != materials.end())
                            mat = it->second;
                    }
                else if(mat_node.IsMap())
                    {
                        mat = build_material(mat_node);
                    }
            }
        if(!mat)
            mat = std::make_shared<DisneyMaterial>(DisneyMaterialParams{});

        // Parse transform matrix if present
        std::vector<double> transform = parse_transform_matrix(mesh["transform"]);
        if(!transform.empty())
            {
                // Apply transform to all vertices
                for(auto &v : verts)
                    {
                        v = transform_point(v, transform);
                    }
            }

        for(const auto &tri : tris_node)
            {
                if(!tri.IsSequence())
                    continue;

                std::vector<int> idx;
                for(const auto &t : tri)
                    idx.push_back(as_int(t, 0));
                if(idx.size() < 3)
                    continue;

                const point3 &v0 = verts.at(static_cast<size_t>(idx[0]));
                const point3 &v1 = verts.at(static_cast<size_t>(idx[1]));
                const point3 &v2 = verts.at(static_cast<size_t>(idx[2]));
                add_triangle_with_lights(world, lights, v0, v1, v2, mat);
            }
    }

    // Very small OBJ loader: supports 'v' and 'f' lines.
    // Optionally applies a transform matrix to all vertices.
    inline void
    load_obj_file(const std::filesystem::path &obj_path,
                  const std::shared_ptr<material> &mat,
                  triangle_collection &world, triangle_collection &lights,
                  const std::vector<double> &transform = {})
    {
        std::ifstream in(obj_path);
        if(!in)
            throw std::runtime_error("Failed to open OBJ file: "
                                     + obj_path.string());

        std::vector<point3> verts;
        verts.reserve(1024);

        std::string line;
        while(std::getline(in, line))
            {
                if(line.empty())
                    continue;
                std::istringstream ss(line);
                std::string tag;
                ss >> tag;

                if(tag == "v")
                    {
                        double x, y, z;
                        if(ss >> x >> y >> z)
                            {
                                verts.emplace_back(x, y, z);
                            }
                    }
                else if(tag == "f")
                    {
                        std::vector<int> fidx;
                        std::string tok;
                        while(ss >> tok)
                            {
                                size_t slash = tok.find('/');
                                std::string vi_str
                                  = (slash == std::string::npos)
                                      ? tok
                                      : tok.substr(0, slash);
                                try
                                    {
                                        int vi = std::stoi(vi_str);
                                        int idx
                                          = (vi > 0)
                                              ? (vi - 1)
                                              : (static_cast<int>(verts.size())
                                                 + vi);
                                        fidx.push_back(idx);
                                    }
                                catch(...)
                                    {
                                        // skip malformed token
                                    }
                            }

                        if(fidx.size() >= 3)
                            {
                                // Fan triangulation
                                for(size_t k = 2; k < fidx.size(); ++k)
                                    {
                                        point3 v0 = verts.at(
                                          static_cast<size_t>(fidx[0]));
                                        point3 v1 = verts.at(
                                          static_cast<size_t>(fidx[k - 1]));
                                        point3 v2 = verts.at(
                                          static_cast<size_t>(fidx[k]));
                                        
                                        // Apply transform if provided
                                        if(!transform.empty())
                                            {
                                                v0 = transform_point(v0, transform);
                                                v1 = transform_point(v1, transform);
                                                v2 = transform_point(v2, transform);
                                            }
                                        
                                        add_triangle_with_lights(
                                          world, lights, v0, v1, v2, mat);
                                    }
                            }
                    }
            }
    }

    // "object": loads an external OBJ, optional material by name or inline.
    inline void
    load_object(const YAML::Node &node, const std::filesystem::path &yaml_dir,
                triangle_collection &world, triangle_collection &lights,
                const std::unordered_map<std::string,
                                         std::shared_ptr<material>> &materials)
    {
        const std::string file_rel = as_string(node["file"]);
        if(file_rel.empty())
            throw std::runtime_error("Object missing file field");

        const std::filesystem::path obj_path = yaml_dir / file_rel;

        std::shared_ptr<material> mat;
        const YAML::Node mat_node = node["material"];
        if(mat_node)
            {
                if(mat_node.IsScalar())
                    {
                        const std::string mat_name = as_string(mat_node);
                        auto it = materials.find(mat_name);
                        if(it != materials.end())
                            mat = it->second;
                    }
                else if(mat_node.IsMap())
                    {
                        mat = build_material(mat_node);
                    }
            }
        if(!mat)
            mat = std::make_shared<DisneyMaterial>(DisneyMaterialParams{});

        load_obj_file(obj_path, mat, world, lights);
    }

    // "obj": loads an external OBJ using "filename" field, supports transforms.
    inline void
    load_obj(const YAML::Node &node, const std::filesystem::path &yaml_dir,
             triangle_collection &world, triangle_collection &lights,
             const std::unordered_map<std::string,
                                      std::shared_ptr<material>> &materials)
    {
        const std::string file_rel = as_string(node["filename"]);
        if(file_rel.empty())
            throw std::runtime_error("Obj missing filename field");

        const std::filesystem::path obj_path = yaml_dir / file_rel;

        std::shared_ptr<material> mat;
        const YAML::Node mat_node = node["material"];
        if(mat_node)
            {
                if(mat_node.IsScalar())
                    {
                        const std::string mat_name = as_string(mat_node);
                        auto it = materials.find(mat_name);
                        if(it != materials.end())
                            mat = it->second;
                    }
                else if(mat_node.IsMap())
                    {
                        mat = build_material(mat_node);
                    }
            }
        if(!mat)
            mat = std::make_shared<DisneyMaterial>(DisneyMaterialParams{});

        // Parse transform matrix if present
        std::vector<double> transform = parse_transform_matrix(node["transform"]);

        load_obj_file(obj_path, mat, world, lights, transform);
    }

    // "cube": creates a unit cube transformed by the given transform matrix.
    inline void
    load_cube(const YAML::Node &node, triangle_collection &world,
              triangle_collection &lights,
              const std::unordered_map<std::string,
                                       std::shared_ptr<material>> &materials)
    {
        std::shared_ptr<material> mat;
        const YAML::Node mat_node = node["material"];
        if(mat_node)
            {
                if(mat_node.IsScalar())
                    {
                        const std::string mat_name = as_string(mat_node);
                        auto it = materials.find(mat_name);
                        if(it != materials.end())
                            mat = it->second;
                    }
                else if(mat_node.IsMap())
                    {
                        mat = build_material(mat_node);
                    }
            }
        if(!mat)
            mat = std::make_shared<DisneyMaterial>(DisneyMaterialParams{});

        // Parse transform matrix - required for cube
        std::vector<double> transform = parse_transform_matrix(node["transform"]);
        if(transform.empty())
            throw std::runtime_error("Cube missing transform matrix");

        // Create a unit cube from (-0.5, -0.5, -0.5) to (0.5, 0.5, 0.5)
        // then apply the transform
        point3 min_pt(-0.5, -0.5, -0.5);
        point3 max_pt(0.5, 0.5, 0.5);

        // Define cube vertices
        point3 v000(min_pt.x(), min_pt.y(), min_pt.z());
        point3 v001(min_pt.x(), min_pt.y(), max_pt.z());
        point3 v010(min_pt.x(), max_pt.y(), min_pt.z());
        point3 v011(min_pt.x(), max_pt.y(), max_pt.z());
        point3 v100(max_pt.x(), min_pt.y(), min_pt.z());
        point3 v101(max_pt.x(), min_pt.y(), max_pt.z());
        point3 v110(max_pt.x(), max_pt.y(), min_pt.z());
        point3 v111(max_pt.x(), max_pt.y(), max_pt.z());

        // Apply transform to all vertices
        v000 = transform_point(v000, transform);
        v001 = transform_point(v001, transform);
        v010 = transform_point(v010, transform);
        v011 = transform_point(v011, transform);
        v100 = transform_point(v100, transform);
        v101 = transform_point(v101, transform);
        v110 = transform_point(v110, transform);
        v111 = transform_point(v111, transform);

        // Define cube faces (12 triangles)
        std::vector<std::array<point3, 3>> faces = {
            {v001, v101, v111}, {v001, v111, v011}, // +Z
            {v000, v010, v110}, {v000, v110, v100}, // -Z
            {v000, v001, v011}, {v000, v011, v010}, // -X
            {v101, v100, v110}, {v101, v110, v111}, // +X
            {v011, v111, v110}, {v011, v110, v010}, // +Y
            {v000, v100, v101}, {v000, v101, v001}  // -Y
        };

        for(const auto &face : faces)
            {
                add_triangle_with_lights(world, lights, face[0], face[1],
                                         face[2], mat);
            }
    }

    // -----------------------------------------------------------------------------
    // Camera
    // -----------------------------------------------------------------------------

    inline void load_camera_from_yaml(const YAML::Node &node, camera &cam)
    {
        if(!node || !node.IsMap())
            throw std::runtime_error("Camera section must be a mapping");

        const YAML::Node res_node = node["resolution"];
        if(!res_node || !res_node.IsSequence() || res_node.size() < 2)
            throw std::runtime_error("Camera missing resolution");

        auto res_values = as_double_list(res_node);
        if(res_values.size() < 2)
            throw std::runtime_error("Camera missing resolution values");

        int width = static_cast<int>(res_values[0]);
        int height = static_cast<int>(res_values[1]);
        if(width <= 0 || height <= 0)
            throw std::runtime_error("Camera resolution must be positive");

        cam.image_width = width;
        cam.aspect_ratio
          = static_cast<double>(width) / static_cast<double>(height);

        // Vertical FOV in degrees (vfov/fov); clamped to a reasonable range.
        double vfov_deg = cam.vfov;
        if(node["vfov"] || node["fov"])
            {
                vfov_deg
                  = as_double(node["vfov"], as_double(node["fov"], vfov_deg));
            }
        vfov_deg = std::clamp(vfov_deg, 1.0, 179.0);
        cam.vfov = vfov_deg;

        cam.focus_dist = as_double(node["focus_distance"], cam.focus_dist);
        cam.defocus_angle = 0.0; // disable depth of field by default

        cam.lookfrom = point3(
          as_vec3(node["location"],
                  vec3(cam.lookfrom.x(), cam.lookfrom.y(), cam.lookfrom.z())));
        cam.lookat = point3(
          as_vec3(node["look_at"],
                  vec3(cam.lookat.x(), cam.lookat.y(), cam.lookat.z())));
        cam.vup = as_vec3(node["up"], cam.vup);
        cam.background = as_color_scaled(node["background"], cam.background);

        cam.samples_per_pixel
          = as_int(node["samples_per_pixel"], cam.samples_per_pixel);
        cam.max_depth = as_int(node["max_depth"], cam.max_depth);

        std::string output = as_string(node["output"]);
        if(!output.empty())
            cam.file_name = output;
    }

} // namespace scene_loader_detail

// -----------------------------------------------------------------------------
// Top-level scene loader
// -----------------------------------------------------------------------------

inline scene_load_result load_scene_from_yaml(const std::string &path)
{
    YAML::Node root = YAML::LoadFile(path);
    if(!root || !root.IsMap())
        throw std::runtime_error("Scene root must be a mapping");

    scene_load_result result;

    // Camera
    scene_loader_detail::load_camera_from_yaml(root["camera"], result.cam);

    // Materials (optional)
    auto materials = scene_loader_detail::load_materials(root["materials"]);

    // Prefer "surfaces", fall back to "scene" for older files
    YAML::Node surfaces = root["surfaces"];
    if(!surfaces)
        surfaces = root["scene"];
    if(!surfaces || !surfaces.IsSequence())
        throw std::runtime_error("Scene/surfaces field must be a sequence");

    const std::filesystem::path yaml_dir
      = std::filesystem::path(path).parent_path();

    for(const auto &mesh : surfaces)
        {
            if(!mesh.IsMap())
                throw std::runtime_error("Scene entries must be mappings");

            const std::string type
              = scene_loader_detail::as_string(mesh["type"]);
            if(type.empty())
                throw std::runtime_error("Mesh missing type field");

            using namespace scene_loader_detail;

            if(type == "TriMesh")
                {
                    load_tri_mesh(mesh, result.world, result.lights);
                }
            else if(type == "sphere")
                {
                    load_sphere(mesh, result.world, result.lights, materials);
                }
            else if(type == "mesh")
                {
                    load_indexed_mesh(mesh, result.world, result.lights,
                                      materials);
                }
            else if(type == "object")
                {
                    load_object(mesh, yaml_dir, result.world, result.lights,
                                materials);
                }
            else if(type == "obj")
                {
                    load_obj(mesh, yaml_dir, result.world, result.lights,
                             materials);
                }
            else if(type == "cube")
                {
                    load_cube(mesh, result.world, result.lights, materials);
                }
            else
                {
                    std::cerr << "Unknown mesh type: " << type << "\n";
                }
        }

    std::cout << "Triangles: " << result.world.size() << "\n";
    return result;
}

#endif // SCENE_LOADER_H
