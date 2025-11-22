#include "main.h"

#include "bvh.h"
#include "camera.h"
#include "hittable.h"
#include "material.h"
#include "triangle.h"
#include "scene_loader.h"
#include "renderers/bidirectional_path_tracer.h"
#include "renderers/unidirectional_path_tracer.h"

#include <iostream>

void cornell_box() {
    triangle_collection world;

    auto make_diffuse = [](const color& albedo) {
        DisneyMaterialParams params;
        params.base_color = albedo;
        params.metallic = 0.0;
        params.roughness = 1.0;
        return make_shared<DisneyMaterial>(params);
    };

    auto make_light = [](const color& emission) {
        DisneyMaterialParams params;
        params.base_color = color(0,0,0);
        params.emission = emission;
        return make_shared<DisneyMaterial>(params);
    };

    auto red   = make_diffuse(color(.65, .05, .05));
    auto white = make_diffuse(color(.73, .73, .73));
    auto green = make_diffuse(color(.12, .45, .15));
    auto light = make_light(color(15, 15, 15));

    // Cornell box sides
    add_quad_triangles(world, point3(555,0,0), vec3(0,0,555), vec3(0,555,0), green);
    add_quad_triangles(world, point3(0,0,555), vec3(0,0,-555), vec3(0,555,0), red);
    add_quad_triangles(world, point3(0,555,0), vec3(555,0,0), vec3(0,0,555), white);
    add_quad_triangles(world, point3(0,0,555), vec3(555,0,0), vec3(0,0,-555), white);
    add_quad_triangles(world, point3(555,0,555), vec3(-555,0,0), vec3(0,555,0), white);

    // Light
    add_quad_triangles(world, point3(213,554,227), vec3(130,0,0), vec3(0,0,105), light);

    // Box
    add_box_triangles(world, point3(0,0,0), point3(165,330,165), white, 15.0, vec3(265,0,295));

    // Light Sources
    auto empty_material = shared_ptr<material>();
    triangle_collection lights;
    add_quad_triangles(lights, point3(343,554,332), vec3(-130,0,0), vec3(0,0,-105), empty_material);

    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = 800;
    cam.samples_per_pixel = 5;
    cam.max_depth         = 10;
    cam.background        = color(0,0,0);

    cam.vfov     = 40;
    cam.lookfrom = point3(278, 278, -800);
    cam.lookat   = point3(278, 278, 0);
    cam.vup      = vec3(0, 1, 0);

    cam.defocus_angle = 0;

    cam.progress_bar_length = 30;
    cam.file_name = "cornell_box.png";

    auto world_bvh = make_shared<bvh_node>(world);
    BidirectionalPathTracer renderer(cam, *world_bvh, lights);
    renderer.render();
}

int main(int argc, char** argv) {
    try {
        if (argc > 1) {
            auto scene = load_scene_from_yaml(argv[1]);
            auto world_bvh = make_shared<bvh_node>(scene.world);
            const triangle_collection& lights = scene.lights.empty() ? scene.world : scene.lights;
            UnidirectionalPathTracer renderer(scene.cam, *world_bvh, lights);
            // BidirectionalPathTracer renderer(scene.cam, *world_bvh, lights);
            renderer.render();
        } else {
            cornell_box();
        }
    } catch (const std::exception& ex) {
        std::cerr << "Failed to load scene: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
