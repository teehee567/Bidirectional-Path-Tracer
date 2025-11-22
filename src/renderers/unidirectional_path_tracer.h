#ifndef UNIDIRECTIONAL_PATH_TRACER_H
#define UNIDIRECTIONAL_PATH_TRACER_H

#include "camera.h"
#include "objects/hittable.h"
#include "objects/primatives/triangle.h"

class UnidirectionalPathTracer {
  public:
    UnidirectionalPathTracer(camera& cam, const hittable& world, const triangle_collection& lights);

    void render();

  private:
    color render_pixel(int i, int j) const;
    color trace_ray(const ray& r) const;
    color path_trace(const ray& r, int depth) const;

    camera& cam;
    const hittable& world;
    const triangle_collection& lights;
};

#endif


