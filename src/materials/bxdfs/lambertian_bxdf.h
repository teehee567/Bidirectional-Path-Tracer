#ifndef LAMBERTIAN_BXDF_H
#define LAMBERTIAN_BXDF_H

#include "../bxdf.h"

class LambertianBxDF : public BxDF {
public:
    LambertianBxDF(const color& albedo) : albedo(albedo) {}

    Spectrum evaluate(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        if (o.z() <= 0) return color(0,0,0);
        return albedo * (1.0 / pi);
    }

    Spectrum sample(const Vec2 &texPos, const Vec3 &i, Vec3 &o, double &pdf, Sampler &RNG) const override {
        vec3 d = random_cosine_direction();
        o = d;
        if (o.z() < 0) o = vec3(o.x(), o.y(), -o.z()); // Should be positive already

        double cos_theta = o.z();
        pdf = cos_theta / pi;

        return albedo * (cos_theta / pi);
    }

    double evaluateImportance(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        if (o.z() <= 0) return 0;
        return o.z() / pi;
    }

private:
    color albedo;
};

#endif

