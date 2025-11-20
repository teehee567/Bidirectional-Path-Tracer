#ifndef GGX_BXDF_H
#define GGX_BXDF_H

#include "../bxdf_utils.h"

class GGX : public BxDF {
  public:
    GGX(const Spectrum& F0, double roughness)
      : F0(F0), rough(std::clamp(roughness, 1e-4, 1.0)) {}

  private:
    Spectrum evaluate(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        if (i.z() * o.z() < 0)
            return Spectrum(0,0,0);

        Vec3 n = normalize_or_default(i + o);
        Spectrum spec = fresnel(i, o, n) * D(n) * G(i, o, n) / std::max(1e-6, 4.0 * std::abs(i.z() * o.z()));
        Spectrum diffuse = F0 * ((1.0 - G(i, o, n)) / pi);
        return spec + diffuse;
    }

    Spectrum sample(const Vec2 &texPos, const Vec3 &i, Vec3 &o, double &pdf, Sampler &RNG) const override {
        double prob = (3.0 - rough) / 3.0;
        if (RNG.rand() < prob) {
            Vec3 n;
            sampleN(n, pdf, RNG);
            o = -i + 2 * dot(i, n) * n;
            double denom = std::max(1e-6, std::abs(4.0 * dot(i, n)));
            pdf = (pdf / denom) * prob;
            if (i.z() * o.z() < 0)
                return Spectrum(0,0,0);
            return evaluate(texPos, i, o) * abs_cos_theta(o);
        } else {
            o = RNG.hemisphere();
            o = vec3(o.x(), o.y(), (i.z() > 0) ? std::abs(o.z()) : -std::abs(o.z()));
            pdf = (1.0 / (2.0 * pi)) * (1.0 - prob);
            if (i.z() * o.z() < 0)
                return Spectrum(0,0,0);
            return F0 * ((1.0 - G(i, o, normalize_or_default(i + o))) / pi) * abs_cos_theta(o);
        }
    }

    double evaluateImportance(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        if (i.z() * o.z() <= 0)
            return 0.0;

        double prob = (3.0 - rough) / 3.0;
        Vec3 n = normalize_or_default(i + o);
        return D(n) * prob + (1.0 / (2.0 * pi)) * (1.0 - prob);
    }

    Spectrum fresnel(const Vec3 &i, const Vec3 &o, const Vec3 &n) const {
        double cos_theta = std::abs(dot(o, n));
        return F0 + (color(1,1,1) - F0) * std::pow(1.0 - cos_theta, 5.0);
    }

    double D(const Vec3 &n) const {
        double alpha = rough * rough;
        double alpha2 = alpha * alpha;
        double cos_n = n.z();
        double denom = cos_n * cos_n * (alpha2 - 1.0) + 1.0;
        return alpha2 / (pi * denom * denom);
    }

    double G(const Vec3 &i, const Vec3 &o, const Vec3 &n) const {
        double alpha = rough * rough;
        double alpha2 = alpha * alpha;

        auto smith = [&](const Vec3& v) {
            double cos_v = std::abs(v.z());
            double sin_v = std::sqrt(std::max(0.0, 1.0 - cos_v * cos_v));
            if (cos_v <= 0)
                return 0.0;
            double tan_v = sin_v / cos_v;
            return 2.0 / (1.0 + std::sqrt(1.0 + alpha2 * tan_v * tan_v));
        };

        return std::max(0.0, smith(i)) * std::max(0.0, smith(o));
    }

    void sampleN(Vec3 &n, double &pdf, Sampler &RNG) const {
        double alpha = rough * rough;
        double alpha2 = alpha * alpha;
        double p = RNG.rand();
        double q = RNG.rand(-pi, pi);

        double cos_n = std::sqrt(std::max(0.0, (1 - p) / (p * (alpha2 - 1) + 1)));
        double sin_n = std::sqrt(std::max(0.0, 1 - cos_n * cos_n));

        n = Vec3(std::cos(q) * sin_n, std::sin(q) * sin_n, cos_n);
        double denom = (alpha2 - 1) * cos_n * cos_n + 1;
        pdf = alpha2 * cos_n / (pi * denom * denom);
    }

    Spectrum F0;
    double rough;
};

#endif

