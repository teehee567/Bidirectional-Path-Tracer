#ifndef BIGGX_BXDF_H
#define BIGGX_BXDF_H

#include "../bxdf_utils.h"

class BiGGX : public BxDF {
  public:
    BiGGX(double ior, double roughness) : ior(ior), rough(std::max(roughness, 1e-4)) {}

  private:
    Spectrum evaluate(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        if (i.z() * o.z() > 0) {
            Vec3 n = normalize_or_default(i + o);
            return fresnel(i, o, n) * D(n) * G(i, o, n) / std::max(1e-6, 4.0 * std::abs(i.z() * o.z()));
        }

        Vec3 n = normalize_or_default(o + i * ((o.z() > 0) ? ior : (1.0 / ior)));
        Spectrum Fv = color(1,1,1) - fresnel(i, o, n);
        Vec3 denom_vec = o + i * ((o.z() > 0) ? ior : (1.0 / ior));
        double denom = std::max(1e-6, denom_vec.length_squared());

        return Fv * D(n) * G(i, o, n)
             * (std::abs(dot(i, n)) * std::abs(dot(o, n)))
             / std::max(1e-6, std::abs(i.z() * o.z()))
             / denom;
    }

    Spectrum sample(const Vec2 &texPos, const Vec3 &i, Vec3 &o, double &pdf, Sampler &RNG) const override {
        Vec3 n;
        sampleN(n, pdf, RNG);

        Vec3 reflected = -i + 2 * dot(i, n) * n;
        Spectrum F = fresnel(i, reflected, n);
        double prob = (rough + spectrum_norm(F)) / (1.0 + 2.0 * rough);

        if (RNG.rand() < prob) {
            o = reflected;
            double denom = std::max(1e-6, std::abs(4.0 * dot(i, n)));
            pdf = (pdf / denom) * prob;
            if (!same_hemisphere(i, o))
                return Spectrum(0,0,0);
            return evaluate(texPos, i, o) * abs_cos_theta(o);
        }

        double cos_v = std::abs(dot(i, n));
        double sin_v = std::sqrt(std::max(0.0, 1.0 - cos_v * cos_v));
        double eta = (i.z() < 0) ? ior : (1.0 / ior);
        double sin_l = sin_v * eta;
        if (sin_l > 1.0) {
            pdf = 1.0;
            return Spectrum(0,0,0);
        }

        Vec3 n_corrected = (dot(n, i) < 0) ? -n : n;
        vec3 tangent = tangent_component(i, n_corrected);
        vec3 tangent_dir = normalize_or_default(tangent, vec3(1,0,0));
        double theta_t = std::asin(std::clamp(sin_l, -1.0, 1.0));
        double tan_t = std::tan(theta_t);

        o = -(n_corrected + tangent_dir * tan_t);
        o = normalize_or_default(o, vec3(0,0,-1));

        Vec3 denom_vec = o + i * ((i.z() < 0) ? ior : (1.0 / ior));
        double denom = std::max(1e-6, denom_vec.length_squared());
        pdf = pdf * std::abs(dot(o, n)) / denom * (1.0 - prob);

        if (same_hemisphere(i, o))
            return Spectrum(0,0,0);
        return evaluate(texPos, i, o) * abs_cos_theta(o);
    }

    double evaluateImportance(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        if (i.z() * o.z() > 0) {
            Vec3 n = normalize_or_default(i + o);
            double prob = (rough + spectrum_norm(fresnel(i, o, n))) / (1.0 + 2.0 * rough);
            return D(n) * prob;
        } else {
            Vec3 n = normalize_or_default(o + i * ((o.z() > 0) ? ior : (1.0 / ior)));
            double prob = (rough + spectrum_norm(fresnel(i, o, n))) / (1.0 + 2.0 * rough);
            return D(n) * (1.0 - prob);
        }
    }

    Spectrum fresnel(const Vec3 &i, const Vec3 &o, const Vec3 &n) const {
        double base = (1.0 - ior) / (1.0 + ior);
        double F0 = base * base;
        Spectrum F0s(F0, F0, F0);

        if (o.z() > 0) {
            double cos_theta = std::abs(dot(o, n));
            return F0s + (color(1,1,1) - F0s) * std::pow(1.0 - cos_theta, 5.0);
        } else {
            double val = std::max(0.0, (dot(o, n) * dot(o, n) - 1.0) * (ior * ior) + 1.0);
            double cos_theta = std::sqrt(val);
            return F0s + (color(1,1,1) - F0s) * std::pow(1.0 - cos_theta, 5.0);
        }
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

        auto lambda = [&](const Vec3& v) {
            double cos_v = std::abs(v.z());
            double sin_v = std::sqrt(std::max(0.0, 1.0 - cos_v * cos_v));
            if (cos_v <= 0) return 0.0;
            double tan_v = sin_v / cos_v;
            return 2.0 / (1.0 + std::sqrt(1.0 + alpha2 * tan_v * tan_v));
        };

        return std::max(0.0, lambda(i)) * std::max(0.0, lambda(o));
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

    double ior;
    double rough;
};

#endif

