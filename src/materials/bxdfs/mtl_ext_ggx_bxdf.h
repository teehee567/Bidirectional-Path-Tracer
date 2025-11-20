#ifndef MTL_EXT_GGX_BXDF_H
#define MTL_EXT_GGX_BXDF_H

#include "../bxdf_utils.h"

class MtlExtGGX : public BxDF {
  public:
    MtlExtGGX(shared_ptr<texture> baseColor,
              shared_ptr<texture> metallic,
              shared_ptr<texture> roughness,
              double ior,
              double dissolve,
              shared_ptr<texture> normalMap = nullptr)
      : base_color(baseColor),
        metallic_tex(metallic),
        roughness_tex(roughness),
        normal_map(normalMap ? normalMap : make_shared<solid_color>(color(0,0,1))),
        ior(ior),
        dissolve(std::clamp(dissolve, 0.0, 1.0)) {}

  private:
    Spectrum evaluate(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        double metallic_factor = std::clamp(sample_scalar(metallic_tex, texPos), 0.0, 1.0);
        double rough_val = std::clamp(sample_scalar(roughness_tex, texPos), 1e-4, 1.0);

        if (i.z() * o.z() > 0) {
            Vec3 n = normalize_or_default(i + o);
            Spectrum spec = F(texPos, i, o, n) * D(texPos, n) * G(texPos, i, o, n)
                / std::max(1e-6, 4.0 * std::abs(i.z() * o.z())) * metallic_factor;
            Spectrum diff = sample_texture(base_color, texPos) / pi * dissolve * (1.0 - metallic_factor);
            return spec + diff;
        } else {
            Vec3 n = normalize_or_default(o + i * ((o.z() > 0) ? ior : (1.0 / ior)));
            Spectrum trans = (color(1,1,1) - F(texPos, i, o, n)) * D(texPos, n) * G(texPos, i, o, n);
            Vec3 denom_vec = o + i * ((o.z() > 0) ? ior : (1.0 / ior));
            double denom = std::max(1e-6, denom_vec.length_squared());
            trans = trans * (std::abs(dot(i, n)) * std::abs(dot(o, n))) / std::max(1e-6, std::abs(i.z() * o.z()));
            trans = trans / denom * (1.0 - dissolve);
            return trans;
        }
    }

    Spectrum sample(const Vec2 &texPos, const Vec3 &i, Vec3 &o, double &pdf, Sampler &RNG) const override {
        double metallic_factor = std::clamp(sample_scalar(metallic_tex, texPos), 0.0, 1.0);
        double rough_val = std::clamp(sample_scalar(roughness_tex, texPos), 1e-4, 1.0);

        Vec3 n;
        sampleN(texPos, n, pdf, RNG);

        Vec3 reflected = -i + 2 * dot(i, n) * n;
        Spectrum fres = F(texPos, i, reflected, n);
        double prob = (rough_val + spectrum_norm(fres) * metallic_factor) / (1.0 + 2.0 * rough_val);

        double xi = RNG.rand();
        if (xi < prob) {
            o = reflected;
            double denom = std::max(1e-6, std::abs(4.0 * dot(i, n)));
            pdf = (pdf / denom) * prob;
            if (i.z() * o.z() <= 0)
                return Spectrum(0,0,0);
            return evaluate(texPos, i, o) * abs_cos_theta(o);
        } else if (xi < prob + dissolve) {
            o = RNG.hemisphere();
            if (i.z() * o.z() < 0)
                o[2] *= -1;
            pdf = (1.0 / (2.0 * pi)) * (1.0 - prob) * dissolve;
            Spectrum diff = sample_texture(base_color, texPos) / pi * (1.0 - metallic_factor);
            return diff * abs_cos_theta(o);
        } else {
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
            pdf = pdf * std::abs(dot(o, n)) / denom * (1.0 - prob) * (1.0 - dissolve);

            if (i.z() * o.z() > 0)
                return Spectrum(0,0,0);
            return evaluate(texPos, i, o) * abs_cos_theta(o);
        }
    }

    double evaluateImportance(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        double metallic_factor = std::clamp(sample_scalar(metallic_tex, texPos), 0.0, 1.0);
        double rough_val = std::clamp(sample_scalar(roughness_tex, texPos), 1e-4, 1.0);

        if (i.z() * o.z() > 0) {
            Vec3 n = normalize_or_default(i + o);
            Spectrum fres = F(texPos, i, o, n);
            double prob = (rough_val + spectrum_norm(fres) * metallic_factor) / (1.0 + 2.0 * rough_val);
            return D(texPos, n) * prob + (1.0 / (2.0 * pi)) * (1.0 - prob) * dissolve;
        } else {
            Vec3 n = normalize_or_default(o + i * ((o.z() > 0) ? ior : (1.0 / ior)));
            Spectrum fres = F(texPos, i, o, n);
            double prob = (rough_val + spectrum_norm(fres) * metallic_factor) / (1.0 + 2.0 * rough_val);
            return D(texPos, n) * (1.0 - prob) * (1.0 - dissolve);
        }
    }

    Spectrum F(const Vec2 &texPos, const Vec3 &i, const Vec3 &o, const Vec3 &n) const {
        Spectrum base = sample_texture(base_color, texPos);
        if (o.z() > 0) {
            double cos_theta = std::abs(dot(o, n));
            return base + (color(1,1,1) - base) * std::pow(1.0 - cos_theta, 5.0);
        } else {
            double val = std::max(0.0, (dot(o, n) * dot(o, n) - 1.0) * (ior * ior) + 1.0);
            double cos_theta = std::sqrt(val);
            return base + (color(1,1,1) - base) * std::pow(1.0 - cos_theta, 5.0);
        }
    }

    double D(const Vec2 &texPos, const Vec3 &n) const {
        Vec3 m = sample_texture(normal_map, texPos);
        m = normalize_or_default(m, vec3(0,0,1));
        double rough_val = std::clamp(sample_scalar(roughness_tex, texPos), 1e-4, 1.0);
        double alpha = rough_val * rough_val;
        double alpha2 = alpha * alpha;
        double nm = dot(n, m);
        double denom = std::pow(nm * nm * (alpha2 - 1.0) + 1.0, 2.0);
        return alpha2 / (pi * denom);
    }

    double G(const Vec2 &texPos, const Vec3 &i, const Vec3 &o, const Vec3 &n) const {
        double rough_val = std::clamp(sample_scalar(roughness_tex, texPos), 1e-4, 1.0);
        double alpha = rough_val * rough_val;
        double alpha2 = alpha * alpha;

        auto smith = [&](const Vec3& v) {
            double cos_v = std::abs(v.z());
            double sin_v = std::sqrt(std::max(0.0, 1.0 - cos_v * cos_v));
            double sign = (v.z() > 0) ? 1.0 : -1.0;
            if (cos_v <= 0)
                return 0.0;
            double tan_v = sign * sin_v / cos_v;
            return 2.0 / (1.0 + std::sqrt(1.0 + alpha2 * tan_v * tan_v));
        };

        return std::max(0.0, smith(i)) * std::max(0.0, smith(o));
    }

    void sampleN(const Vec2 &texPos, Vec3 &n, double &pdf, Sampler &RNG) const {
        double rough_val = std::clamp(sample_scalar(roughness_tex, texPos), 1e-4, 1.0);
        double alpha = rough_val * rough_val;
        double alpha2 = alpha * alpha;
        double p = RNG.rand();
        double q = RNG.rand(-pi, pi);

        double cos_n = std::sqrt(std::max(0.0, (1 - p) / (p * (alpha2 - 1) + 1)));
        double sin_n = std::sqrt(std::max(0.0, 1 - cos_n * cos_n));
        Vec3 lN = Vec3(std::cos(q) * sin_n, std::sin(q) * sin_n, cos_n);
        double denom = (alpha2 - 1) * cos_n * cos_n + 1;
        pdf = alpha2 * cos_n / (pi * denom * denom);

        Vec3 m = sample_texture(normal_map, texPos);
        m = normalize_or_default(m, vec3(0,0,1));
        Mat3 T, TInv;
        rotateAxis(m, m, T, TInv);
        n = TInv * lN;
    }

    shared_ptr<texture> base_color;
    shared_ptr<texture> metallic_tex;
    shared_ptr<texture> roughness_tex;
    shared_ptr<texture> normal_map;
    double ior;
    double dissolve;
};

#endif

