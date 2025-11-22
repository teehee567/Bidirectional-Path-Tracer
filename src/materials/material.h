#ifndef MATERIAL_H
#define MATERIAL_H

#include <algorithm>

#include "bsdf.h"
#include "hittable.h"
#include "pdf.h"

class scatter_record {
  public:
    color attenuation = color(1,1,1);
    shared_ptr<pdf> pdf_ptr;
    bool skip_pdf = false;
    ray skip_pdf_ray;
};

class material {
  public:
    virtual ~material() = default;

    virtual color emitted(
        const ray& r_in, const hit_record& rec, double u, double v, const point3& p
    ) const {
        return color(0,0,0);
    }

    virtual bool scatter(const ray& r_in, const hit_record& rec, scatter_record& srec) const = 0;

    virtual double scattering_pdf(const ray& r_in, const hit_record& rec, const ray& scattered)
    const = 0;

    // NOTE: wi is the outgoing/view direction (toward the previous vertex), and wo is the
    // sampled incoming direction (toward the next vertex). This matches the historical usage
    // in the integrator.
    virtual color evaluate_bsdf(const hit_record& rec, const vec3& wi, const vec3& wo) const = 0;

    virtual bool is_delta() const { return false; }
    virtual bool is_emissive() const { return false; }
};

struct DisneyMaterialParams {
    color base_color{0.8, 0.8, 0.8};
    color transmittance_color{1.0, 1.0, 1.0};
    color emission{0.0, 0.0, 0.0};

    double sheen = 0.0;
    double sheen_tint = 0.5;
    double clearcoat = 0.0;
    double clearcoat_gloss = 1.0;
    double metallic = 0.0;
    double specular_transmission = 0.0;
    double diffuse_transmission = 0.0;
    double flatness = 0.0;
    double anisotropic = 0.0;
    double relative_ior = 1.5;
    double specular_tint = 0.0;
    double roughness = 0.5;
    double scatter_distance = 1.0;
    double ior = 1.5;
    bool thin = false;
};

class DisneyMaterial : public material {
  public:
    explicit DisneyMaterial(const DisneyMaterialParams& params)
      : base_color(params.base_color),
        transmittance_color(params.transmittance_color),
        emission(params.emission),
        sheen(params.sheen),
        sheen_tint(params.sheen_tint),
        clearcoat(params.clearcoat),
        clearcoat_gloss(params.clearcoat_gloss),
        metallic(params.metallic),
        spec_trans(params.specular_transmission),
        diff_trans(params.diffuse_transmission),
        flatness(params.flatness),
        anisotropic(params.anisotropic),
        relative_ior(params.relative_ior),
        specular_tint(params.specular_tint),
        roughness(params.roughness),
        scatter_distance(params.scatter_distance),
        ior(params.ior),
        thin(params.thin)
    {}

    color emitted(
        const ray& r_in, const hit_record& rec, double u, double v, const point3& p
    ) const override {
        if (!rec.front_face)
            return color(0,0,0);
        return emission;
    }

    bool scatter(const ray& r_in, const hit_record& rec, scatter_record& srec) const override;

    double scattering_pdf(const ray& r_in, const hit_record& rec, const ray& scattered)
    const override;

    color evaluate_bsdf(const hit_record& rec, const vec3& wi, const vec3& wo) const override;

    bool is_emissive() const override {
        return emission.length_squared() > 0.0;
    }

  private:
    friend class DisneyPdf;

    bsdf::SurfaceParameters make_surface(const hit_record& rec, const vec3& view_dir) const;
    glm::vec3 sample_direction(const hit_record& rec, const vec3& view_dir) const;
    double pdf_value(const hit_record& rec, const vec3& view_dir, const vec3& sample_dir) const;

    static glm::vec3 to_glm(const vec3& v) {
        return glm::vec3(
            static_cast<float>(v.x()),
            static_cast<float>(v.y()),
            static_cast<float>(v.z())
        );
    }

    static vec3 from_glm(const glm::vec3& v) {
        return vec3(v.x, v.y, v.z);
    }

    color base_color;
    color transmittance_color;
    color emission;

    double sheen;
    double sheen_tint;
    double clearcoat;
    double clearcoat_gloss;
    double metallic;
    double spec_trans;
    double diff_trans;
    double flatness;
    double anisotropic;
    double relative_ior;
    double specular_tint;
    double roughness;
    double scatter_distance;
    double ior;
    bool thin;
};

class DisneyPdf : public pdf {
  public:
    DisneyPdf(const DisneyMaterial& material, const hit_record& rec, const vec3& view_dir)
      : material(material),
        rec(rec),
        view_dir(unit_vector(view_dir))
    {}

    double value(const vec3& direction) const override {
        return material.pdf_value(rec, view_dir, unit_vector(direction));
    }

    vec3 generate() const override {
        glm::vec3 sampled = material.sample_direction(rec, view_dir);
        if (glm::dot(sampled, sampled) == 0.0f)
            return rec.normal;
        return unit_vector(DisneyMaterial::from_glm(sampled));
    }

  private:
    const DisneyMaterial& material;
    hit_record rec;
    vec3 view_dir;
};

inline bool DisneyMaterial::scatter(
    const ray& r_in,
    const hit_record& rec,
    scatter_record& srec
) const {
    srec.attenuation = color(1,1,1);
    srec.pdf_ptr = make_shared<DisneyPdf>(*this, rec, unit_vector(-r_in.direction()));
    srec.skip_pdf = false;
    return true;
}

inline double DisneyMaterial::scattering_pdf(
    const ray& r_in,
    const hit_record& rec,
    const ray& scattered
) const {
    return pdf_value(rec, unit_vector(-r_in.direction()), unit_vector(scattered.direction()));
}

inline color DisneyMaterial::evaluate_bsdf(
    const hit_record& rec,
    const vec3& wi,
    const vec3& wo
) const {
    bsdf::SurfaceParameters surface = make_surface(rec, wi);
    float forwardPdf = 0.0f;
    float reversePdf = 0.0f;
    glm::vec3 reflectance = bsdf::EvaluateDisney(surface, to_glm(wi), to_glm(wo), thin, forwardPdf, reversePdf);

    double cos_theta = std::max(1e-6, std::fabs(dot(rec.normal, wo)));
    return from_glm(reflectance) / cos_theta;
}

inline bsdf::SurfaceParameters DisneyMaterial::make_surface(
    const hit_record& rec,
    const vec3& view_dir
) const {
    bsdf::SurfaceParameters surface;
    glm::vec3 normal = to_glm(rec.normal);

    surface.position = to_glm(rec.p);
    surface.basis = utils::LocalBasis::from_normal(normal);
    surface.worldToTangent = glm::transpose(surface.basis.transform);
    surface.error = static_cast<float>(rec.t);
    surface.view = to_glm(view_dir);
    surface.baseColor = to_glm(base_color);
    surface.transmittanceColor = to_glm(transmittance_color);
    surface.sheen = static_cast<float>(sheen);
    surface.sheenTint = static_cast<float>(sheen_tint);
    surface.clearcoat = static_cast<float>(clearcoat);
    surface.clearcoatGloss = static_cast<float>(clearcoat_gloss);
    surface.metallic = static_cast<float>(metallic);
    surface.specTrans = static_cast<float>(spec_trans);
    surface.diffTrans = static_cast<float>(diff_trans);
    surface.flatness = static_cast<float>(flatness);
    surface.anisotropic = static_cast<float>(anisotropic);
    surface.relativeIOR = static_cast<float>(relative_ior);
    surface.specularTint = static_cast<float>(specular_tint);
    surface.roughness = static_cast<float>(std::clamp(roughness, 0.001, 1.0));
    surface.scatterDistance = static_cast<float>(scatter_distance);
    surface.ior = static_cast<float>(ior);
    return surface;
}

inline glm::vec3 DisneyMaterial::sample_direction(
    const hit_record& rec,
    const vec3& view_dir
) const {
    bsdf::SurfaceParameters surface = make_surface(rec, view_dir);
    glm::vec3 view = to_glm(view_dir);

    float pSpecular, pDiffuse, pClearcoat, pSpecTrans;
    bsdf::CalculateLobePdfs(surface, pSpecular, pDiffuse, pClearcoat, pSpecTrans);

    const float picks[4] = {pSpecular, pDiffuse, pClearcoat, pSpecTrans};
    const auto sample_lobe = [&](int index, bsdf::BsdfSample& sample) -> bool {
        switch (index) {
            case 0: return bsdf::SampleDisneySpecular(surface, view, sample);
            case 1: return bsdf::SampleDisneyDiffuse(surface, view, thin, sample);
            case 2: return bsdf::SampleDisneyClearcoat(surface, view, sample);
            case 3: return bsdf::SampleDisneySpecTransmission(surface, view, thin, sample);
            default: return false;
        }
    };

    float xi = utils::sampler::random_double();
    float cumulative = 0.0f;
    for (int i = 0; i < 4; ++i) {
        cumulative += picks[i];
        if (picks[i] <= 0.0f)
            continue;
        if (xi <= cumulative) {
            bsdf::BsdfSample sample;
            if (sample_lobe(i, sample))
                return sample.wi;
            break;
        }
    }

    // Fallback: attempt lobes in priority order
    for (int i = 0; i < 4; ++i) {
        if (picks[i] <= 0.0f)
            continue;
        bsdf::BsdfSample sample;
        if (sample_lobe(i, sample))
            return sample.wi;
    }

    // Final fallback: cosine hemisphere
    float r0 = utils::sampler::random_double();
    float r1 = utils::sampler::random_double();
    glm::vec3 local = bsdf::SampleCosineWeightedHemisphere(r0, r1);
    return glm::normalize(surface.basis.transform * local);
}

inline double DisneyMaterial::pdf_value(
    const hit_record& rec,
    const vec3& view_dir,
    const vec3& sample_dir
) const {
    bsdf::SurfaceParameters surface = make_surface(rec, view_dir);
    float forwardPdf = 0.0f;
    float reversePdf = 0.0f;
    bsdf::EvaluateDisney(surface, to_glm(view_dir), to_glm(sample_dir), thin, forwardPdf, reversePdf);
    return static_cast<double>(forwardPdf);
}

// Glass material optimized for caustics - perfect specular with proper Fresnel
struct GlassMaterialParams {
    double ior = 1.5;  // Index of refraction (glass typically 1.5)
    color tint = color(1.0, 1.0, 1.0);  // Color tint for the glass
};

class GlassMaterial : public material {
  public:
    explicit GlassMaterial(const GlassMaterialParams& params)
      : ior(params.ior),
        tint(params.tint)
    {}

    color emitted(
        const ray& r_in, const hit_record& rec, double u, double v, const point3& p
    ) const override {
        return color(0,0,0);
    }

    bool scatter(const ray& r_in, const hit_record& rec, scatter_record& srec) const override;

    double scattering_pdf(const ray& r_in, const hit_record& rec, const ray& scattered)
    const override {
        // Delta material - PDF is infinite for exact direction, 0 otherwise
        return 0.0;
    }

    color evaluate_bsdf(const hit_record& rec, const vec3& wi, const vec3& wo) const override {
        // For delta materials, BSDF evaluation is only meaningful for exact directions
        // This is used for connection in BDPT, but glass is delta so connections skip it
        return color(0,0,0);
    }

    bool is_delta() const override {
        return true;  // Perfect specular material
    }

  private:

    static double fresnel_dielectric(double cos_theta_i, double ni, double nt) {
        // Ensure cos_theta_i is in valid range
        cos_theta_i = std::clamp(cos_theta_i, -1.0, 1.0);
        
        // Swap indices if coming from inside
        if (cos_theta_i < 0.0) {
            std::swap(ni, nt);
            cos_theta_i = -cos_theta_i;
        }

        double sin_theta_i = sqrt(fmax(0.0, 1.0 - cos_theta_i * cos_theta_i));
        double sin_theta_t = (ni / nt) * sin_theta_i;

        // Total internal reflection
        if (sin_theta_t >= 1.0) {
            return 1.0;
        }

        double cos_theta_t = sqrt(fmax(0.0, 1.0 - sin_theta_t * sin_theta_t));

        double r_parallel = ((nt * cos_theta_i) - (ni * cos_theta_t)) / 
                           ((nt * cos_theta_i) + (ni * cos_theta_t));
        double r_perpendicular = ((ni * cos_theta_i) - (nt * cos_theta_t)) / 
                                ((ni * cos_theta_i) + (nt * cos_theta_t));

        return (r_parallel * r_parallel + r_perpendicular * r_perpendicular) / 2.0;
    }

    double ior;
    color tint;
};

inline bool GlassMaterial::scatter(
    const ray& r_in,
    const hit_record& rec,
    scatter_record& srec
) const {
    vec3 unit_direction = unit_vector(r_in.direction());
    
    // Determine IOR based on which side we're entering from
    double ni = rec.front_face ? 1.0 : ior;
    double nt = rec.front_face ? ior : 1.0;
    double etai_over_etat = ni / nt;

    double cos_theta = dot(-unit_direction, rec.normal);
    double fresnel = fresnel_dielectric(cos_theta, ni, nt);

    // Sample reflection or refraction based on Fresnel
    vec3 scattered_dir;
    color attenuation = color(1.0, 1.0, 1.0);  // Default to white
    
    if (::random_double() < fresnel) {
        // Reflect - glass reflections are typically white/clear
        scattered_dir = reflect(unit_direction, rec.normal);
        attenuation = color(1.0, 1.0, 1.0);
    } else {
        // Refract
        scattered_dir = refract(unit_direction, rec.normal, etai_over_etat);
        
        // If refraction failed (total internal reflection), reflect instead
        // Check if the refracted direction is valid
        double sin_theta_t = (ni / nt) * sqrt(fmax(0.0, 1.0 - cos_theta * cos_theta));
        if (sin_theta_t >= 1.0 || scattered_dir.length_squared() < 0.01) {
            scattered_dir = reflect(unit_direction, rec.normal);
            attenuation = color(1.0, 1.0, 1.0);
        } else {
            // Apply tint for transmission (light passes through glass/liquid)
            // The tint represents the color that passes through
            attenuation = tint;
        }
    }

    // Offset ray origin to avoid self-intersection
    point3 offset_origin = rec.p + (0.001 * scattered_dir);
    srec.skip_pdf_ray = ray(offset_origin, scattered_dir, r_in.time());
    srec.attenuation = attenuation;
    srec.skip_pdf = true;
    
    return true;
}

#endif