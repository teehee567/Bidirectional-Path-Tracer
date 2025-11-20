#ifndef BXDF_MATERIAL_H
#define BXDF_MATERIAL_H

#include "material.h"
#include "bxdf.h"
#include "bxdfs/lambertian_bxdf.h"
#include "bxdfs/specular_bxdf.h"
#include "bxdfs/biggx_bxdf.h"
#include "bxdfs/ggx_bxdf.h"
#include "bxdfs/mtl_ggx_bxdf.h"
#include "bxdfs/mtl_ext_ggx_bxdf.h"

class BxDFPdf : public pdf {
  public:
    BxDFPdf(shared_ptr<BxDF> bxdf, const vec3& n, const vec3& i, const Vec2& tex)
      : bxdf(bxdf), n(n), i(unit_vector(i)), tex(tex) {}

    double value(const vec3& direction) const override {
        return bxdf->evaluateImportance(tex, n, vec3(0,0,0), i, unit_vector(direction));
    }

    vec3 generate() const override {
        vec3 o;
        Sampler sampler;
        bxdf->sample(tex, n, vec3(0,0,0), i, o, sampler);
        return unit_vector(o);
    }

  private:
    shared_ptr<BxDF> bxdf;
    vec3 n;
    vec3 i;
    Vec2 tex;
};

class BxDFMaterial : public material {
  public:
    explicit BxDFMaterial(shared_ptr<BxDF> bxdf) : bxdf(bxdf) {}

    bool scatter(const ray& r_in, const hit_record& rec, scatter_record& srec) const override {
        Vec2 tex(rec.u, rec.v, rec.p);
        vec3 gI = unit_vector(-r_in.direction());

        if (bxdf->is_delta()) {
            srec.skip_pdf = true;
            Sampler sampler;
            vec3 o;
            srec.attenuation = bxdf->sample(tex, rec.normal, vec3(0,0,0), gI, o, sampler);
            srec.skip_pdf_ray = ray(rec.p, o, r_in.time());
            return true;
        }

        srec.skip_pdf = false;
        srec.pdf_ptr = make_shared<BxDFPdf>(bxdf, rec.normal, gI, tex);
        srec.attenuation = color(1,1,1);
        return true;
    }

    double scattering_pdf(const ray& r_in, const hit_record& rec, const ray& scattered) const override {
        return bxdf->evaluateImportance(Vec2(rec.u, rec.v, rec.p), rec.normal, vec3(0,0,0),
                                        unit_vector(-r_in.direction()), unit_vector(scattered.direction()));
    }

    color evaluate_bsdf(const hit_record& rec, const vec3& wi, const vec3& wo) const override {
        return bxdf->evaluate(Vec2(rec.u, rec.v, rec.p), rec.normal, vec3(0,0,0), unit_vector(wi),
                              unit_vector(wo));
    }

    bool is_delta() const override { return bxdf->is_delta(); }

  private:
    shared_ptr<BxDF> bxdf;
};

#endif

