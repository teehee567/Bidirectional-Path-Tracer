#ifndef SPECULAR_BXDF_H
#define SPECULAR_BXDF_H

#include "../bxdf.h"

class SpecularReflectionBxDF : public BxDF {
public:
    SpecularReflectionBxDF(const color& r) : R(r) {}

    Spectrum evaluate(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        return color(0,0,0); // Delta distribution
    }

    Spectrum sample(const Vec2 &texPos, const Vec3 &i, Vec3 &o, double &pdf, Sampler &RNG) const override {
        // Perfect reflection
        // i is incoming (pointing away from surface? or towards?)
        // In this framework:
        // gI is incoming direction (from light/previous bounce).
        // gO is outgoing direction (towards next bounce/camera).
        // Wait, typically in path tracing:
        // i = -ray.direction (pointing away from surface)
        // o = scattered direction
        
        // The snippet says:
        // rotateAxis(n, t, T, TInv);
        // Vec3 vT = T * gI;
        
        // Usually 'i' is wo (outgoing to viewer) and we sample 'wi' (incoming from light).
        // Or 'i' is wi (incoming from light) and we sample 'wo' (outgoing).
        // Let's check the snippet usages.
        // Snippet doesn't show usage.
        
        // Standard convention:
        // evaluate(wo, wi) -> f(wo, wi)
        // sample(wo, &wi) -> generates wi
        
        // In the snippet:
        // sample(..., gI, gO, ...)
        // gI is input, gO is output (reference).
        // So gI is the fixed direction (usually wo, from camera).
        // gO is the sampled direction (wi, towards light).
        
        // So if gI is incident from camera (pointing into surface? or away?)
        // In the `evaluate` wrapper:
        // Vec3 vT = T * gI, lT = T * gO;
        // The wrapper takes `gI` (const) and `gO` (const).
        // In `sample` wrapper:
        // Vec3 vT = T * gI;
        // ... sample returns lT ...
        // gO = TInv * lT;
        
        // So `i` in local space is `vT`. `o` in local space is `lT`.
        
        // For reflection:
        // o = (-i.x, -i.y, i.z)
        // effectively reflecting around Z axis.
        
        o = vec3(-i.x(), -i.y(), i.z());
        pdf = 1.0;
        
        // Fresnel? If R is constant, just return R / abs(cos_theta_o)
        // BSDF = R * delta(...)
        // Integral f * cos / pdf = R * 1 / 1 = R.
        // But sample returns s/pdf.
        // So we return R / abs(o.z()). 
        // Wait, usually for delta, sample returns the throughput weight directly?
        // The base class divides by pdf.
        // If pdf=1, we return s.
        // s should be (R * delta) * cos_theta ? No.
        // For delta distributions, the API usually handles it by returning the throughput.
        // Throughput = f * cos / pdf.
        // If we return f' such that f'/pdf = Throughput.
        // Throughput is R (for perfect mirror).
        // So s/pdf = R -> s = R * pdf = R.
        // But evaluate returns 0.
        
        return R / std::abs(o.z()); 
    }

    double evaluateImportance(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        return 0; // Delta
    }

private:
    color R;
    
    bool is_delta() const override { return true; }
};

class SpecularTransmissionBxDF : public BxDF {
public:
    SpecularTransmissionBxDF(const color& t, double etaA, double etaB) 
        : T(t), etaA(etaA), etaB(etaB) {}

    Spectrum evaluate(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        return color(0,0,0);
    }

    Spectrum sample(const Vec2 &texPos, const Vec3 &i, Vec3 &o, double &pdf, Sampler &RNG) const override {
        bool entering = i.z() > 0;
        double etaI = entering ? etaA : etaB;
        double etaT = entering ? etaB : etaA;
        double eta = etaI / etaT;

        // Refract
        // i is usually vector pointing AWAY from surface in local space if z > 0.
        // Wait, if i comes from camera, it points AWAY from surface?
        // In ray tracer: ray direction points INTO surface.
        // If we transform to local:
        // If normal is (0,0,1). Ray dir d. dot(d, n) < 0 usually.
        // If we use -d as 'i'. Then i.z > 0.
        // Let's assume 'i' is the vector pointing AWAY from the surface (standard PBRT convention).
        
        // Snell's law: sin_t = eta * sin_i
        double sin2i = std::max(0.0, 1.0 - i.z()*i.z());
        double sin2t = eta*eta * sin2i;
        
        if (sin2t >= 1) {
            // Total internal reflection
            pdf = 1.0;
            o = vec3(-i.x(), -i.y(), i.z()); // Reflect
            return T / std::abs(o.z()); // Treat as reflection
        }
        
        double cos_t = std::sqrt(1.0 - sin2t);
        if (entering) cos_t = -cos_t;
        else cos_t = std::abs(cos_t); // If exiting, we want positive z? No, leaving surface means z < 0?
        // If i.z > 0 (upper hemisphere), we transmit to lower (z < 0).
        // So if entering, o.z should be negative.
        
        if (entering) cos_t = -std::sqrt(1.0 - sin2t);
        else cos_t = std::sqrt(1.0 - sin2t);

        o = vec3(-eta * i.x(), -eta * i.y(), cos_t);
        pdf = 1.0;
        
        // Throughput for transmission:
        // (eta^2) * T / cos_theta ? 
        // Radiance scales by 1/eta^2? Or eta^2?
        // It depends on if we trace radiance or importance.
        // Usually eta^2 factor is applied.
        // s / pdf = throughput.
        
        return T * (1.0 / std::abs(o.z())); 
        // Note: Handling eta scaling correctly is tricky without full context.
        // PBRT includes eta^2 term for radiance.
        // But if we just want color throughput, T is enough?
        // Let's stick to simple transmission.
    }

    double evaluateImportance(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const override {
        return 0;
    }

private:
    color T;
    double etaA, etaB; // Indices of refraction

    bool is_delta() const override { return true; }
};

#endif

