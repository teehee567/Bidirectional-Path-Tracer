#ifndef BXDF_H
#define BXDF_H

#include "main.h"
#include "vec3.h"
#include "color.h"
#include "onb.h"

#include <cmath>

using Spectrum = color;
using Vec3 = vec3;

struct Vec2 {
    double u, v;
    point3 p;

    Vec2(double u=0, double v=0, const point3& p = point3(0,0,0)) : u(u), v(v), p(p) {}
};

class Sampler {
public:
    virtual double get1D() { return random_double(); }
    virtual Vec2 get2D() { return Vec2(random_double(), random_double()); }
    virtual double rand() { return random_double(); }
    virtual double rand(double min, double max) { return random_double(min, max); }
    virtual vec3 hemisphere() {
        // Uniform hemisphere oriented around +Z in local space
        double z = rand();
        double r = std::sqrt(std::max(0.0, 1.0 - z*z));
        double phi = 2 * pi * rand();
        return vec3(r * std::cos(phi), r * std::sin(phi), z);
    }
    virtual ~Sampler() = default;
};

struct Mat3 {
    vec3 rows[3];
    
    Mat3() {}
    Mat3(vec3 r0, vec3 r1, vec3 r2) {
        rows[0] = r0; rows[1] = r1; rows[2] = r2;
    }

    vec3 operator*(const vec3& v) const {
        return vec3(
            dot(rows[0], v),
            dot(rows[1], v),
            dot(rows[2], v)
        );
    }
};

inline void rotateAxis(const Vec3& n, const Vec3& t, Mat3& T, Mat3& TInv) {
    // Construct basis from n (normal) and t (tangent)
    // Assuming n is normalized. 
    // If t is zero or not orthogonal, we might need to fix it.
    // Standard ONB construction if t is not provided?
    // The snippet assumes t is provided.
    
    vec3 w = unit_vector(n);
    vec3 u = unit_vector(t); 
    // If t is not orthogonal to n, we should project it? 
    // Or just use onb helper if t is dummy.
    
    // Check if t is valid (non-zero).
    if (t.length_squared() < 1e-8) {
        onb basis(w);
        u = basis.u();
    } else {
         // Ensure u is orthogonal to w
         u = unit_vector(cross(cross(w, u), w));
    }
    vec3 v = cross(w, u);

    // T transforms from Global to Local (where n is Z axis usually)
    // or Local to Global?
    // Usually BxDFs operate in Local space where normal is (0,0,1).
    // The snippet says:
    // Vec3 vT = T * gI; // global Incoming -> transformed (Local?)
    // So T is Global -> Local (Rotation matrix rows are the basis vectors in global coords? No.)
    // If rows are basis vectors U, V, W. Then M * v = (U.v, V.v, W.v).
    // This projects v onto U, V, W. So vT is in the frame (U, V, W).
    // So T = [u, v, w]^T.
    
    T = Mat3(u, v, w);
    
    // TInv transforms Local -> Global. 
    // Columns are basis vectors.
    // Or simply transpose of T.
    // TInv * lT = (u*lT.x + v*lT.y + w*lT.z)
    // So we need column-based multiplication or construct Mat3 differently.
    // My Mat3 * vec3 is row-based dot product.
    // So for TInv to map local (x,y,z) to x*u + y*v + z*w:
    // It should have rows... wait.
    // If T is orthogonal, TInv = T^T.
    // T = [ ux uy uz ]
    //     [ vx vy vz ]
    //     [ wx wy wz ]
    // T * g = (u.g, v.g, w.g) -> (local_x, local_y, local_z). Correct.
    
    // TInv = T^T
    // TInv = [ ux vx wx ]
    //        [ uy vy wy ]
    //        [ uz vz wz ]
    // TInv * l = (ux*lx + vx*ly + wx*lz, ...) = lx*u + ly*v + lz*w. Correct.
    
    TInv = Mat3(
        vec3(u.x(), v.x(), w.x()),
        vec3(u.y(), v.y(), w.y()),
        vec3(u.z(), v.z(), w.z())
    );
}

class BxDF {
public:
    virtual ~BxDF() = default;

    Spectrum evaluate(const Vec2 &texPos, const Vec3 &n, const Vec3 &t, const Vec3 &gI, const Vec3 &gO) const;
    Spectrum sample(const Vec2 &texPos, const Vec3 &n, const Vec3 &t, const Vec3 &gI, Vec3 &gO, Sampler &RNG) const;
    double evaluateImportance(const Vec2 &texPos, const Vec3 &n, const Vec3 &t, const Vec3 &gI, const Vec3 &gO) const;

protected:
    virtual Spectrum evaluate(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const {
        return Spectrum(0,0,0);
        // throw std::invalid_argument("NotImplementedError"); 
    }

    // Returns BSDF value, sets outgoing direction 'o', and pdf
    virtual Spectrum sample(const Vec2 &texPos, const Vec3 &i, Vec3 &o, double &pdf, Sampler &RNG) const {
        return Spectrum(0,0,0);
        // throw std::invalid_argument("NotImplementedError");
    }

    virtual double evaluateImportance(const Vec2 &texPos, const Vec3 &i, const Vec3 &o) const {
        return 0.0;
        // throw std::invalid_argument("NotImplementedError");
    }

public:
    virtual bool is_delta() const { return false; }
};

// Implementation of base class non-virtual methods (template pattern)

inline Spectrum BxDF::evaluate(const Vec2 &texPos, const Vec3 &n, const Vec3 &t, const Vec3 &gI, const Vec3 &gO) const {
    Mat3 T, TInv;
    rotateAxis(n, t, T, TInv);
    Vec3 vT = T * gI, lT = T * gO;
    return evaluate(texPos, vT, lT);
}

inline Spectrum BxDF::sample(const Vec2 &texPos, const Vec3 &n, const Vec3 &t, const Vec3 &gI, Vec3 &gO, Sampler &RNG) const {
    Mat3 T, TInv;
    rotateAxis(n, t, T, TInv);
    Vec3 vT = T * gI, lT;
    double pdf = 0;
    Spectrum s = sample(texPos, vT, lT, pdf, RNG);
    gO = TInv * lT;
    if (pdf > 0)
        return s / pdf; // The snippet returns s / pdf. This is often the weight (BSDF value / pdf).
    else 
        return Spectrum(0,0,0);
}

inline double BxDF::evaluateImportance(const Vec2 &texPos, const Vec3 &n, const Vec3 &t, const Vec3 &gI, const Vec3 &gO) const {
    Mat3 T, TInv;
    rotateAxis(n, t, T, TInv);
    Vec3 vT = T * gI, lT = T * gO;
    return evaluateImportance(texPos, vT, lT);
}

#endif

