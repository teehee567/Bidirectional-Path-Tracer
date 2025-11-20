#ifndef BXDF_UTILS_H
#define BXDF_UTILS_H

#include "bxdf.h"
#include "texture.h"

#include <algorithm>

inline double cos_theta(const vec3& v) {
    return v.z();
}

inline double abs_cos_theta(const vec3& v) {
    return std::fabs(v.z());
}

inline bool same_hemisphere(const vec3& a, const vec3& b) {
    return a.z() * b.z() > 0;
}

inline vec3 normalize_or_default(const vec3& v, const vec3& fallback = vec3(0,0,1)) {
    double len2 = v.length_squared();
    if (len2 <= 0.0)
        return fallback;
    return v / std::sqrt(len2);
}

inline vec3 tangent_component(const vec3& v, const vec3& n) {
    return v - dot(v, n) * n;
}

inline double luminance(const color& c) {
    return 0.2126 * c.x() + 0.7152 * c.y() + 0.0722 * c.z();
}

inline double spectrum_norm(const color& c) {
    return std::sqrt(std::max(0.0, dot(c, c)));
}

inline color saturate(const color& c) {
    return color(
        std::clamp(c.x(), 0.0, 1.0),
        std::clamp(c.y(), 0.0, 1.0),
        std::clamp(c.z(), 0.0, 1.0)
    );
}

inline color sample_texture(const shared_ptr<texture>& tex, const Vec2& texPos) {
    if (!tex)
        return color(1,1,1);
    return tex->value(texPos.u, texPos.v, texPos.p);
}

inline double sample_scalar(const shared_ptr<texture>& tex, const Vec2& texPos) {
    return luminance(sample_texture(tex, texPos));
}

#endif

