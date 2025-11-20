#pragma once

#include "main.h"

#include <cmath>
#include <glm/glm.hpp>

namespace utils {

inline float clamp(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

namespace sampler {
inline float random_double() {
    return static_cast<float>(::random_double());
}
} // namespace sampler

struct LocalBasis {
    glm::mat3 transform{1.0f};

    static LocalBasis from_normal(const glm::vec3& normal) {
        const glm::vec3 n = glm::normalize(normal);
        glm::vec3 tangent;
        if (std::abs(n.y) < 0.999f)
            tangent = glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), n));
        else
            tangent = glm::normalize(glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), n));
        glm::vec3 bitangent = glm::cross(n, tangent);

        LocalBasis basis;
        basis.transform = glm::mat3(tangent, n, bitangent);
        return basis;
    }
};

} // namespace utils

