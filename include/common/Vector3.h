#pragma once

#include <cmath>

namespace vbt
{
    struct Vector3
    {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};

        constexpr Vector3() = default;

        constexpr Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

        constexpr float magnitudeSquared() const
        {
            return x * x + y * y + z * z;
        }

        constexpr float magnitude() const 
        {
            return std::sqrt(magnitudeSquared());
        }
    };
}