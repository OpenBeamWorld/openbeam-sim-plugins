/*
    This source file is part of Rigs of Rods

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

/// @file JBeamCoordinateTransform.h
/// @brief Exact, dependency-free coordinate conversion at the JBeam boundary.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace RoR {
namespace BeamNG {

/// A position expressed in whichever coordinate system names the API call.
struct JBeamPoint3
{
    double x;
    double y;
    double z;

    JBeamPoint3() :
        x(0.0),
        y(0.0),
        z(0.0)
    {
    }

    JBeamPoint3(double x_value, double y_value, double z_value) :
        x(x_value),
        y(y_value),
        z(z_value)
    {
    }
};

/// A direction or displacement expressed in the named coordinate system.
/// This is intentionally a distinct type so point and vector conversions
/// remain explicit when translation is added to a higher-level import stage.
struct JBeamVector3
{
    double x;
    double y;
    double z;

    JBeamVector3() :
        x(0.0),
        y(0.0),
        z(0.0)
    {
    }

    JBeamVector3(double x_value, double y_value, double z_value) :
        x(x_value),
        y(y_value),
        z(z_value)
    {
    }
};

enum class JBeamTransformHandedness
{
    PRESERVED,
    REVERSED
};

/// BeamNG vehicle axes are +X left, +Y backward, +Z up. RoR vehicle axes are
/// +X backward, +Y up, +Z left. The corresponding matrix is:
///
///     [ 0 1 0 ]
///     [ 0 0 1 ]
///     [ 1 0 0 ]
///
/// It is a proper orthogonal transform with determinant +1. Consequently it
/// preserves lengths, angles, cross products, triangle winding, and
/// right-handed orientation.
inline double GetBeamNGToRoRTransformDeterminant()
{
    return 1.0;
}

inline double GetRoRToBeamNGTransformDeterminant()
{
    return 1.0;
}

inline JBeamTransformHandedness GetBeamNGToRoRTransformHandedness()
{
    return JBeamTransformHandedness::PRESERVED;
}

inline JBeamTransformHandedness GetRoRToBeamNGTransformHandedness()
{
    return JBeamTransformHandedness::PRESERVED;
}

namespace Detail {

/// std::isfinite can be folded under aggressive finite-math assumptions.
/// Inspecting the IEEE-754 exponent bits keeps the untrusted-data boundary
/// effective in fast-math builds as well.
inline bool IsFiniteBinary64(double value)
{
    static_assert(sizeof(double) == sizeof(std::uint64_t),
        "JBeam coordinate conversion requires a binary64 double");
    static_assert(std::numeric_limits<double>::is_iec559,
        "JBeam coordinate conversion requires IEC 60559 doubles");

    // Volatile byte reads are deliberate. Some compilers otherwise reason
    // from -ffinite-math-only that every double argument is finite and fold
    // even an apparently bitwise check to true.
    std::uint64_t bits = 0U;
    const volatile unsigned char* const source_bytes =
        reinterpret_cast<const volatile unsigned char*>(&value);
    unsigned char* const destination_bytes =
        reinterpret_cast<unsigned char*>(&bits);
    for (std::size_t byte_index = 0U;
         byte_index < sizeof(bits);
         ++byte_index)
    {
        destination_bytes[byte_index] = source_bytes[byte_index];
    }
    const std::uint64_t exponent_mask =
        UINT64_C(0x7ff0000000000000);
    return (bits & exponent_mask) != exponent_mask;
}

inline bool StoreFinitePoint(
    double x,
    double y,
    double z,
    JBeamPoint3* output)
{
    if (output == NULL)
    {
        return false;
    }

    // Reset first so every rejected input has one deterministic, inert result.
    // x/y/z are value parameters, making in-place conversion safe.
    *output = JBeamPoint3();
    if (!IsFiniteBinary64(x) ||
        !IsFiniteBinary64(y) ||
        !IsFiniteBinary64(z))
    {
        return false;
    }

    *output = JBeamPoint3(x, y, z);
    return true;
}

inline bool StoreFiniteVector(
    double x,
    double y,
    double z,
    JBeamVector3* output)
{
    if (output == NULL)
    {
        return false;
    }

    *output = JBeamVector3();
    if (!IsFiniteBinary64(x) ||
        !IsFiniteBinary64(y) ||
        !IsFiniteBinary64(z))
    {
        return false;
    }

    *output = JBeamVector3(x, y, z);
    return true;
}

} // namespace Detail

inline bool IsFiniteJBeamPoint(const JBeamPoint3& point)
{
    return Detail::IsFiniteBinary64(point.x) &&
        Detail::IsFiniteBinary64(point.y) &&
        Detail::IsFiniteBinary64(point.z);
}

inline bool IsFiniteJBeamVector(const JBeamVector3& vector)
{
    return Detail::IsFiniteBinary64(vector.x) &&
        Detail::IsFiniteBinary64(vector.y) &&
        Detail::IsFiniteBinary64(vector.z);
}

/// Maps (x, y, z) in BeamNG to (y, z, x) in RoR.
/// On invalid input, returns false and resets output to (0, 0, 0).
inline bool TryTransformBeamNGPointToRoR(
    const JBeamPoint3& beamng_point,
    JBeamPoint3* ror_point)
{
    return Detail::StoreFinitePoint(
        beamng_point.y,
        beamng_point.z,
        beamng_point.x,
        ror_point);
}

/// Maps (x, y, z) in RoR to (z, x, y) in BeamNG.
/// On invalid input, returns false and resets output to (0, 0, 0).
inline bool TryTransformRoRPointToBeamNG(
    const JBeamPoint3& ror_point,
    JBeamPoint3* beamng_point)
{
    return Detail::StoreFinitePoint(
        ror_point.z,
        ror_point.x,
        ror_point.y,
        beamng_point);
}

/// Maps (x, y, z) in BeamNG to (y, z, x) in RoR.
/// On invalid input, returns false and resets output to (0, 0, 0).
inline bool TryTransformBeamNGVectorToRoR(
    const JBeamVector3& beamng_vector,
    JBeamVector3* ror_vector)
{
    return Detail::StoreFiniteVector(
        beamng_vector.y,
        beamng_vector.z,
        beamng_vector.x,
        ror_vector);
}

/// Maps (x, y, z) in RoR to (z, x, y) in BeamNG.
/// On invalid input, returns false and resets output to (0, 0, 0).
inline bool TryTransformRoRVectorToBeamNG(
    const JBeamVector3& ror_vector,
    JBeamVector3* beamng_vector)
{
    return Detail::StoreFiniteVector(
        ror_vector.z,
        ror_vector.x,
        ror_vector.y,
        beamng_vector);
}

/// Validates one row-major proper orthonormal 3x3 rotation without relying on
/// std::isfinite at an untrusted-data boundary.
inline bool IsProperOrthonormalRotationMatrix(
    const std::array<double, 9U>& matrix,
    double tolerance)
{
    if (!Detail::IsFiniteBinary64(tolerance) || tolerance <= 0.0)
    {
        return false;
    }
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        double row_length_squared = 0.0;
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            const double value = matrix[row * 3U + column];
            if (!Detail::IsFiniteBinary64(value))
            {
                return false;
            }
            row_length_squared += value * value;
        }
        if (!Detail::IsFiniteBinary64(row_length_squared) ||
            std::fabs(row_length_squared - 1.0) > tolerance)
        {
            return false;
        }
        for (std::size_t other_row = row + 1U;
             other_row < 3U;
             ++other_row)
        {
            double dot = 0.0;
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                dot += matrix[row * 3U + column] *
                    matrix[other_row * 3U + column];
            }
            if (!Detail::IsFiniteBinary64(dot) ||
                std::fabs(dot) > tolerance)
            {
                return false;
            }
        }
    }
    const double determinant =
        matrix[0U] * (matrix[4U] * matrix[8U] -
            matrix[5U] * matrix[7U]) -
        matrix[1U] * (matrix[3U] * matrix[8U] -
            matrix[5U] * matrix[6U]) +
        matrix[2U] * (matrix[3U] * matrix[7U] -
            matrix[4U] * matrix[6U]);
    return Detail::IsFiniteBinary64(determinant) &&
        std::fabs(determinant - 1.0) <= tolerance;
}

/// Maps a row-major BeamNG rotation to RoR as P * R * transpose(P), where P
/// is the same proper axis permutation used for points and vectors.
inline bool TryTransformBeamNGRotationMatrixToRoR(
    const std::array<double, 9U>& beamng,
    std::array<double, 9U>* ror)
{
    if (ror == NULL)
    {
        return false;
    }
    *ror = std::array<double, 9U>{{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    for (std::size_t index = 0U; index < beamng.size(); ++index)
    {
        if (!Detail::IsFiniteBinary64(beamng[index]))
        {
            return false;
        }
    }
    const std::array<std::size_t, 3U> permutation{{1U, 2U, 0U}};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            (*ror)[row * 3U + column] =
                beamng[permutation[row] * 3U + permutation[column]];
        }
    }
    return true;
}

/// Maps a BeamNG/Torque serialized column-major rotationMatrix to RoR.
///
/// The JSON array stores matrix columns consecutively even though the
/// in-memory operation is conventionally described as a row-major matrix.
/// Transpose while applying the same proper axis permutation used for points
/// and vectors. Keeping this separate from the row-major helper prevents file
/// serialization layout from leaking into mathematical callers.
inline bool TryTransformBeamNGSerializedRotationMatrixToRoR(
    const std::array<double, 9U>& beamng_serialized,
    std::array<double, 9U>* ror)
{
    if (ror == NULL)
    {
        return false;
    }
    *ror = std::array<double, 9U>{{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    for (std::size_t index = 0U;
         index < beamng_serialized.size(); ++index)
    {
        if (!Detail::IsFiniteBinary64(beamng_serialized[index]))
        {
            return false;
        }
    }
    const std::array<std::size_t, 3U> permutation{{1U, 2U, 0U}};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            (*ror)[row * 3U + column] =
                beamng_serialized[
                    permutation[column] * 3U + permutation[row]];
        }
    }
    return true;
}

} // namespace BeamNG
} // namespace RoR
