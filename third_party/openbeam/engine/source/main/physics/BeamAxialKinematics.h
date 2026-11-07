/*
    This source file is part of Rigs of Rods
    For more information, see http://www.rigsofrods.org/

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

/// @file
/// @brief Accurate, dependency-free axial geometry for calibrated beams.

#pragma once

#include <array>

namespace RoR {
namespace BeamAxialKinematics {

/// Geometry failures are explicit so an opted-in calibrated beam can latch
/// the corresponding production adapter fault instead of falling back to the
/// legacy force law.
enum class Error
{
    NONE,
    NONFINITE_INPUT,
    INVALID_LENGTH,
    NUMERIC_OVERFLOW
};

struct Input
{
    std::array<double, 3> endpoint_1_position_m = {{0.0, 0.0, 0.0}};
    std::array<double, 3> endpoint_2_position_m = {{0.0, 0.0, 0.0}};
    std::array<double, 3> endpoint_1_velocity_mps = {{0.0, 0.0, 0.0}};
    std::array<double, 3> endpoint_2_velocity_mps = {{0.0, 0.0, 0.0}};
};

struct Result
{
    Error error = Error::NONE;
    double current_length_m = 0.0;
    double axial_relative_velocity_mps = 0.0;
    std::array<double, 3> unit_direction = {{0.0, 0.0, 0.0}};

    /// Checked runtime values used by Actor's float solver boundary.
    float current_length_runtime_m = 0.0f;
    float axial_relative_velocity_runtime_mps = 0.0f;

    bool IsValid() const { return error == Error::NONE; }
};

/// Computes length, a true unit direction, and the velocity projection from
/// the same binary64 normalization. The implementation is compiled outside
/// the game's global fast-math mode so its finite/overflow contract remains
/// valid in optimized builds. Publication is transactional: failed results
/// contain only their error code and zero-valued outputs.
Result Compute(const Input& input);

} // namespace BeamAxialKinematics
} // namespace RoR
