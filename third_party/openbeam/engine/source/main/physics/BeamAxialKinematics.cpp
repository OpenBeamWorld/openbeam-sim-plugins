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

#include "BeamAxialKinematics.h"

#include "BeamAxialResponse.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace RoR {
namespace BeamAxialKinematics {
namespace {

Result Failure(Error error)
{
    Result result;
    result.error = error;
    return result;
}

bool IsFiniteVector(const std::array<double, 3>& value)
{
    return
        BeamAxialResponse::IsFinite(value[0]) &&
        BeamAxialResponse::IsFinite(value[1]) &&
        BeamAxialResponse::IsFinite(value[2]);
}

} // namespace

Result Compute(const Input& input)
{
    if (!IsFiniteVector(input.endpoint_1_position_m) ||
        !IsFiniteVector(input.endpoint_2_position_m) ||
        !IsFiniteVector(input.endpoint_1_velocity_mps) ||
        !IsFiniteVector(input.endpoint_2_velocity_mps))
    {
        return Failure(Error::NONFINITE_INPUT);
    }

    const std::array<double, 3> displacement = {{
        input.endpoint_1_position_m[0] - input.endpoint_2_position_m[0],
        input.endpoint_1_position_m[1] - input.endpoint_2_position_m[1],
        input.endpoint_1_position_m[2] - input.endpoint_2_position_m[2]
    }};
    const std::array<double, 3> relative_velocity = {{
        input.endpoint_1_velocity_mps[0] - input.endpoint_2_velocity_mps[0],
        input.endpoint_1_velocity_mps[1] - input.endpoint_2_velocity_mps[1],
        input.endpoint_1_velocity_mps[2] - input.endpoint_2_velocity_mps[2]
    }};
    if (!IsFiniteVector(displacement) || !IsFiniteVector(relative_velocity))
        return Failure(Error::NUMERIC_OVERFLOW);

    const double scale = std::max(
        std::abs(displacement[0]),
        std::max(
            std::abs(displacement[1]),
            std::abs(displacement[2])));
    if (!BeamAxialResponse::IsFinite(scale))
        return Failure(Error::NONFINITE_INPUT);
    if (scale <= 0.0)
        return Failure(Error::INVALID_LENGTH);

    const std::array<double, 3> scaled = {{
        displacement[0] / scale,
        displacement[1] / scale,
        displacement[2] / scale
    }};
    const double scaled_length_squared =
        scaled[0] * scaled[0] +
        scaled[1] * scaled[1] +
        scaled[2] * scaled[2];
    if (!BeamAxialResponse::IsFinite(scaled_length_squared) ||
        scaled_length_squared <= 0.0)
    {
        return Failure(Error::NUMERIC_OVERFLOW);
    }

    const double inverse_scaled_length =
        1.0 / std::sqrt(scaled_length_squared);
    const double current_length = scale / inverse_scaled_length;
    if (!BeamAxialResponse::IsFinite(inverse_scaled_length) ||
        inverse_scaled_length <= 0.0 ||
        !BeamAxialResponse::IsFinite(current_length))
    {
        return Failure(Error::NUMERIC_OVERFLOW);
    }

    const double minimum_length = std::sqrt(
        static_cast<double>(BeamAxialResponse::MIN_LENGTH_SQUARED));
    if (current_length <= minimum_length)
        return Failure(Error::INVALID_LENGTH);

    const std::array<double, 3> direction = {{
        scaled[0] * inverse_scaled_length,
        scaled[1] * inverse_scaled_length,
        scaled[2] * inverse_scaled_length
    }};
    if (!IsFiniteVector(direction))
        return Failure(Error::NUMERIC_OVERFLOW);

    const double axial_relative_velocity =
        relative_velocity[0] * direction[0] +
        relative_velocity[1] * direction[1] +
        relative_velocity[2] * direction[2];
    if (!BeamAxialResponse::IsFinite(axial_relative_velocity))
        return Failure(Error::NUMERIC_OVERFLOW);

    const double runtime_maximum =
        static_cast<double>(std::numeric_limits<float>::max());
    if (current_length > runtime_maximum ||
        std::abs(axial_relative_velocity) > runtime_maximum)
    {
        return Failure(Error::NUMERIC_OVERFLOW);
    }

    const float runtime_length = static_cast<float>(current_length);
    const float runtime_velocity =
        static_cast<float>(axial_relative_velocity);
    if (!BeamAxialResponse::IsFinite(runtime_length) ||
        !BeamAxialResponse::IsFinite(runtime_velocity))
    {
        return Failure(Error::NUMERIC_OVERFLOW);
    }

    Result result;
    result.current_length_m = current_length;
    result.axial_relative_velocity_mps = axial_relative_velocity;
    result.unit_direction = direction;
    result.current_length_runtime_m = runtime_length;
    result.axial_relative_velocity_runtime_mps = runtime_velocity;
    return result;
}

} // namespace BeamAxialKinematics
} // namespace RoR
