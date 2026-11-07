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
/// @brief Dependency-free, numerically bounded axial beam response helpers.

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>

#if defined(_MSC_VER)
#   define ROR_BEAM_FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#   define ROR_BEAM_FORCE_INLINE inline __attribute__((always_inline))
#else
#   define ROR_BEAM_FORCE_INLINE inline
#endif

namespace RoR {
namespace BeamAxialResponse {

static_assert(sizeof(float) == sizeof(std::uint32_t), "32-bit IEEE-754 float required");
static_assert(sizeof(double) == sizeof(std::uint64_t), "64-bit IEEE-754 double required");
static_assert(std::numeric_limits<float>::is_iec559, "IEEE-754 float required");
static_assert(std::numeric_limits<double>::is_iec559, "IEEE-754 double required");

/// Squared lengths at or below this value have no reliable axial direction.
static const float MIN_LENGTH_SQUARED = 1.0e-12f;

/// Result of applying viscous damping along a beam axis.
struct DampingResult
{
    float force = 0.0f;                 //!< Signed axial force opposing relative velocity.
    float effective_coefficient = 0.0f; //!< Damping coefficient after the energy bound.
    bool  was_limited = false;
};

/// std::isfinite() can be optimized away by the game's release `-ffast-math`
/// flags. Inspecting the IEEE-754 exponent bits keeps the simulation guard
/// effective in those builds.
ROR_BEAM_FORCE_INLINE bool IsFinite(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
#if defined(__GNUC__) || defined(__clang__)
    // A finite-math-only build may fold a classification of a value whose
    // producing operation carries no-NaN/no-infinity flags; the empty asm
    // hides where the bits came from at no cost. MSVC's /fp:fast tracks no
    // such property through an integer copy, so it needs nothing.
    asm volatile("" : "+r"(bits));
#endif
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

ROR_BEAM_FORCE_INLINE bool IsFinite(double value)
{
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
#if defined(__GNUC__) || defined(__clang__)
    // A finite-math-only build may fold a classification of a value whose
    // producing operation carries no-NaN/no-infinity flags; the empty asm
    // hides where the bits came from at no cost. MSVC's /fp:fast tracks no
    // such property through an integer copy, so it needs nothing.
    asm volatile("" : "+r"(bits));
#endif
    return (bits & UINT64_C(0x7ff0000000000000)) !=
        UINT64_C(0x7ff0000000000000);
}

ROR_BEAM_FORCE_INLINE bool HasUsableLength(float squared_length)
{
    return IsFinite(squared_length) && squared_length > MIN_LENGTH_SQUARED;
}

ROR_BEAM_FORCE_INLINE bool HasUsableLength(double squared_length)
{
    return IsFinite(squared_length) &&
        squared_length > static_cast<double>(MIN_LENGTH_SQUARED);
}

enum class RestLengthDeformationDirection : std::uint8_t
{
    COMPRESSION = 0,
    EXPANSION
};

/// BeamNG's `deformLimitExpansion`: "Limits by how much of original length the
/// beam can deform in expansion", default matching `deformLimit`, "used to
/// avoid excessive stretching"
/// (<https://documentation.beamng.com/modding/vehicle/sections/beams/>).
///
/// It caps PERMANENT deformation and is independent of `beamDeform`, which is
/// the force in newtons at which permanent deformation begins. Authored values
/// are a multiple of the original length and sit above 1.0: 1.1 accounts for
/// 1032 of the occurrences across the three reference packages, with 1.5 and
/// 1.2 the only other in-range values seen.
///
/// `declared` is a flag rather than a zero/negative sentinel on the length.
/// Release builds compile this tree with -ffast-math, which folds the finite
/// and comparison guards a sentinel would rely on, so an undeclared ceiling
/// has to be distinguishable without arithmetic.
struct RestLengthExpansionCeiling
{
    bool declared = false;
    /// Absolute maximum rest length in metres, i.e. the authored ratio already
    /// multiplied by the beam's ORIGINAL rest length -- not the ratio itself,
    /// and not relative to the current (already deformed) length, so repeated
    /// yields cannot ratchet the limit outward.
    float maximum_rest_length = 0.0f;
};

struct RestLengthDeformationResult
{
    float next_rest_length = 0.0f;
    float applied_delta = 0.0f;
    float next_strength = 0.0f;
    bool accepted = false;
};

/// Applies one directional legacy plastic-deformation delta while preserving
/// the engine's positive rest-length floor. Damping can trip a yield test before
/// the geometric displacement reaches that direction's yield length, so a
/// compression event may receive a positive delta or an expansion event a
/// negative one. Those opposite-direction deltas are a force transient, not
/// plastic deformation, and must not mutate rest length or beam strength.
/// The update is transactional: invalid/non-finite input, opposite-direction
/// deformation, or non-finite tensile strength loss rejects the whole update
/// instead of being hidden by std::max under fast-math.
ROR_BEAM_FORCE_INLINE RestLengthDeformationResult ApplyRestLengthDeformation(
    float current_rest_length,
    float deformation_delta,
    float minimum_rest_length,
    RestLengthDeformationDirection direction,
    float current_strength,
    float spring,
    RestLengthExpansionCeiling expansion_ceiling = RestLengthExpansionCeiling())
{
    RestLengthDeformationResult result;
    result.next_rest_length = current_rest_length;
    result.next_strength = current_strength;
    if (!IsFinite(current_rest_length) || current_rest_length <= 0.0f ||
        !IsFinite(deformation_delta) ||
        !IsFinite(minimum_rest_length) || minimum_rest_length <= 0.0f ||
        !IsFinite(current_strength) || !IsFinite(spring) ||
        (direction != RestLengthDeformationDirection::COMPRESSION &&
            direction != RestLengthDeformationDirection::EXPANSION))
    {
        return result;
    }

    const bool correct_direction =
        direction == RestLengthDeformationDirection::COMPRESSION
            ? deformation_delta <= 0.0f
            : deformation_delta >= 0.0f;
    if (!correct_direction)
        return result;

    const float candidate = current_rest_length + deformation_delta;
    if (!IsFinite(candidate))
        return result;

    // Compression has always had a floor; expansion had no ceiling at all,
    // which is exactly the slot deformLimitExpansion fills. An undeclared or
    // non-finite ceiling leaves legacy behaviour untouched, and a ceiling at
    // or below the current rest length cannot pull the beam inward: the
    // clamp may only refuse further stretch, never reverse it.
    float expansion_candidate = candidate;
    if (direction == RestLengthDeformationDirection::EXPANSION &&
        expansion_ceiling.declared &&
        IsFinite(expansion_ceiling.maximum_rest_length) &&
        expansion_ceiling.maximum_rest_length > 0.0f)
    {
        const float ceiling = std::max(
            expansion_ceiling.maximum_rest_length, current_rest_length);
        if (expansion_candidate > ceiling)
            expansion_candidate = ceiling;
    }
    result.next_rest_length =
        direction == RestLengthDeformationDirection::COMPRESSION
            ? std::max(minimum_rest_length, candidate)
            : expansion_candidate;
    result.applied_delta =
        result.next_rest_length - current_rest_length;
    if (!IsFinite(result.applied_delta) ||
        (direction == RestLengthDeformationDirection::COMPRESSION &&
            result.applied_delta > 0.0f) ||
        (direction == RestLengthDeformationDirection::EXPANSION &&
            result.applied_delta < 0.0f))
    {
        result.next_rest_length = current_rest_length;
        result.applied_delta = 0.0f;
        return result;
    }

    if (direction == RestLengthDeformationDirection::EXPANSION)
    {
        const float strength_loss = result.applied_delta * spring;
        const float next_strength = current_strength - strength_loss;
        if (!IsFinite(strength_loss) || !IsFinite(next_strength))
        {
            result.next_rest_length = current_rest_length;
            result.applied_delta = 0.0f;
            return result;
        }
        result.next_strength = next_strength;
    }
    result.accepted = true;
    return result;
}

/// The force at which the legacy solver first examines a beam: the smallest of
/// either direction's plastic yield force and the beam's strength.
///
/// The solver tests `|force| > minmaxposnegstress` first and only inside that
/// gate asks whether the beam yields (against `maxposstress`/`maxnegstress`) or
/// breaks (against `strength`). A gate above the strength therefore makes the
/// strength unreachable: a beam spawned under the 400 kN legacy deform floor
/// with a 70 kN strength could not break below 400 kN. This is the same value
/// the solver re-derives after every plastic event, so establishing it at
/// spawn and at reset keeps one rule for the whole run.
ROR_BEAM_FORCE_INLINE float YieldOrBreakGate(
    float max_positive_stress,
    float max_negative_stress,
    float strength)
{
    return std::min(
        std::min(max_positive_stress, -max_negative_stress), strength);
}

/// The deform threshold a `set_beam_defaults` row spawns with, before its
/// deformation scale is applied.
struct LegacyDeformThreshold
{
    float threshold = 0.0f;
    /// An AUTHORED threshold below the legacy floor was raised to it because
    /// `enable_advanced_deformation` did not precede the row. Unauthored
    /// defaults never set this: nothing was overridden.
    bool raised_to_legacy_floor = false;
};

/// Resolves a deform threshold in the legacy order: an authored value is
/// raised to `legacy_deform_floor` unless advanced deformation is on, and any
/// value is then raised to `creak_floor` unless the row authored a plastic
/// coefficient that disables that floor. Unauthored rows use the legacy floor.
ROR_BEAM_FORCE_INLINE LegacyDeformThreshold ResolveLegacyDeformThreshold(
    bool user_defined,
    float authored_threshold,
    bool advanced_deformation,
    bool creak_floor_disabled,
    float legacy_deform_floor,
    float creak_floor)
{
    LegacyDeformThreshold result;
    result.threshold = legacy_deform_floor;
    float creak = creak_floor;
    if (user_defined)
    {
        result.threshold = authored_threshold;
        if (!advanced_deformation && authored_threshold < legacy_deform_floor)
        {
            result.threshold = legacy_deform_floor;
            result.raised_to_legacy_floor = true;
        }
        if (creak_floor_disabled)
        {
            creak = 0.0f;
        }
    }
    if (result.threshold < creak)
    {
        result.threshold = creak;
    }
    return result;
}

/// Returns a finite inverse mass. Extremely small positive masses saturate
/// instead of allowing their reciprocal to become infinity.
ROR_BEAM_FORCE_INLINE float InverseMass(float mass, bool movable)
{
    if (!movable || !IsFinite(mass) || mass <= 0.0f)
    {
        return 0.0f;
    }

    const float inverse_mass = 1.0f / mass;
    return IsFinite(inverse_mass)
        ? inverse_mass
        : std::numeric_limits<float>::max();
}

ROR_BEAM_FORCE_INLINE float EffectiveMass(
    float mass_1,
    float mass_2,
    bool movable_1,
    bool movable_2)
{
    if ((movable_1 && (!IsFinite(mass_1) || mass_1 <= 0.0f)) ||
        (movable_2 && (!IsFinite(mass_2) || mass_2 <= 0.0f)))
    {
        return 0.0f;
    }

    if (movable_1 && movable_2)
    {
        const float smaller_mass = std::min(mass_1, mass_2);
        const float larger_mass = std::max(mass_1, mass_2);
        return smaller_mass / (1.0f + smaller_mass / larger_mass);
    }
    if (movable_1)
    {
        return mass_1;
    }
    if (movable_2)
    {
        return mass_2;
    }
    return 0.0f;
}

/// Computes a viscous damping force that cannot reverse the two endpoints'
/// relative axial velocity in a single fixed step.
///
/// For inverse effective mass `w`, unconstrained damping is `F = -d*v`.
/// Bounding `d` to `1 / (w*dt)` guarantees:
///
///     v_next = v + w*dt*F
///
/// reaches zero at most, instead of crossing zero and adding kinetic energy.
/// An immovable endpoint contributes zero inverse mass. Mirrored half-beam
/// pairs must include both endpoints in every call so their two forces share
/// this bound. Mark an endpoint immovable only when it is genuinely kinematic.
ROR_BEAM_FORCE_INLINE DampingResult ComputeDamping(
    float relative_velocity,
    float damping_coefficient,
    float time_step,
    float mass_1,
    float mass_2,
    bool movable_1,
    bool movable_2)
{
    DampingResult result;

    if (!IsFinite(relative_velocity) ||
        !IsFinite(damping_coefficient) ||
        !IsFinite(time_step) ||
        (movable_1 && (!IsFinite(mass_1) || mass_1 <= 0.0f)) ||
        (movable_2 && (!IsFinite(mass_2) || mass_2 <= 0.0f)) ||
        damping_coefficient <= 0.0f ||
        time_step <= 0.0f)
    {
        return result;
    }

    if (!movable_1 && !movable_2)
    {
        return result;
    }

    // Most vehicle beams are comfortably below their exact damping limit.
    // The reduced mass of two movable endpoints is always at least half the
    // smaller endpoint mass, which gives this division-free conservative
    // fast path. A single movable endpoint uses its full mass.
    const float conservative_effective_mass =
        movable_1 && movable_2
            ? 0.5f * std::min(mass_1, mass_2)
            : (movable_1 ? mass_1 : mass_2);
    const float requested_damping_mass = damping_coefficient * time_step;
    if (IsFinite(requested_damping_mass) &&
        requested_damping_mass <= conservative_effective_mass)
    {
        result.effective_coefficient = damping_coefficient;
        result.force = -damping_coefficient * relative_velocity;
        if (!IsFinite(result.force))
        {
            return DampingResult();
        }
        return result;
    }

    const float effective_mass =
        EffectiveMass(mass_1, mass_2, movable_1, movable_2);
    if (effective_mass <= 0.0f)
    {
        return result;
    }

    const float maximum_coefficient = effective_mass / time_step;
    if (!IsFinite(maximum_coefficient) || maximum_coefficient <= 0.0f)
    {
        return result;
    }

    result.effective_coefficient =
        std::min(damping_coefficient, maximum_coefficient);
    result.was_limited = result.effective_coefficient < damping_coefficient;
    result.force = -result.effective_coefficient * relative_velocity;

    // Avoid propagating an overflow into every node connected to this beam.
    if (!IsFinite(result.force))
    {
        return DampingResult();
    }

    return result;
}

} // namespace BeamAxialResponse
} // namespace RoR

#undef ROR_BEAM_FORCE_INLINE
