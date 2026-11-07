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
/// @brief Dependency-free, versioned uniaxial elastoplastic damage material.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace RoR {
namespace CalibratedBeamMaterial {

// This is still the first, pre-integration schema. The state layout below is
// intentionally being fixed before any authored material or savegame can
// persist it.
static const std::uint32_t MATERIAL_SCHEMA_VERSION = 1;

/// Version-1 parameters use SI stress/strain units.
///
/// damage_driver_capacity_density is the local post-onset damage-driver
/// capacity per reference volume (J/m^3, dimensionally Pa). It is not, by
/// itself, a mesh-independent crack energy and must not be assigned directly
/// from G_f / l_char. A beam adapter must solve or calibrate this driver
/// capacity against one explicitly declared fracture-energy convention, then
/// multiply density results by reference volume. For this law's monotonic
/// post-onset nominal stress/total-strain-area
/// convention, C = 2 (G_f / l_char) / (1 + H/E); a total-dissipation convention
/// has a different mapping. Cyclic response still requires calibration. That
/// crack-band work belongs in the area/rest-length adapter so a refined beam
/// network does not absorb proportionally more energy. Mesh objectivity also
/// depends on controlling strain localization; characteristic length alone is
/// not a complete proof.
struct Parameters
{
    std::uint32_t schema_version = MATERIAL_SCHEMA_VERSION;
    double elastic_modulus = 0.0;                 //!< Pa
    double yield_stress = 0.0;                    //!< Pa
    double hardening_modulus = 0.0;               //!< Pa per plastic strain
    double damage_onset_plastic_strain = 0.0;     //!< dimensionless
    double damage_driver_capacity_density = 0.0;  //!< J/m^3 driver cap
};

/// History variables that must be reset, saved, replayed, and hashed together.
///
/// last_total_strain makes the assumed linear strain path for an Update
/// explicit. Without it, within-step work, reversal, and a fracture event
/// cannot be reconstructed from the plastic variables alone.
struct State
{
    double plastic_strain = 0.0;
    double accumulated_plastic_strain = 0.0;
    double damage = 0.0;
    double damage_driver_density = 0.0;
    double last_total_strain = 0.0;
    bool fractured = false;
};

enum class Error
{
    NONE,
    UNSUPPORTED_SCHEMA,
    NONFINITE_INPUT,
    INVALID_ELASTIC_MODULUS,
    INVALID_YIELD_STRESS,
    INVALID_HARDENING_MODULUS,
    INVALID_DAMAGE_ONSET,
    INVALID_DAMAGE_DRIVER_CAPACITY,
    INVALID_STATE,
    NUMERIC_OVERFLOW
};

struct Response
{
    State state;
    Error error = Error::NONE;
    double stress = 0.0;                         //!< Nominal axial stress, Pa
    double effective_stress = 0.0;               //!< Undamaged stress, Pa
    double elastic_strain = 0.0;
    double plastic_multiplier = 0.0;             //!< Accepted, event-limited
    double nominal_tangent_modulus = 0.0;        //!< d(nominal stress)/d(strain)
    double stored_energy_density = 0.0;
    double plastic_dissipation_increment = 0.0;
    double damage_dissipation_increment = 0.0;
    double mechanical_work_increment = 0.0;      //!< Exact linear-path work
    double peak_abs_stress = 0.0;                //!< Peak within this update
    double fracture_event_strain = 0.0;          //!< Valid when fractured_this_step
    bool yielded = false;
    bool fractured_this_step = false;

    bool IsValid() const { return error == Error::NONE; }
};

/// Field-for-field equality, IEEE `==` on every double: a NaN field never
/// compares equal, so anything keyed on this equality is never reused for
/// it. The production adapter keys its configuration proof on the
/// parameters and the network range keys its proven-history skip on the
/// state; both fall back to the full check the moment a field differs.
inline bool IdenticalParameters(
    const Parameters& first,
    const Parameters& second)
{
    return first.schema_version == second.schema_version &&
        first.elastic_modulus == second.elastic_modulus &&
        first.yield_stress == second.yield_stress &&
        first.hardening_modulus == second.hardening_modulus &&
        first.damage_onset_plastic_strain ==
            second.damage_onset_plastic_strain &&
        first.damage_driver_capacity_density ==
            second.damage_driver_capacity_density;
}

inline bool IdenticalState(const State& first, const State& second)
{
    return first.plastic_strain == second.plastic_strain &&
        first.accumulated_plastic_strain ==
            second.accumulated_plastic_strain &&
        first.damage == second.damage &&
        first.damage_driver_density == second.damage_driver_density &&
        first.last_total_strain == second.last_total_strain &&
        first.fractured == second.fractured;
}

inline bool IsFinite(double value)
{
    static_assert(
        sizeof(double) == sizeof(std::uint64_t),
        "64-bit IEEE-754 double required");
    static_assert(
        std::numeric_limits<double>::is_iec559,
        "IEEE-754 double required");

    // The integer exponent mask over the object representation is the
    // guard; it carries no floating-point assumption a fast-math fold could
    // act on under MSVC; GCC/Clang get the register barrier below.
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

inline double NumericTolerance(double scale)
{
    return 256.0 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(scale));
}

inline bool NearNumeric(double first, double second, double scale)
{
    return std::abs(first - second) <= NumericTolerance(scale);
}

inline Error ClassifyFracturePlasticStrain(
    const Parameters& parameters,
    double& fracture_plastic_strain);

/// A parameter set is valid only if this law can integrate it: every scalar
/// must be finite and in range, AND the fracture event must land at a plastic
/// strain the arithmetic can distinguish from damage onset. Without the second
/// clause a set can pass every range check and still fail on every single
/// step, which the production adapter then latches as a fault - a beam that is
/// configured but can never carry force. Measured on the NLR ZX-6R: every
/// beam authored `beamDeform: FLT_MAX` derived a yield of 3.4e42 Pa, was
/// accepted at spawn, and was disabled on its first step.
inline Error ValidateParameters(
    const Parameters& parameters,
    double* fracture_plastic_strain = nullptr)
{
    if (fracture_plastic_strain != nullptr)
        *fracture_plastic_strain = 0.0;
    if (parameters.schema_version != MATERIAL_SCHEMA_VERSION)
        return Error::UNSUPPORTED_SCHEMA;
    if (!IsFinite(parameters.elastic_modulus) ||
        !IsFinite(parameters.yield_stress) ||
        !IsFinite(parameters.hardening_modulus) ||
        !IsFinite(parameters.damage_onset_plastic_strain) ||
        !IsFinite(parameters.damage_driver_capacity_density))
    {
        return Error::NONFINITE_INPUT;
    }
    if (parameters.elastic_modulus <= 0.0)
        return Error::INVALID_ELASTIC_MODULUS;
    if (parameters.yield_stress <= 0.0)
        return Error::INVALID_YIELD_STRESS;
    if (parameters.hardening_modulus < 0.0)
        return Error::INVALID_HARDENING_MODULUS;
    if (parameters.damage_onset_plastic_strain < 0.0)
        return Error::INVALID_DAMAGE_ONSET;
    if (parameters.damage_driver_capacity_density <= 0.0)
        return Error::INVALID_DAMAGE_DRIVER_CAPACITY;
    double classified_fracture_plastic_strain = 0.0;
    const Error classification = ClassifyFracturePlasticStrain(
        parameters, classified_fracture_plastic_strain);
    if (classification == Error::NONE && fracture_plastic_strain != nullptr)
        *fracture_plastic_strain = classified_fracture_plastic_strain;
    return classification;
}

inline bool TryFlowStress(
    double accumulated_plastic_strain,
    const Parameters& parameters,
    double& flow_stress)
{
    flow_stress =
        parameters.yield_stress +
        parameters.hardening_modulus *
        accumulated_plastic_strain;
    return IsFinite(flow_stress) && flow_stress > 0.0;
}

/// The damage driver is post-onset effective plastic work:
///
/// kappa(alpha) = integral[sigma_y + H alpha] d alpha.
///
/// Damage is exactly min(kappa / damage_driver_capacity_density, 1).
/// Keeping this a
/// pure function of alpha removes incremental drift and makes state reachability
/// testable.
inline bool TryDamageDriver(
    double accumulated_plastic_strain,
    const Parameters& parameters,
    double& damage_driver)
{
    if (accumulated_plastic_strain <=
        parameters.damage_onset_plastic_strain)
    {
        damage_driver = 0.0;
        return true;
    }

    const double offset =
        accumulated_plastic_strain -
        parameters.damage_onset_plastic_strain;
    double onset_flow_stress = 0.0;
    if (!TryFlowStress(
            parameters.damage_onset_plastic_strain,
            parameters,
            onset_flow_stress))
    {
        return false;
    }

    const double linear = onset_flow_stress * offset;
    const double quadratic =
        0.5 * parameters.hardening_modulus * offset * offset;
    damage_driver = linear + quadratic;
    return IsFinite(linear) &&
        IsFinite(quadratic) &&
        IsFinite(damage_driver) &&
        damage_driver >= 0.0;
}

/// Where the fracture event lands for these parameters, and why it cannot
/// when it cannot. NUMERIC_OVERFLOW names an intermediate that left the
/// finite range; INVALID_YIELD_STRESS names a flow stress so far above the
/// damage capacity that the post-onset increment vanishes against the onset
/// strain itself - the event exists in exact arithmetic but no step could
/// ever reach it, so the set is not one this law can carry.
inline Error ClassifyFracturePlasticStrain(
    const Parameters& parameters,
    double& fracture_plastic_strain)
{
    fracture_plastic_strain = 0.0;
    double onset_flow_stress = 0.0;
    if (!TryFlowStress(
            parameters.damage_onset_plastic_strain,
            parameters,
            onset_flow_stress))
    {
        return Error::NUMERIC_OVERFLOW;
    }

    double post_onset_increment = 0.0;
    if (parameters.hardening_modulus == 0.0)
    {
        post_onset_increment =
            parameters.damage_driver_capacity_density /
            onset_flow_stress;
    }
    else
    {
        const double onset_squared =
            onset_flow_stress * onset_flow_stress;
        const double hardening_energy =
            2.0 *
            parameters.hardening_modulus *
            parameters.damage_driver_capacity_density;
        const double discriminant =
            onset_squared + hardening_energy;
        if (!IsFinite(onset_squared) ||
            !IsFinite(hardening_energy) ||
            !IsFinite(discriminant))
        {
            return Error::NUMERIC_OVERFLOW;
        }

        const double root = std::sqrt(discriminant);
        const double stable_denominator =
            0.5 * onset_flow_stress + 0.5 * root;
        post_onset_increment =
            parameters.damage_driver_capacity_density /
            stable_denominator;
    }
    if (!IsFinite(post_onset_increment))
        return Error::NUMERIC_OVERFLOW;

    const double candidate =
        parameters.damage_onset_plastic_strain +
        post_onset_increment;
    if (!IsFinite(candidate))
        return Error::NUMERIC_OVERFLOW;
    if (!(post_onset_increment > 0.0) ||
        !(candidate > parameters.damage_onset_plastic_strain))
    {
        return Error::INVALID_YIELD_STRESS;
    }
    fracture_plastic_strain = candidate;
    return Error::NONE;
}

inline bool TryFracturePlasticStrain(
    const Parameters& parameters,
    double& fracture_plastic_strain)
{
    return ClassifyFracturePlasticStrain(
        parameters, fracture_plastic_strain) == Error::NONE;
}

/// IsValidState for a parameter set ValidateParameters admitted in this same
/// step, with the fracture plastic strain it classified. The production
/// adapter validates each beam's configuration once per step and hands the
/// result here, so the same finite parameters are not re-classified a second
/// and a third time per beam per step. Every check on the history itself is
/// unchanged; a caller that has not validated the parameters uses
/// IsValidState.
inline bool IsValidStateWithValidatedParameters(
    const State& state,
    const Parameters& parameters,
    double fracture_plastic_strain)
{
    if (!IsFinite(state.plastic_strain) ||
        !IsFinite(state.accumulated_plastic_strain) ||
        !IsFinite(state.damage) ||
        !IsFinite(state.damage_driver_density) ||
        !IsFinite(state.last_total_strain) ||
        state.accumulated_plastic_strain < 0.0 ||
        state.damage < 0.0 ||
        state.damage > 1.0 ||
        state.damage_driver_density < 0.0 ||
        state.damage_driver_density >
            parameters.damage_driver_capacity_density)
    {
        return false;
    }

    const double strain_scale = std::max(
        std::abs(state.plastic_strain),
        state.accumulated_plastic_strain);
    if (std::abs(state.plastic_strain) >
        state.accumulated_plastic_strain +
            NumericTolerance(strain_scale))
    {
        return false;
    }

    double raw_damage_driver = 0.0;
    // Before damage onset no fracture event has occurred or is needed to
    // validate this history. Damage-active and fractured histories compare
    // against the exact event, which the admitted parameters classified.
    const bool requires_fracture_event = state.fractured ||
        state.accumulated_plastic_strain >
            parameters.damage_onset_plastic_strain;
    if (!requires_fracture_event &&
        (state.damage != 0.0 || state.damage_driver_density != 0.0))
    {
        return false;
    }
    if (!TryDamageDriver(
            state.accumulated_plastic_strain,
            parameters,
            raw_damage_driver))
    {
        return false;
    }

    if (state.fractured)
    {
        if (!NearNumeric(
                state.accumulated_plastic_strain,
                fracture_plastic_strain,
                fracture_plastic_strain) ||
            !NearNumeric(state.damage, 1.0, 1.0) ||
            !NearNumeric(
                state.damage_driver_density,
                parameters.damage_driver_capacity_density,
                parameters.damage_driver_capacity_density))
        {
            return false;
        }
        return true;
    }

    if (requires_fracture_event &&
        state.accumulated_plastic_strain >=
        fracture_plastic_strain)
    {
        return false;
    }

    const double expected_work = std::min(
        raw_damage_driver,
        parameters.damage_driver_capacity_density);
    const double expected_damage =
        expected_work / parameters.damage_driver_capacity_density;
    if (!NearNumeric(
            state.damage_driver_density,
            expected_work,
            parameters.damage_driver_capacity_density) ||
        !NearNumeric(state.damage, expected_damage, 1.0) ||
        state.damage >= 1.0)
    {
        return false;
    }

    double flow_stress = 0.0;
    if (!TryFlowStress(
            state.accumulated_plastic_strain,
            parameters,
            flow_stress))
    {
        return false;
    }
    const double effective_stress =
        parameters.elastic_modulus *
        (state.last_total_strain - state.plastic_strain);
    if (!IsFinite(effective_stress))
        return false;

    // The elastic strain can be the difference of two much larger strains.
    // Include that subtraction's conditioning in the tolerance; otherwise a
    // valid return-mapped state can be rejected solely by roundoff in E(e-p).
    const double elastic_evaluation_scale =
        parameters.elastic_modulus *
        (std::abs(state.last_total_strain) +
            std::abs(state.plastic_strain));
    if (!IsFinite(elastic_evaluation_scale))
        return false;
    const double yield_scale = std::max(
        std::max(
            std::abs(effective_stress),
            flow_stress),
        elastic_evaluation_scale);
    return std::abs(effective_stress) <=
        flow_stress + NumericTolerance(yield_scale);
}

inline bool IsValidState(
    const State& state,
    const Parameters& parameters)
{
    double fracture_plastic_strain = 0.0;
    if (ValidateParameters(parameters, &fracture_plastic_strain) !=
        Error::NONE)
    {
        return false;
    }
    return IsValidStateWithValidatedParameters(
        state, parameters, fracture_plastic_strain);
}

inline bool TryStoredEnergy(
    double total_strain,
    const State& state,
    const Parameters& parameters,
    double& stored_energy)
{
    if (state.fractured)
    {
        stored_energy = 0.0;
        return true;
    }

    const double elastic_strain =
        total_strain - state.plastic_strain;
    const double elastic_energy =
        0.5 * parameters.elastic_modulus *
        elastic_strain * elastic_strain;
    const double hardening_energy =
        0.5 * parameters.hardening_modulus *
        state.accumulated_plastic_strain *
        state.accumulated_plastic_strain;
    const double undamaged_energy =
        elastic_energy + hardening_energy;
    stored_energy = (1.0 - state.damage) * undamaged_energy;
    return IsFinite(elastic_energy) &&
        IsFinite(hardening_energy) &&
        IsFinite(undamaged_energy) &&
        IsFinite(stored_energy) &&
        stored_energy >= 0.0;
}

inline bool TryFlowWork(
    double begin_alpha,
    double end_alpha,
    const Parameters& parameters,
    double& flow_work)
{
    const double increment = end_alpha - begin_alpha;
    const double mean_flow_stress =
        parameters.yield_stress +
        0.5 * parameters.hardening_modulus *
        (begin_alpha + end_alpha);
    flow_work = increment * mean_flow_stress;
    return increment >= 0.0 &&
        IsFinite(mean_flow_stress) &&
        IsFinite(flow_work) &&
        flow_work >= 0.0;
}

inline bool TryDamageDriverIntegral(
    double begin_alpha,
    double end_alpha,
    const Parameters& parameters,
    double& integral)
{
    const double begin_offset =
        begin_alpha - parameters.damage_onset_plastic_strain;
    const double end_offset =
        end_alpha - parameters.damage_onset_plastic_strain;
    const double increment = end_offset - begin_offset;
    double onset_flow_stress = 0.0;
    if (begin_offset < 0.0 ||
        increment < 0.0 ||
        !TryFlowStress(
            parameters.damage_onset_plastic_strain,
            parameters,
            onset_flow_stress))
    {
        return false;
    }

    const double quadratic_difference =
        begin_offset * begin_offset +
        begin_offset * end_offset +
        end_offset * end_offset;
    const double mean_linear_term =
        0.5 * onset_flow_stress *
        (begin_offset + end_offset);
    const double mean_quadratic_term =
        parameters.hardening_modulus *
        quadratic_difference / 6.0;
    integral = increment *
        (mean_linear_term + mean_quadratic_term);
    return IsFinite(quadratic_difference) &&
        IsFinite(mean_linear_term) &&
        IsFinite(mean_quadratic_term) &&
        IsFinite(integral) &&
        integral >= 0.0;
}

inline bool TryPlasticDissipation(
    double begin_alpha,
    double end_alpha,
    const Parameters& parameters,
    double& plastic_dissipation)
{
    plastic_dissipation = 0.0;
    if (end_alpha < begin_alpha)
        return false;

    const double onset =
        parameters.damage_onset_plastic_strain;
    const double undamaged_end = std::min(end_alpha, onset);
    if (undamaged_end > begin_alpha)
    {
        const double increment = undamaged_end - begin_alpha;
        plastic_dissipation +=
            parameters.yield_stress * increment;
    }

    const double damaged_begin = std::max(begin_alpha, onset);
    if (end_alpha > damaged_begin)
    {
        double driver_integral = 0.0;
        if (!TryDamageDriverIntegral(
                damaged_begin,
                end_alpha,
                parameters,
                driver_integral))
        {
            return false;
        }
        const double degraded_increment =
            (end_alpha - damaged_begin) -
            driver_integral /
                parameters.damage_driver_capacity_density;
        const double degraded_tolerance =
            NumericTolerance(end_alpha - damaged_begin);
        if (degraded_increment < -degraded_tolerance)
            return false;
        plastic_dissipation +=
            parameters.yield_stress *
            std::max(0.0, degraded_increment);
    }

    return IsFinite(plastic_dissipation) &&
        plastic_dissipation >= 0.0;
}

inline bool TryPlasticPathWork(
    double begin_alpha,
    double end_alpha,
    const Parameters& parameters,
    double& path_work)
{
    path_work = 0.0;
    if (end_alpha < begin_alpha)
        return false;

    const double onset =
        parameters.damage_onset_plastic_strain;
    const double undamaged_end = std::min(end_alpha, onset);
    if (undamaged_end > begin_alpha)
    {
        double work = 0.0;
        if (!TryFlowWork(
                begin_alpha,
                undamaged_end,
                parameters,
                work))
        {
            return false;
        }
        path_work += work;
    }

    const double damaged_begin = std::max(begin_alpha, onset);
    if (end_alpha > damaged_begin)
    {
        double begin_driver = 0.0;
        double end_driver = 0.0;
        if (!TryDamageDriver(
                damaged_begin,
                parameters,
                begin_driver) ||
            !TryDamageDriver(
                end_alpha,
                parameters,
                end_driver))
        {
            return false;
        }
        begin_driver = std::min(
            begin_driver,
            parameters.damage_driver_capacity_density);
        end_driver = std::min(
            end_driver,
            parameters.damage_driver_capacity_density);
        const double driver_increment =
            end_driver - begin_driver;
        const double mean_survival =
            1.0 -
            0.5 * (begin_driver + end_driver) /
                parameters.damage_driver_capacity_density;
        if (driver_increment < 0.0 ||
            mean_survival < -NumericTolerance(1.0))
        {
            return false;
        }
        path_work += driver_increment *
            std::max(0.0, mean_survival);
    }

    const double strain_per_alpha =
        1.0 +
        parameters.hardening_modulus /
            parameters.elastic_modulus;
    path_work *= strain_per_alpha;
    return IsFinite(strain_per_alpha) &&
        IsFinite(path_work) &&
        path_work >= 0.0;
}

inline bool TryNominalYieldMagnitude(
    double accumulated_plastic_strain,
    const Parameters& parameters,
    double& magnitude)
{
    double flow_stress = 0.0;
    double driver = 0.0;
    if (!TryFlowStress(
            accumulated_plastic_strain,
            parameters,
            flow_stress) ||
        !TryDamageDriver(
            accumulated_plastic_strain,
            parameters,
            driver))
    {
        return false;
    }
    const double damage = std::min(
        driver / parameters.damage_driver_capacity_density,
        1.0);
    magnitude = (1.0 - damage) * flow_stress;
    return IsFinite(magnitude) && magnitude >= 0.0;
}

inline bool TryDamageSofteningDerivativeNumerator(
    double accumulated_plastic_strain,
    const Parameters& parameters,
    double& numerator)
{
    double flow_stress = 0.0;
    double driver = 0.0;
    if (!TryFlowStress(
            accumulated_plastic_strain,
            parameters,
            flow_stress) ||
        !TryDamageDriver(
            accumulated_plastic_strain,
            parameters,
            driver))
    {
        return false;
    }
    const double hardening_term =
        parameters.hardening_modulus *
        (parameters.damage_driver_capacity_density -
            std::min(driver, parameters.damage_driver_capacity_density));
    const double softening_term = flow_stress * flow_stress;
    numerator = hardening_term - softening_term;
    return IsFinite(hardening_term) &&
        IsFinite(softening_term) &&
        IsFinite(numerator);
}

inline bool TryPeakNominalYieldMagnitude(
    double begin_alpha,
    double end_alpha,
    const Parameters& parameters,
    double& peak_magnitude)
{
    double begin_magnitude = 0.0;
    double end_magnitude = 0.0;
    if (!TryNominalYieldMagnitude(
            begin_alpha,
            parameters,
            begin_magnitude) ||
        !TryNominalYieldMagnitude(
            end_alpha,
            parameters,
            end_magnitude))
    {
        return false;
    }
    peak_magnitude = std::max(begin_magnitude, end_magnitude);

    const double onset =
        parameters.damage_onset_plastic_strain;
    if (begin_alpha < onset && onset < end_alpha)
    {
        double onset_magnitude = 0.0;
        if (!TryNominalYieldMagnitude(
                onset,
                parameters,
                onset_magnitude))
        {
            return false;
        }
        peak_magnitude =
            std::max(peak_magnitude, onset_magnitude);
    }

    if (parameters.hardening_modulus == 0.0 ||
        end_alpha <= onset)
    {
        return true;
    }

    double lower = std::max(begin_alpha, onset);
    double upper = end_alpha;
    double lower_derivative = 0.0;
    double upper_derivative = 0.0;
    if (!TryDamageSofteningDerivativeNumerator(
            lower,
            parameters,
            lower_derivative) ||
        !TryDamageSofteningDerivativeNumerator(
            upper,
            parameters,
            upper_derivative))
    {
        return false;
    }

    if (lower_derivative > 0.0 && upper_derivative < 0.0)
    {
        for (int iteration = 0; iteration < 64; ++iteration)
        {
            const double midpoint =
                lower + 0.5 * (upper - lower);
            double midpoint_derivative = 0.0;
            if (!TryDamageSofteningDerivativeNumerator(
                    midpoint,
                    parameters,
                    midpoint_derivative))
            {
                return false;
            }
            if (midpoint_derivative > 0.0)
                lower = midpoint;
            else
                upper = midpoint;
        }

        double critical_magnitude = 0.0;
        if (!TryNominalYieldMagnitude(
                lower + 0.5 * (upper - lower),
                parameters,
                critical_magnitude))
        {
            return false;
        }
        peak_magnitude =
            std::max(peak_magnitude, critical_magnitude);
    }
    return IsFinite(peak_magnitude);
}

inline Response FailureResponse(
    const State& previous_state,
    Error error)
{
    Response response;
    response.state = previous_state;
    response.error = error;
    return response;
}

/// Advances one material point along a linear total-strain segment.
///
/// The undamaged effective material uses a closed-form backward-Euler return
/// map with linear isotropic hardening. Its free energy is
///
///   psi = (1 - d) [ 0.5 E epsilon_e^2 + 0.5 H alpha^2 ].
///
/// Consequently the plastic dissipation is (1-d) sigma_y d alpha and the
/// damage dissipation is psi_0 d d. Both increments below are integrated over
/// the accepted path, not evaluated only at the endpoint. If the requested
/// step crosses d=1, plastic history is capped at the exact fracture event;
/// the remaining strain segment carries zero stress and performs zero work.
/// Update for a parameter set ValidateParameters admitted in this same step,
/// with the fracture plastic strain it classified (see
/// IsValidStateWithValidatedParameters). Numerically identical to Update.
///
/// `previous_state_validated` skips the re-check of `previous_state` and
/// nothing else. A caller passes true only for a history that already
/// passed IsValidStateWithValidatedParameters under these same parameters
/// and this same fracture strain and has not been written since: the state
/// the previous update of this same material point published, which passed
/// that check before it was returned (the fractured early return below
/// publishes without it, so a fractured history is never claimed). The
/// check is a pure function of its arguments, so on such a history it
/// would return true again; skipping it changes no arithmetic and no
/// outcome. Any history the caller cannot prove that way takes the check.
inline Response UpdateWithValidatedParameters(
    double total_strain,
    const Parameters& parameters,
    double fracture_plastic_strain,
    const State& previous_state,
    bool previous_state_validated = false)
{
    if (!IsFinite(total_strain))
        return FailureResponse(previous_state, Error::NONFINITE_INPUT);

    if (!previous_state_validated &&
        !IsValidStateWithValidatedParameters(
            previous_state, parameters, fracture_plastic_strain))
    {
        return FailureResponse(previous_state, Error::INVALID_STATE);
    }

    Response response;
    response.state = previous_state;
    response.state.last_total_strain = total_strain;

    if (previous_state.fractured)
        return response;

    double previous_stored_energy = 0.0;
    if (!TryStoredEnergy(
            previous_state.last_total_strain,
            previous_state,
            parameters,
            previous_stored_energy))
    {
        return FailureResponse(previous_state, Error::NUMERIC_OVERFLOW);
    }

    double current_flow_stress = 0.0;
    if (!TryFlowStress(
            previous_state.accumulated_plastic_strain,
            parameters,
            current_flow_stress))
    {
        return FailureResponse(previous_state, Error::NUMERIC_OVERFLOW);
    }

    const double previous_effective_stress =
        parameters.elastic_modulus *
        (previous_state.last_total_strain -
            previous_state.plastic_strain);
    const double previous_nominal_stress =
        (1.0 - previous_state.damage) *
        previous_effective_stress;
    const double trial_stress =
        parameters.elastic_modulus *
        (total_strain - previous_state.plastic_strain);
    if (!IsFinite(previous_effective_stress) ||
        !IsFinite(previous_nominal_stress) ||
        !IsFinite(trial_stress))
    {
        return FailureResponse(previous_state, Error::NUMERIC_OVERFLOW);
    }

    const double yield_scale = std::max(
        std::abs(trial_stress),
        current_flow_stress);
    const double yield_function =
        std::abs(trial_stress) - current_flow_stress;
    if (yield_function <= NumericTolerance(yield_scale))
    {
        response.elastic_strain =
            total_strain - previous_state.plastic_strain;
        response.effective_stress = trial_stress;
        response.stress =
            (1.0 - previous_state.damage) * trial_stress;
        response.nominal_tangent_modulus =
            (1.0 - previous_state.damage) *
            parameters.elastic_modulus;
        if (!TryStoredEnergy(
                total_strain,
                response.state,
                parameters,
                response.stored_energy_density))
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }
        response.mechanical_work_increment =
            response.stored_energy_density -
            previous_stored_energy;
        response.peak_abs_stress = std::max(
            std::abs(previous_nominal_stress),
            std::abs(response.stress));
    }
    else
    {
        // A plastic update needs a bounded fracture event: the admitted
        // parameters classified it. It is never clamped, capacity is never
        // retuned, and no partial history is committed.
        const double return_denominator =
            parameters.elastic_modulus +
            parameters.hardening_modulus;
        if (!IsFinite(return_denominator) ||
            return_denominator <= 0.0)
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }

        const double requested_multiplier =
            yield_function / return_denominator;
        const double remaining_multiplier =
            fracture_plastic_strain -
            previous_state.accumulated_plastic_strain;
        if (!IsFinite(requested_multiplier) ||
            requested_multiplier <= 0.0 ||
            !IsFinite(remaining_multiplier) ||
            remaining_multiplier <= 0.0)
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }

        const bool reaches_fracture =
            requested_multiplier >= remaining_multiplier;
        const double accepted_multiplier =
            reaches_fracture ?
            remaining_multiplier :
            requested_multiplier;
        const double flow_direction =
            trial_stress < 0.0 ? -1.0 : 1.0;

        response.yielded = true;
        response.fractured_this_step = reaches_fracture;
        response.plastic_multiplier = accepted_multiplier;
        response.state.plastic_strain +=
            flow_direction * accepted_multiplier;
        response.state.accumulated_plastic_strain =
            reaches_fracture ?
            fracture_plastic_strain :
            previous_state.accumulated_plastic_strain +
                accepted_multiplier;

        double new_flow_stress = 0.0;
        if (!TryFlowStress(
                response.state.accumulated_plastic_strain,
                parameters,
                new_flow_stress))
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }

        if (reaches_fracture)
        {
            response.state.damage = 1.0;
            response.state.damage_driver_density =
                parameters.damage_driver_capacity_density;
            response.state.fractured = true;
            response.fracture_event_strain =
                response.state.plastic_strain +
                flow_direction *
                new_flow_stress /
                    parameters.elastic_modulus;
            if (!IsFinite(response.fracture_event_strain))
            {
                return FailureResponse(
                    previous_state,
                    Error::NUMERIC_OVERFLOW);
            }
        }
        else
        {
            double damage_driver = 0.0;
            if (!TryDamageDriver(
                    response.state.accumulated_plastic_strain,
                    parameters,
                    damage_driver))
            {
                return FailureResponse(
                    previous_state,
                    Error::NUMERIC_OVERFLOW);
            }
            response.state.damage_driver_density =
                damage_driver;
            response.state.damage =
                damage_driver /
                parameters.damage_driver_capacity_density;
        }

        const double yield_event_strain =
            previous_state.plastic_strain +
            flow_direction *
            current_flow_stress /
                parameters.elastic_modulus;
        State yield_event_state = previous_state;
        double yield_event_energy = 0.0;
        if (!IsFinite(yield_event_strain) ||
            !TryStoredEnergy(
                yield_event_strain,
                yield_event_state,
                parameters,
                yield_event_energy))
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }

        double plastic_path_work = 0.0;
        if (!TryPlasticPathWork(
                previous_state.accumulated_plastic_strain,
                response.state.accumulated_plastic_strain,
                parameters,
                plastic_path_work) ||
            !TryPlasticDissipation(
                previous_state.accumulated_plastic_strain,
                response.state.accumulated_plastic_strain,
                parameters,
                response.plastic_dissipation_increment))
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }

        response.mechanical_work_increment =
            (yield_event_energy - previous_stored_energy) +
            plastic_path_work;
        if (!TryStoredEnergy(
                total_strain,
                response.state,
                parameters,
                response.stored_energy_density))
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }

        response.damage_dissipation_increment =
            response.mechanical_work_increment -
            (response.stored_energy_density -
                previous_stored_energy) -
            response.plastic_dissipation_increment;
        const double energy_scale = std::max(
            std::abs(response.mechanical_work_increment),
            std::max(
                previous_stored_energy,
                response.stored_energy_density +
                    response.plastic_dissipation_increment));
        const double energy_tolerance =
            NumericTolerance(energy_scale);
        if (response.damage_dissipation_increment <
            -energy_tolerance)
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }
        response.damage_dissipation_increment =
            std::max(
                0.0,
                response.damage_dissipation_increment);

        double plastic_peak = 0.0;
        if (!TryPeakNominalYieldMagnitude(
                previous_state.accumulated_plastic_strain,
                response.state.accumulated_plastic_strain,
                parameters,
                plastic_peak))
        {
            return FailureResponse(
                previous_state,
                Error::NUMERIC_OVERFLOW);
        }
        response.peak_abs_stress = std::max(
            std::abs(previous_nominal_stress),
            plastic_peak);

        if (reaches_fracture)
        {
            response.stress = 0.0;
            response.effective_stress = 0.0;
            response.elastic_strain = 0.0;
            response.nominal_tangent_modulus = 0.0;
            response.stored_energy_density = 0.0;
        }
        else
        {
            response.elastic_strain =
                total_strain -
                response.state.plastic_strain;
            response.effective_stress =
                parameters.elastic_modulus *
                response.elastic_strain;
            response.stress =
                (1.0 - response.state.damage) *
                response.effective_stress;

            if (response.state.accumulated_plastic_strain >
                parameters.damage_onset_plastic_strain)
            {
                const double damage_softening =
                    new_flow_stress * new_flow_stress /
                    parameters.damage_driver_capacity_density;
                const double tangent_bracket =
                    (1.0 - response.state.damage) *
                    parameters.hardening_modulus -
                    damage_softening;
                response.nominal_tangent_modulus =
                    parameters.elastic_modulus /
                    return_denominator *
                    tangent_bracket;
            }
            else
            {
                response.nominal_tangent_modulus =
                    parameters.elastic_modulus *
                    parameters.hardening_modulus /
                    return_denominator;
            }
        }
    }

    if (!IsValidStateWithValidatedParameters(
            response.state, parameters, fracture_plastic_strain) ||
        !IsFinite(response.stress) ||
        !IsFinite(response.effective_stress) ||
        !IsFinite(response.elastic_strain) ||
        !IsFinite(response.plastic_multiplier) ||
        !IsFinite(response.nominal_tangent_modulus) ||
        !IsFinite(response.stored_energy_density) ||
        !IsFinite(response.plastic_dissipation_increment) ||
        !IsFinite(response.damage_dissipation_increment) ||
        !IsFinite(response.mechanical_work_increment) ||
        !IsFinite(response.peak_abs_stress) ||
        !IsFinite(response.fracture_event_strain) ||
        response.stored_energy_density < 0.0 ||
        response.plastic_dissipation_increment < 0.0 ||
        response.damage_dissipation_increment < 0.0 ||
        response.peak_abs_stress < 0.0)
    {
        return FailureResponse(previous_state, Error::NUMERIC_OVERFLOW);
    }
    return response;
}

inline Response Update(
    double total_strain,
    const Parameters& parameters,
    const State& previous_state)
{
    double fracture_plastic_strain = 0.0;
    const Error parameter_error =
        ValidateParameters(parameters, &fracture_plastic_strain);
    if (parameter_error != Error::NONE)
        return FailureResponse(previous_state, parameter_error);
    return UpdateWithValidatedParameters(
        total_strain, parameters, fracture_plastic_strain, previous_state);
}

} // namespace CalibratedBeamMaterial
} // namespace RoR
