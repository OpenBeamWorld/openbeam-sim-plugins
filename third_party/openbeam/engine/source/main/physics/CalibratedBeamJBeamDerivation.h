/*
    This source file is part of Rigs of Rods
    Copyright 2024+ Rigs of Rods contributors

    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

/// @file
/// @brief Derives a calibrated beam material from a mod's own authored JBeam
///        thresholds, under exactly one declared assumption.
///
/// The calibrated elastoplastic damage material is complete and wired into the
/// solver, but it is opt-in and nothing a player drives opts in: the BeamNG
/// import path never requests it, so every imported vehicle deforms on the
/// legacy heuristic rest-length law instead. That is not for want of data. A
/// BeamNG beam authors `beamSpring` (N/m), `beamDeform` (N) and `beamStrength`
/// (N), and the 350Z alone ships 561 deform and 558 strength values.
///
/// ## What is derived, and what is assumed
///
/// The **cross-sectional area A is the single free parameter** and it is
/// DECLARED, never measured — a JBeam beam is a scalar spring with no
/// geometry. Once A is fixed, two of the five material parameters follow
/// *exactly* from authored values, with no further assumption:
///
///   * `elastic_modulus  E = k·L / A`   from `beamSpring` and the rest length,
///     because an axial bar of stiffness k satisfies k = EA/L identically.
///   * `yield_stress     σ_y = F_deform / A`   from `beamDeform`, because that
///     is the authored axial force at which permanent deformation begins, and
///     stress is force over area.
///
/// A beam whose `beamDeform` is unbounded (BeamNG spells it `FLT_MAX`: the
/// beam never deforms permanently, though it may still break at its
/// `beamStrength`) has NO yield point, and an elastoplastic law with no yield
/// point is not this law. Such a beam is refused here and keeps the legacy
/// law, which honours exactly that authored pair - no deformation, break at
/// the authored force. Deriving anyway was measured to be destructive: the
/// sentinel widened to a yield of 3.4e42 Pa that passed every range check,
/// and the NLR ZX-6R's suspension springs, rider joints and engine mounts -
/// every beam it authors `FLT_MAX` deform for - were then disabled on their
/// first physics step, silently, because no step of that material could ever
/// be integrated. The derived parameters are therefore validated against the
/// material's own rules before they are handed out.
///
/// The remaining three have no authored counterpart in JBeam and are declared
/// defaults, carried here so every consumer shares one basis rather than
/// inventing its own:
///
///   * `hardening_modulus`
///   * `damage_onset_plastic_strain`
///   * `damage_driver_capacity_density`
///
/// `beamStrength` sets the capacity (basis v2). The legacy law RoR has always
/// applied to the same authored pair lets a stretched beam lose k of strength
/// per metre of plastic stretch and breaks it when the strength falls to the
/// flow force: after (F_s - F_y)/k, a plastic strain of (sigma_s - sigma_y)/E
/// at the flow stress sigma_y. A breakable beam's capacity is that work
/// density, sigma_y (sigma_s - sigma_y) / E, never below the declared default,
/// so no beam fractures earlier than under v1 or the legacy law. The declared
/// area cancels, as it does from E/yield. v1 declared one capacity for every
/// breakable beam and so fractured each at about its DEFORM force: the Bell
/// 407's hub beams (deform 5 kN, strength 11.16 MN) broke at ~9 kN settling
/// from a 1 m spawn (backlog item 20). A beam the author marked unbreakable
/// (BeamNG spells this `FLT_MAX`) still gets a declared capacity far outside
/// any work an admitted beam accumulates.
///
/// ## Why this lives here
///
/// `JBeamToRigDef` is what L4 exists to retire, so a derivation written there
/// is deleted by the content plan. This header depends on nothing but the
/// material types, so the current RigDef lowering and a future native spawner
/// can both consume it unchanged.
///
/// Nothing here is a physical measurement. Every quantity is either an exact
/// consequence of an authored value under the declared area, or a declared
/// default named below — the same footing as the P1 calibration basis ruled on
/// in decision D5.

#pragma once

#include "CalibratedBeamMaterial.h"
#include "CalibratedBeamMaterialAdapter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace RoR {
namespace CalibratedBeamJBeamDerivation {

/// The declared basis. Every field is an assumption, not a measurement, and
/// the whole struct is versioned so a receipt can name which basis produced a
/// given material.
struct DeclaredBasis
{
    /// Basis identity, for disclosure. Bump when any default below changes.
    static const std::uint32_t BASIS_VERSION = 2U;

    /// Cross-sectional area assumed for every beam, m^2. 1 cm^2 is a
    /// deliberately plain round number: it is a declared convention, and
    /// pretending to a more precise figure would imply a measurement that was
    /// never taken. E and yield stress both scale as 1/A, so this choice sets
    /// the absolute scale of both and cancels from their ratio.
    double cross_section_area_m2 = 1.0e-4;

    /// Post-yield slope, as a fraction of the derived elastic modulus.
    /// Structural metals harden at a small fraction of E; 1% keeps the
    /// post-yield branch stable without asserting a specific alloy.
    double hardening_fraction_of_elastic_modulus = 0.01;

    /// Plastic strain at which damage begins to accumulate.
    double damage_onset_plastic_strain = 0.02;

    /// Damage driver capacity, J/m^3: a breakable beam's floor, and the whole
    /// capacity of one whose authored strength does not exceed its deform.
    double damage_driver_capacity_density = 2.0e6;

    /// Capacity used for a beam the author marked unbreakable. It must be
    /// FINITE: CalibratedBeamMaterial::Validate refuses a non-finite parameter
    /// outright, so an infinity here would be rejected and would silently drop
    /// every unbreakable beam back to the legacy law. This is a declared
    /// stand-in for "never fractures", six orders above the breakable value
    /// and far outside the work any admitted beam accumulates.
    double unbreakable_damage_driver_capacity_density = 1.0e12;
};

enum class DerivationError
{
    NONE,
    /// A required authored value was absent. The caller must leave the beam on
    /// the legacy law rather than substituting a guess.
    MISSING_AUTHORED_INPUT,
    NONFINITE_INPUT,
    INVALID_CROSS_SECTION_AREA,
    /// Rest length must be strictly positive: E = k*L/A is undefined at zero.
    INVALID_REST_LENGTH,
    INVALID_SPRING,
    INVALID_DEFORM_THRESHOLD,
    /// The author marked the beam as never deforming (`FLT_MAX`): there is no
    /// yield point to derive, so the beam keeps the legacy law.
    UNBOUNDED_DEFORM_THRESHOLD,
    /// The derived parameters pass every input check but the material itself
    /// refuses them (CalibratedBeamMaterial::ValidateParameters), so no step
    /// could ever be integrated. The beam keeps the legacy law.
    UNREPRESENTABLE_MATERIAL
};

/// One beam's authored JBeam thresholds, reduced to what the derivation reads.
struct AuthoredBeam
{
    double spring_n_per_m = 0.0;    //!< beamSpring
    double deform_n = 0.0;          //!< beamDeform, axial force at yield
    double strength_n = 0.0;        //!< beamStrength; ignored when unbreakable
    double rest_length_m = 0.0;     //!< |p2 - p1| at spawn
    bool has_spring = false;
    bool has_deform = false;
    bool deform_unbounded = false;   //!< authored FLT_MAX: never yields
    bool strength_unbounded = false; //!< authored FLT_MAX: never fractures
    bool has_strength = false;
};

inline bool IsFinitePositive(double value)
{
    return CalibratedBeamMaterial::IsFinite(value) && value > 0.0;
}

/// Derives the material configuration for one beam.
///
/// Returns false and leaves `out` untouched on any refusal, so a caller that
/// ignores the error still cannot silently install a half-derived material.
inline bool DeriveFromAuthoredBeam(
    const AuthoredBeam& authored,
    const DeclaredBasis& basis,
    CalibratedBeamMaterialAdapter::Configuration& out,
    DerivationError& error)
{
    error = DerivationError::NONE;

    if (!authored.has_spring || !authored.has_deform)
    {
        // Both are required: without a spring there is no modulus, and without
        // a deform threshold there is no yield point. A vehicle that authors
        // neither keeps the legacy law, which is the honest outcome.
        error = DerivationError::MISSING_AUTHORED_INPUT;
        return false;
    }
    if (!CalibratedBeamMaterial::IsFinite(authored.spring_n_per_m) ||
        !CalibratedBeamMaterial::IsFinite(authored.deform_n) ||
        !CalibratedBeamMaterial::IsFinite(authored.rest_length_m) ||
        (authored.has_strength && !authored.strength_unbounded &&
            !CalibratedBeamMaterial::IsFinite(authored.strength_n)))
    {
        error = DerivationError::NONFINITE_INPUT;
        return false;
    }
    if (!IsFinitePositive(basis.cross_section_area_m2))
    {
        error = DerivationError::INVALID_CROSS_SECTION_AREA;
        return false;
    }
    if (!IsFinitePositive(authored.rest_length_m))
    {
        error = DerivationError::INVALID_REST_LENGTH;
        return false;
    }
    if (!IsFinitePositive(authored.spring_n_per_m))
    {
        error = DerivationError::INVALID_SPRING;
        return false;
    }
    if (!IsFinitePositive(authored.deform_n))
    {
        error = DerivationError::INVALID_DEFORM_THRESHOLD;
        return false;
    }
    // The importer carries the author's intent explicitly; the sentinel is
    // also recognised by value because a caller that only has the scaled
    // threshold (binary32 FLT_MAX, widened) must reach the same answer.
    if (authored.deform_unbounded ||
        authored.deform_n >=
            static_cast<double>(std::numeric_limits<float>::max()))
    {
        error = DerivationError::UNBOUNDED_DEFORM_THRESHOLD;
        return false;
    }

    // k = E*A/L identically for an axial bar, so E follows exactly.
    const double elastic_modulus =
        authored.spring_n_per_m * authored.rest_length_m /
        basis.cross_section_area_m2;
    // Stress is force over area; beamDeform is the authored yield force.
    const double yield_stress =
        authored.deform_n / basis.cross_section_area_m2;

    if (!IsFinitePositive(elastic_modulus) ||
        !IsFinitePositive(yield_stress))
    {
        error = DerivationError::NONFINITE_INPUT;
        return false;
    }

    CalibratedBeamMaterialAdapter::Configuration configuration;
    configuration.cross_section_area_m2 = basis.cross_section_area_m2;
    configuration.material.elastic_modulus = elastic_modulus;
    configuration.material.yield_stress = yield_stress;
    configuration.material.hardening_modulus =
        elastic_modulus * basis.hardening_fraction_of_elastic_modulus;
    configuration.material.damage_onset_plastic_strain =
        basis.damage_onset_plastic_strain;

    // An unbreakable beam must not fracture at a merely large load, but the
    // capacity has to stay finite (see the basis field). A breakable one
    // fractures no earlier than the legacy law breaks the same authored pair
    // (see the file comment), and never later than an unbreakable one.
    double capacity = basis.damage_driver_capacity_density;
    if (authored.has_strength && !authored.strength_unbounded)
    {
        const double strength_stress =
            authored.strength_n / basis.cross_section_area_m2;
        if (strength_stress > yield_stress)
        {
            const double legacy_work = yield_stress *
                (strength_stress - yield_stress) / elastic_modulus;
            if (IsFinitePositive(legacy_work) && legacy_work > capacity)
                capacity = std::min(legacy_work,
                    basis.unbreakable_damage_driver_capacity_density);
        }
    }
    configuration.material.damage_driver_capacity_density =
        (authored.strength_unbounded || !authored.has_strength)
            ? basis.unbreakable_damage_driver_capacity_density
            : capacity;

    // The material is the authority on what it can integrate. A set it
    // refuses must not leave here: the spawner would accept it and the
    // solver would then disable the beam on its first step.
    if (CalibratedBeamMaterial::ValidateParameters(configuration.material) !=
        CalibratedBeamMaterial::Error::NONE)
    {
        error = DerivationError::UNREPRESENTABLE_MATERIAL;
        return false;
    }

    out = configuration;
    return true;
}

} // namespace CalibratedBeamJBeamDerivation
} // namespace RoR
