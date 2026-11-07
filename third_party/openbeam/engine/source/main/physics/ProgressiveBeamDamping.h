/*
    This source file is part of Rigs of Rods
    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3.
*/

/// @file
/// @brief BeamNG's progressive (velocity-split) beam damping, clamped so it
///        cannot leave the fixed-step envelope the un-split beam occupied.
///
/// WHAT THIS IS
/// ------------
/// A BeamNG beam may author four damping coefficients instead of one:
/// `beamDamp` and `beamDampFast` on the compression side, `beamDampRebound`
/// and `beamDampReboundFast` on the extension side, with `beamDampVelocitySplit`
/// naming the axial speed at which each side hands over from its slow rate to
/// its fast rate. Real content uses this to make a damper DIGRESSIVE - soft at
/// speed so a sharp bump is not transmitted - and to damp rebound harder than
/// bump. RoR's beam carries one coefficient, so until now all four were
/// dropped and the beam ran at its slow bump rate everywhere.
///
/// THE LAW, and why it is a coefficient and not a force
/// ----------------------------------------------------
/// The authored law is piecewise linear in FORCE, not in the coefficient:
///
///     |F| = slow * min(|v|, split) + fast * max(0, |v| - split)
///
/// so the coefficient a solver needs, `d = |F| / |v|`, is the SECANT of that
/// curve and is a convex combination of `slow` and `fast`:
///
///     |v| <= split :  d = slow
///     |v| >  split :  d = (slow * split + fast * (|v| - split)) / |v|
///
/// This is the same shape RoR's own `Actor::CalcShocks3` already computes for
/// the `shocks3` law, and it is where its supremum comes from: because `d` is
/// a convex combination, `sup_v d(v) = max(slow, fast)` EXACTLY - tight, not
/// merely an upper bound. `PeakCoefficient` below is that supremum, and it is
/// what makes the envelope clamp a true bound rather than an estimate.
///
/// Note `shocks3` is not reachable from an import and nothing here assumes it
/// is: `ProcessShock3` sets `bm_type = BEAM_HYDRO`, which is mutually
/// exclusive with `calibrated_material.enabled`. This element lives on the
/// ordinary beam path instead, so the split composes with the calibrated
/// elastoplastic law rather than displacing it.
///
/// THE CLAMP, and why the split must not widen the envelope
/// -------------------------------------------------------
/// The fixed-step envelope charges each beam ONE damping coefficient and
/// bounds, per node, the largest eigenvalue of that node's incident damping
/// block `sum_j r_j u_j u_j^T` against `DAMPING_NUMBER_MAX`. Admitting a
/// rebound rate ABOVE the charged one would raise `r_j` for that beam, and
/// since the block is a sum of positive-semidefinite rank-1 terms its largest
/// eigenvalue is non-decreasing in every `r_j`. The node would then sit
/// outside the envelope it was admitted under.
///
/// Re-charging the envelope the larger rate instead was measured and rejected:
/// the charge is applied as a RATIO to the rate the runtime presents, so
/// charging a bigger number shrinks every damper on the vehicle. On real
/// content that moved the GT-R's worst damping reduction from 1 500 to 7 000
/// and the A319's from 15 200 to 40 000. Nor can the cost be booked only
/// against the beams that author a split: at a saturated node the max-min
/// redistribution deepens the reduction of the node's OTHER beams whatever
/// the charge is attributed to, so no accounting rule makes a non-splitting
/// beam free.
///
/// So the split is clamped instead. `ClampToEnvelopeCharge` caps every one of
/// the four rates at the coefficient the envelope already charged this beam,
/// which gives, for every velocity,
///
///     d(v) <= max(slow, fast) <= charge
///
/// i.e. the beam can never present more damping than it presented as a plain
/// linear beam. The envelope's inputs are untouched, so its admission, its
/// refusals and every figure on its receipt are bit-identical with the split
/// present or absent - neutrality by construction, not by measurement. What
/// the content still gets is the whole digressive half of its curve: rates
/// BELOW the charge pass through exactly, which is where `beamDampFast` and
/// `beamDampReboundFast` live. Only a rate above the charge is cut, and
/// cutting it is the conservative direction (it under-damps rather than
/// over-damps).
///
/// MEMORYLESS ON PURPOSE
/// ---------------------
/// `Coefficient` is a pure function of the current axial velocity, so this
/// element needs no per-beam state and cannot desynchronise a deterministic
/// replay. That is also why BeamNG's `dampCutoffHz` - a first-order low-pass
/// on damper force, which by definition needs its previous output - is NOT
/// implemented here and stays disclosed as retained-but-unsimulated.

#pragma once

#include "BeamAxialResponse.h"

#include <algorithm>

namespace RoR {
namespace ProgressiveBeamDamping {

/// One beam's progressive damping, already clamped and already scaled by
/// whatever ratio the fixed-step envelope returned. `authored == false` is
/// the ordinary case and means the beam runs on its single linear rate.
///
/// `authored` is a FLAG and not a zero/negative sentinel on any rate: this
/// tree is compiled with `-ffast-math` in release, under which a sentinel's
/// guard is exactly what the optimiser is licensed to fold away.
struct Rates
{
    bool  authored = false;
    /// Compression side (`beamDamp`, `beamDampFast`).
    float slow_compression = 0.0f;
    float fast_compression = 0.0f;
    /// Extension side (`beamDampRebound`, `beamDampReboundFast`).
    float slow_expansion = 0.0f;
    float fast_expansion = 0.0f;
    /// `beamDampVelocitySplit`, in m/s. A speed, never a rate - so the
    /// envelope ratio must not touch it.
    float velocity_split = 0.0f;
};

/// Every rate finite and non-negative, and a positive handover speed. A
/// non-positive or non-finite split is treated as NO split rather than as a
/// hand-over at zero: that keeps the beam on exactly the behaviour it had
/// before this element existed instead of inventing a default the content
/// never authored.
inline bool IsUsable(const Rates& rates)
{
    return rates.authored &&
        BeamAxialResponse::IsFinite(rates.slow_compression) &&
        BeamAxialResponse::IsFinite(rates.fast_compression) &&
        BeamAxialResponse::IsFinite(rates.slow_expansion) &&
        BeamAxialResponse::IsFinite(rates.fast_expansion) &&
        BeamAxialResponse::IsFinite(rates.velocity_split) &&
        rates.slow_compression >= 0.0f &&
        rates.fast_compression >= 0.0f &&
        rates.slow_expansion >= 0.0f &&
        rates.fast_expansion >= 0.0f &&
        rates.velocity_split > 0.0f;
}

/// `sup_v d(v)`, exactly. The secant is a convex combination of the two rates
/// on whichever side the velocity falls, so the supremum over all velocities
/// is the largest of the four. Returns 0 for an unusable set, which is the
/// correct charge for a set nothing will present.
inline float PeakCoefficient(const Rates& rates)
{
    if (!IsUsable(rates))
        return 0.0f;
    return std::max(
        std::max(rates.slow_compression, rates.fast_compression),
        std::max(rates.slow_expansion, rates.fast_expansion));
}

/// The coefficient this beam presents at `axial_relative_velocity`.
///
/// SIGN CONVENTION: positive is EXTENSION (the beam is lengthening), matching
/// `BeamAxialKinematics` - whose axial velocity is `(v1 - v2)` projected on
/// the unit vector from endpoint 2 to endpoint 1 - and matching
/// `Actor::CalcShocks3`, whose own branch reads `v > 0 // Extension`.
///
/// `linear_fallback` is returned whenever the split is absent or unusable, so
/// a caller can pass the beam's ordinary `d` and get today's behaviour with no
/// branch of its own.
inline float Coefficient(
    float axial_relative_velocity,
    const Rates& rates,
    float linear_fallback)
{
    if (!IsUsable(rates) ||
        !BeamAxialResponse::IsFinite(axial_relative_velocity))
    {
        return linear_fallback;
    }

    const bool expanding = axial_relative_velocity > 0.0f;
    const float slow = expanding ? rates.slow_expansion : rates.slow_compression;
    const float fast = expanding ? rates.fast_expansion : rates.fast_compression;
    const float speed = (axial_relative_velocity < 0.0f)
        ? -axial_relative_velocity
        : axial_relative_velocity;

    // At or below the handover the secant IS the slow rate, exactly. Taking
    // this branch first is also what keeps the division below safe without an
    // epsilon: it runs only when `speed > velocity_split > 0`.
    if (!(speed > rates.velocity_split))
        return slow;

    const float force =
        (slow * rates.velocity_split) +
        (fast * (speed - rates.velocity_split));
    const float coefficient = force / speed;
    if (!BeamAxialResponse::IsFinite(coefficient) || coefficient < 0.0f)
        return linear_fallback;
    return coefficient;
}

/// THE ENVELOPE-NEUTRAL CLAMP. Caps every rate at the coefficient the
/// fixed-step envelope already charged this beam, so `PeakCoefficient` of the
/// result cannot exceed `charge` and the node's damping eigenvalue cannot rise
/// above the value it was admitted at. A non-finite or non-positive charge
/// disables the split rather than admitting it uncharged.
inline Rates ClampToEnvelopeCharge(const Rates& rates, float charge)
{
    Rates clamped = rates;
    if (!IsUsable(rates) ||
        !BeamAxialResponse::IsFinite(charge) ||
        !(charge > 0.0f))
    {
        return Rates();
    }
    clamped.slow_compression = std::min(clamped.slow_compression, charge);
    clamped.fast_compression = std::min(clamped.fast_compression, charge);
    clamped.slow_expansion = std::min(clamped.slow_expansion, charge);
    clamped.fast_expansion = std::min(clamped.fast_expansion, charge);
    return clamped;
}

/// Applies the ratio the envelope returned to all four rates, exactly as the
/// `|BOUNDED` element's two ends are scaled. The handover SPEED is deliberately
/// untouched: it is a velocity, and scaling it would move the knee of the
/// curve rather than its height.
///
/// A ratio of zero yields NO curve rather than a flat zero one, symmetrically
/// with `ClampToEnvelopeCharge` refusing a zero charge: in both cases the
/// envelope has taken this beam's damping to nothing, the linear rate it falls
/// back to is that same nothing, and a set of four zeroes would otherwise be
/// counted and reported as a progressive damper that is doing something.
inline Rates Scale(const Rates& rates, float scale)
{
    Rates scaled = rates;
    if (!IsUsable(rates) ||
        !BeamAxialResponse::IsFinite(scale) ||
        !(scale > 0.0f))
    {
        return Rates();
    }
    scaled.slow_compression *= scale;
    scaled.fast_compression *= scale;
    scaled.slow_expansion *= scale;
    scaled.fast_expansion *= scale;
    if (!BeamAxialResponse::IsFinite(scaled.slow_compression) ||
        !BeamAxialResponse::IsFinite(scaled.fast_compression) ||
        !BeamAxialResponse::IsFinite(scaled.slow_expansion) ||
        !BeamAxialResponse::IsFinite(scaled.fast_expansion))
    {
        return Rates();
    }
    return scaled;
}

} // namespace ProgressiveBeamDamping
} // namespace RoR
