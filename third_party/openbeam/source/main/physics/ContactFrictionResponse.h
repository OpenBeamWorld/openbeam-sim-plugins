// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace RoR { namespace ContactFrictionResponse {

enum class Status { Ok, InvalidInput, ArithmeticRange };

struct Result
{
    Status status = Status::InvalidInput;
    double opposing_magnitude = 0.0; // Nonnegative N, applied opposite slip.
    bool valid() const { return status == Status::Ok; }
};

inline bool Finite(double value)
{
    static_assert(sizeof(double) == sizeof(std::uint64_t) &&
        std::numeric_limits<double>::is_iec559, "requires IEEE binary64");
    std::uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    volatile std::uint64_t observed = bits;
    // Unlike std::isfinite this guard remains effective with fast-math.
    return (observed & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000);
}

// The existing static-contact smoothing law, evaluated without cancellation:
//   magnitude = staticCapacity * (1 - exp(-slipSpeed/adhesionSpeed)).
// Capacity is already mu_static * normalReaction * material/node factors.
// -expm1(-x) is exactly zero at x=0 and tends continuously to zero as x->0.
// A coarse exp approximation whose exp(0)!=1 introduces a finite force kick;
// direct 1-exp(-x) instead loses the small response through cancellation.
//
// This corrects the evaluation, not the contact model: it neither changes
// capacity nor clamps the later cancellation of existing tangent forces.
// It does NOT prove explicit nodal contact passivity or prevent overshoot
// for a large timestep*capacity/(nodeMass*adhesionSpeed).
inline Result StaticSmoothing(double slip_speed, double adhesion_speed,
    double static_capacity)
{
    Result result;
    if (!Finite(slip_speed) || !Finite(adhesion_speed) || !Finite(static_capacity) ||
        slip_speed < 0.0 || adhesion_speed <= 0.0 || static_capacity < 0.0)
        return result;

    result.status = Status::Ok;
    if (slip_speed == 0.0 || static_capacity == 0.0)
        return result;

    const double ratio = slip_speed / adhesion_speed;
    // Positive finite inputs can overflow only to positive infinity here.
    // The exact finite-input law then rounds to its capacity, not NaN/refusal.
    if (!Finite(ratio))
    {
        result.opposing_magnitude = static_capacity;
        return result;
    }
    const double fraction = -std::expm1(-ratio);
    const double magnitude = static_capacity * fraction;
    if (!Finite(fraction) || fraction < 0.0 || fraction > 1.0 ||
        !Finite(magnitude) || magnitude < 0.0 || magnitude > static_capacity)
    {
        result.status = Status::ArithmeticRange;
        return result;
    }
    result.opposing_magnitude = magnitude;
    return result;
}

// The same response bounded so it cannot REVERSE the slip within one step:
//   magnitude <= nodeMass * slipSpeed / timestep.
// Static contact cancels every other tangent force on the node, so this
// response alone sets the next slip speed, v' = v - magnitude * dt / m. Its
// small-slip gain is capacity / adhesionSpeed; above 2 m / dt that overshoots,
// the next step overshoots back, and the chatter feeds energy into the
// contact. That is the unbounded regime noted above, reached on 2026-09-23 by
// the NLR ZX-6R: 0.159 kg tyre nodes at 10.5x the surface grip crept at rest
// and toppled. Bounded, the slip can come to rest in a step, never flip; a
// response already under the bound is returned exactly as it was.
inline Result NoReversal(const Result& response, double node_mass,
    double slip_speed, double timestep)
{
    Result result;
    if (!response.valid() || !Finite(node_mass) || !Finite(slip_speed) ||
        !Finite(timestep) || node_mass <= 0.0 || slip_speed < 0.0 || timestep <= 0.0)
        return result;
    const double bound = node_mass * slip_speed / timestep;
    if (!Finite(bound) || bound < 0.0)
    {
        result.status = Status::ArithmeticRange;
        return result;
    }
    result.status = Status::Ok;
    result.opposing_magnitude = response.opposing_magnitude < bound
        ? response.opposing_magnitude : bound;
    return result;
}

// The sliding (Stribeck) response bounded the same way. Sliding contact keeps
// every other tangent force on the node, so along the slip the next speed is
//   v' = v + (driving - magnitude) * dt / m,
// `driving` being those forces' component along the slip. The bound
//   magnitude <= max(0, m v / dt + driving)
// uses the SIGNED force: opposing drive reduces the remaining stopping force.
// When the free kick is nonnegative this response cannot reverse it in the
// scalar model, and a response already under the bound is unchanged. If the
// other force alone reverses the slip, return zero in the old-slip direction;
// this limiter cannot prevent that externally driven reversal. `driving`
// must describe the other retained forces on this slip axis and mass. This
// is not a certificate for a later joint-brake re-solve or omitted fluid force.
// Unbounded, a node-to-cab contact that cancels a large inward load - a crash
// pressing a node into a triangle with its own beams, 1e5-1e7 N on a 0.4-12 kg
// node - opposed the slip with mu * load * dt / m, hundreds to thousands of
// m/s, in one step: the slip flipped and grew every step, the hydrodynamic
// term t2 * v raising mu up to 5.75 on the way. Replaying the showcase race
// car against the race track's barrier triangles, a fender node slid at 73,
// 129, 272, 326 and 602 m/s in consecutive steps, and a two-car pile-up into
// the barrier reached 55 km/s with 200 times its kinetic energy.
inline Result SlidingNoReversal(double magnitude, double node_mass,
    double slip_speed, double driving_force, double timestep)
{
    Result result;
    if (!Finite(magnitude) || !Finite(node_mass) || !Finite(slip_speed) ||
        !Finite(driving_force) || !Finite(timestep) || magnitude < 0.0 ||
        node_mass <= 0.0 || slip_speed < 0.0 || timestep <= 0.0)
        return result;
    const double stopping_force = node_mass * slip_speed / timestep + driving_force;
    if (!Finite(stopping_force))
    {
        result.status = Status::ArithmeticRange;
        return result;
    }
    const double bound = stopping_force > 0.0 ? stopping_force : 0.0;
    result.status = Status::Ok;
    result.opposing_magnitude = magnitude < bound ? magnitude : bound;
    return result;
}

// Opt-in Coulomb stick for an actor that declares `openbeamContact.grip =
// "stick"` (JBeamVehicleImporter: robot gripper pads, doc/nextgen/SO101.md).
// `driving` is the node's tangential force this step, `slip` its relative
// tangential velocity. The force that stops the slip in one step is
// T = -driving - m slip / dt; held while |T| <= static_capacity, else the
// contact slides at sliding_capacity against the slip, bounded so it never
// reverses it (SlidingNoReversal). Pure arithmetic, double precision.
// Half the slip is removed per step: a full one-step cancel made the
// SO-101's pads and a squeezed cube chatter (the cube walked out sideways).
constexpr double kStickVelocityGain = 0.5;

struct StickResult
{
    bool sticking = false;
    double tangential[3] = {0.0, 0.0, 0.0};
};

inline StickResult StickGrip(const double (&driving)[3], const double (&slip)[3], double mass, double dt,
    double static_capacity, double sliding_capacity)
{
    StickResult result;
    if (!Finite(mass) || !Finite(dt) || mass <= 0.0 || dt <= 0.0 ||
        !Finite(static_capacity) || !Finite(sliding_capacity) ||
        static_capacity < 0.0 || sliding_capacity < 0.0)
        return result;
    double t[3];
    double t2 = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        if (!Finite(driving[i]) || !Finite(slip[i])) return StickResult();
        t[i] = -driving[i] - kStickVelocityGain * mass * slip[i] / dt;
        t2 += t[i] * t[i];
    }
    if (t2 <= static_capacity * static_capacity)
    {
        result.sticking = true;
        for (int i = 0; i < 3; ++i) result.tangential[i] = t[i];
        return result;
    }
    const double speed = std::sqrt(slip[0] * slip[0] + slip[1] * slip[1] + slip[2] * slip[2]);
    if (!(speed > 0.0))
    {
        // Not moving yet but pushed past the static limit: slide along the push.
        const double push = std::sqrt(t2);
        for (int i = 0; i < 3; ++i) result.tangential[i] = t[i] / push * sliding_capacity;
        return result;
    }
    double along = 0.0;
    for (int i = 0; i < 3; ++i) along += driving[i] * slip[i] / speed;
    const Result bounded = SlidingNoReversal(sliding_capacity, mass, speed, along, dt);
    const double magnitude = bounded.valid() ? bounded.opposing_magnitude : 0.0;
    for (int i = 0; i < 3; ++i) result.tangential[i] = -slip[i] / speed * magnitude;
    return result;
}

// The stick contact's normal correction at rest (the legacy law corrects
// depth only while the node moves in): a gentle separation speed, 2 % of the
// depth per step and at most kStickMaxSeparation, so a light pad node is
// never kicked (20 % per step blew the SO-101's 1.5 g jaw nodes apart).
constexpr double kStickMaxSeparation = 0.05; // m/s
inline double StickDepthBias(double normal_velocity, double depth, double mass, double dt)
{
    if (!Finite(normal_velocity) || !Finite(depth) || !Finite(mass) || !Finite(dt) ||
        normal_velocity < 0.0 || depth <= 0.0 || mass <= 0.0 || dt <= 0.0)
        return 0.0;
    const double speed = 0.02 * depth / dt;
    return (speed < kStickMaxSeparation ? speed : kStickMaxSeparation) * mass / dt;
}

// An imported (JBeam) tyre's contact nodes (node_t::nd_jbeam_tyre_node) grip
// like a road tyre rather than like RoR's truck contact, which on 2026-09-27
// held a braked GT-R's locked tyres glued below 3 m/s of slip: the stop ended
// in a 1.2-1.6 g snap and a 12 cm roll-back, and the car never skidded.
//  - Static contact holds only below kTyreAdhesionSpeed of slip, or below the
//    surface's own adhesion speed when that is smaller (ice).
//  - Sliding friction falls from the static coefficient toward
//    max(mc, kTyreSlidingFloor * ms) along the surface's own Stribeck curve,
//    with no hydrodynamic term (it raised a skid's friction with speed): a
//    locked tyre keeps 70-80 % of its peak on dry asphalt.
constexpr double kTyreAdhesionSpeed = 0.4;  // m/s
constexpr double kTyreSlidingFloor = 0.7;   // of the static coefficient

inline double TyreAdhesionSpeed(double surface_adhesion_speed)
{
    return surface_adhesion_speed < kTyreAdhesionSpeed
        ? surface_adhesion_speed : kTyreAdhesionSpeed;
}

// The sliding response of such a node: coefficient(slip) * reaction, where
// reaction is the moderated normal load (Freaction * strength * friction_coef)
// and the coefficient is
//   floor + (ms - floor) * exp(-(slip / stribeckSpeed)^exponent).
// A zero Stribeck speed, like the surface law's, leaves only the floor.
inline Result TyreSliding(double slip_speed, double reaction,
    double static_coefficient, double sliding_coefficient,
    double stribeck_speed, double exponent)
{
    Result result;
    if (!Finite(slip_speed) || !Finite(reaction) || !Finite(static_coefficient) ||
        !Finite(sliding_coefficient) || !Finite(stribeck_speed) || !Finite(exponent) ||
        slip_speed < 0.0 || reaction < 0.0 || static_coefficient < 0.0 ||
        sliding_coefficient < 0.0 || stribeck_speed < 0.0 || exponent <= 0.0)
        return result;
    const double tyre_floor = kTyreSlidingFloor * static_coefficient;
    const double floor = sliding_coefficient > tyre_floor ? sliding_coefficient : tyre_floor;
    double decay = 0.0;
    if (stribeck_speed > 0.0)
    {
        const double ratio = slip_speed / stribeck_speed;
        const double power = Finite(ratio) ? std::pow(ratio, exponent) : 0.0;
        decay = Finite(ratio) && Finite(power) ? std::exp(-power) : 0.0;
    }
    const double coefficient = floor + (static_coefficient - floor) * decay;
    const double magnitude = coefficient * reaction;
    if (!Finite(coefficient) || coefficient < 0.0 || !Finite(magnitude) || magnitude < 0.0)
    {
        result.status = Status::ArithmeticRange;
        return result;
    }
    result.status = Status::Ok;
    result.opposing_magnitude = magnitude;
    return result;
}

} } // namespace RoR::ContactFrictionResponse
