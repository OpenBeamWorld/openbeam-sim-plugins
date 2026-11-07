// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/// @file
/// How far inside the ground a node may start, and how fast the ground may
/// push it back out (doc/nextgen/physics_findings/03).
///
/// A `fixes` actor spawned rotated at a cliff lip buried its free end, and the
/// contact response - which cancels 80% of an inward normal speed and adds
/// 0.2 * depth / dt of outward speed in one step - launched those nodes at
/// about 400 m/s per metre of depth: 223 beams broke at 4-6 MN in one second.
/// The two halves of the fix live here so the game and the tests share them:
/// the separation speed a static contact adds is bounded, and a spawn is
/// lifted until no free node starts below the surface.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace RoR {
namespace ContactPenetration {

/// Fastest a static contact (terrain, collision mesh, collision box) pushes
/// a buried node back out, m/s. At the 0.5 ms physics step this binds only
/// beyond 25 mm of depth, which an ordinary contact reaches only from an
/// impact faster than 50 m/s on flat ground: a node's first-contact depth is
/// at most its speed times dt, and the response exits it the step after.
constexpr float kMaxStaticSeparationSpeed = 10.0f;

/// No bound: the contact law exactly as it was (inter-actor contacts keep it).
constexpr float kUnboundedSeparationSpeed = std::numeric_limits<float>::max();

/// The outward speed (<= 0 in the law's sign) the contact adds for a node
/// `depth` (<= 0) inside the surface, floored at -max_separation_speed.
inline float SeparationVelocity(float depth, float dt, float max_separation_speed)
{
    const float separation = 0.2f * depth / dt;
    return std::max(separation, -max_separation_speed);
}

/// The normal reaction primitiveCollision applies: cancel the normal force
/// and, while the node still moves inward, remove 80% of that speed and push
/// it out of `depth` at up to `max_separation_speed` (Newton's second law
/// over one step).
inline float NormalReaction(float normal_force, float normal_velocity, float depth,
    float mass, float dt, float max_separation_speed)
{
    float reaction = -normal_force;
    if (normal_velocity < 0)
    {
        reaction -= (0.8f * normal_velocity +
            SeparationVelocity(depth, dt, max_separation_speed)) * mass / dt;
    }
    return reaction;
}

/// The static contact's own recovery (terrain, collision meshes, collision
/// boxes; 2026-10-29). NormalReaction above pushes a node out only while it
/// moves inward, at 0.2 x depth / dt: once moving out nothing slows it, so a
/// light node under a heavy load (a robot's 20-60 g sole carrying 20-60 N)
/// leaves the surface, free-falls one step under its whole load and lands
/// again. On flat ground such a sole lost contact on 12.5 % of steps with
/// 0.17-0.49 m/s of normal speed noise, and the friction it lost on those
/// steps let it slide at 64 mm/s under a fifth of its static capacity.
///
/// Here, inside the surface, the normal speed is driven to one target either
/// way: the outward recovery speed 0.05 x (depth - slop) / dt (the same bound). A node
/// slower than the target is set to it; a faster outward node is slowed to
/// it, but by no more than the inward load the contact already cancels: the
/// ground pushes, it never pulls. The same sole then keeps contact on every
/// step and holds under the same shear; a 2 kg node lands from 3 m/s without
/// rebound. Inter-actor contacts keep NormalReaction, and the joint
/// wheel-brake solve keeps NormalReaction's target (primitiveCollision).
constexpr float kStaticRecoveryRate = 0.05f;

/// The depth (m) a static contact leaves unrecovered. Recovering all the way
/// to the surface decays the depth geometrically until binary32 rounding of
/// the cancelling load and reaction (~1e-11 m per step) pops the node out:
/// a 20 g sole under 20 N then free-fell one step (0.5 m/s) every ~360
/// steps. Resting 10 um inside keeps it in contact; the depth is invisible.
constexpr float kStaticRecoverySlop = 1e-5f;

/// The outward normal speed (>= 0) a static contact drives a node to at `depth` (<= 0).
inline float StaticRecoveryTarget(float depth, float dt, float max_separation_speed)
{
    return -std::max(kStaticRecoveryRate * std::min(depth + kStaticRecoverySlop, 0.f) / dt,
        -max_separation_speed);
}

/// The normal reaction of a static contact (see kStaticRecoveryRate).
inline float StaticNormalReaction(float normal_force, float normal_velocity, float depth,
    float mass, float dt, float max_separation_speed)
{
    const float target = StaticRecoveryTarget(depth, dt, max_separation_speed);
    float reaction = -normal_force;
    if (normal_velocity < target)
        reaction += (target - normal_velocity) * mass / dt;
    else if (normal_force < 0.f)
        reaction += std::max((target - normal_velocity) * mass / dt, normal_force);
    return reaction;
}

/// Depth (m, positive) past which a separation bound starts to bind.
inline float BindingDepth(float max_separation_speed, float dt)
{
    return max_separation_speed * dt / 0.2f;
}

/// Unlike std::isfinite this stays effective under fast-math: a by-value
/// argument can be assumed finite and folded, a volatile read cannot.
inline bool Finite(const float& value)
{
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    volatile std::uint32_t observed = bits;
    return (observed & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

/// One spawned node against the surface directly below it.
struct NodeSurface
{
    float height = 0.0f;         ///< node y
    float surface = 0.0f;        ///< highest ground/mesh surface below the actor's top
    bool fixed = false;          ///< `fixes` node: never integrated
    bool ground_contact = true;  ///< false for nodes that never touch the ground
};

/// How far a spawn must rise so no free node that touches the ground starts
/// below the surface. Fixed nodes never count: they are never integrated, so
/// a buried anchor is not launched, and one set into the ground on purpose
/// keeps its depth.
inline float RequiredSpawnLift(const std::vector<NodeSurface>& nodes)
{
    float lift = 0.0f;
    for (const NodeSurface& node : nodes)
    {
        if (node.fixed || !node.ground_contact || !Finite(node.height) || !Finite(node.surface))
            continue;
        lift = std::max(lift, node.surface - node.height);
    }
    return lift;
}

/// A node still below the surface once the actor has risen by `lift`.
struct Buried
{
    int node = -1;
    float depth = 0.0f; ///< metres below the surface, > 0
    bool fixed = false;
};

/// Every ground-contact node deeper than `tolerance` after rising by `lift`.
inline std::vector<Buried> BuriedAfterLift(const std::vector<NodeSurface>& nodes, float lift,
    float tolerance = 0.001f)
{
    std::vector<Buried> buried;
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        const NodeSurface& node = nodes[i];
        if (!node.ground_contact || !Finite(node.height) || !Finite(node.surface))
            continue;
        const float depth = node.surface - (node.height + lift);
        if (depth > tolerance)
            buried.push_back({static_cast<int>(i), depth, node.fixed});
    }
    return buried;
}

} // namespace ContactPenetration
} // namespace RoR
