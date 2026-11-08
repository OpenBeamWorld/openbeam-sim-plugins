/*
    This source file is part of Rigs of Rods / OpenBeam
    SPDX-License-Identifier: GPL-3.0-or-later
*/

// The OpenBeam soft-body core behind openbeam_core.h.
//
// WHAT RUNS. One fixed 0.5 ms step, in the game's order and binary32 node
// state (node_t):
//   1. gravity, then every beam through the game's own law:
//        * a JBeam beam the game opts into the calibrated elastoplastic damage
//          material (JBeamToRigDef.cpp: every NORMAL beam with a finite
//          deform) steps through CalibratedBeamProductionStep::Step, with the
//          material derived by CalibratedBeamJBeamDerivation under the same
//          declared basis;
//        * any other beam takes the ordinary path of Actor::CalcBeams: the
//          damper clamp (BeamAxialResponse::ComputeDamping), plastic
//          rest-length deformation (ApplyRestLengthDeformation, with
//          deformLimitExpansion's ceiling) and breaking at beamStrength;
//      a broken beam breaks its breakGroup;
//   2. contact of every free collision node against the host's planes and
//      colliders with primitiveCollision's dry branch: ContactPenetration::
//      NormalReaction (cancel 80 % of the inward speed, push out at up to
//      10 m/s) and ContactFrictionResponse's static/sliding friction with the
//      ground model's Stribeck curve, in the collider's moving frame;
//   3. kick and drift (v += F/m h; x += v h).
// The force each collider took (minus what its contacts gave the nodes) is
// summed with its moment, and averaged over the advance for the host.
//
// WHAT DOES NOT. Wheels, hydros, shocks, SUPPORT/BOUNDED beams, Lua, cab
// triangles and inter-actor contact. The legacy path uses 1/sqrt where the
// game uses fast_invSqrt.

#include "openbeam_core.h"

#include "JBeamStructure.h"

#include "BeamAxialResponse.h"
#include "CalibratedBeamJBeamDerivation.h"
#include "CalibratedBeamProductionStep.h"
#include "ContactFrictionResponse.h"
#include "ContactPenetration.h"
#include "SimConstants.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kDefaultStep = RoR::FixedStep::SECONDS; // the game's 0.5 ms
// Ten simulated seconds per call at most, so a bad dt cannot hang the host.
constexpr long kMaxStepsPerAdvance = 200000;

struct V3
{
    float x = 0.0f, y = 0.0f, z = 0.0f;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float Len(V3 a) { return std::sqrt(Dot(a, a)); }

struct D3
{
    double x = 0.0, y = 0.0, z = 0.0;
};
inline D3 operator+(D3 a, D3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline D3 operator-(D3 a, D3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline D3 operator*(D3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double Dot(D3 a, D3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline D3 Cross(D3 a, D3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline D3 ToD(V3 a) { return {a.x, a.y, a.z}; }
inline V3 ToF(D3 a) { return {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)}; }
inline D3 Load(const double* p) { return p ? D3{p[0], p[1], p[2]} : D3{}; }

// Rotation matrix of a (w, x, y, z) quaternion, normalised; identity if null
// or degenerate. Row major.
void QuatToMatrix(const double* q, double r[9])
{
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    if (q)
    {
        const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n > 1.0e-12 && std::isfinite(n))
        {
            w = q[0] / n;
            x = q[1] / n;
            y = q[2] / n;
            z = q[3] / n;
        }
    }
    r[0] = 1 - 2 * (y * y + z * z); r[1] = 2 * (x * y - w * z);     r[2] = 2 * (x * z + w * y);
    r[3] = 2 * (x * y + w * z);     r[4] = 1 - 2 * (x * x + z * z); r[5] = 2 * (y * z - w * x);
    r[6] = 2 * (x * z - w * y);     r[7] = 2 * (y * z + w * x);     r[8] = 1 - 2 * (x * x + y * y);
}
inline D3 Rotate(const double r[9], D3 v)
{
    return {r[0] * v.x + r[1] * v.y + r[2] * v.z, r[3] * v.x + r[4] * v.y + r[5] * v.z,
        r[6] * v.x + r[7] * v.y + r[8] * v.z};
}
inline D3 RotateT(const double r[9], D3 v)
{
    return {r[0] * v.x + r[3] * v.y + r[6] * v.z, r[1] * v.x + r[4] * v.y + r[7] * v.z,
        r[2] * v.x + r[5] * v.y + r[8] * v.z};
}

struct Node
{
    V3 x, v, f;
    float m = 0.0f;
    float friction_coef = 1.0f;
    bool fixed = false;
    bool collision = true;
    // A node welded to a collider does not also touch it; one just released
    // ignores it until it is clear of it, as an opening gripper lets go,
    // rather than being thrown out of an overlap it never chose.
    int held_by = -1;
    int released_from = -1;
};

struct Beam
{
    std::uint32_t a = 0, b = 0;
    float k = 0.0f, d = 0.0f, L = 0.0f, refL = 0.0f;
    float maxposstress = 0.0f, maxnegstress = 0.0f, minmaxposnegstress = 0.0f;
    float strength = 0.0f, plastic_coef = BEAM_PLASTIC_COEF_DEFAULT;
    RoR::BeamAxialResponse::RestLengthExpansionCeiling ceiling;
    bool calibrated = false;
    RoR::CalibratedBeamMaterialAdapter::Runtime runtime;
    int group = -1;
    bool broken = false;
    bool deformed = false;
    float stress = 0.0f;
};

struct Plane
{
    V3 point, normal;
    ob_ground_model gm;
};

struct Collider
{
    int shape = OB_SHAPE_SPHERE;
    double size[3] = {0.0, 0.0, 0.0};
    ob_ground_model gm;
    D3 position, velocity, angular_velocity;
    double rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    // This advance: the force on the collider and its moment about the world
    // origin, summed over steps.
    D3 sum_force, sum_moment;
    long steps = 0;
};

ob_ground_model Concrete()
{
    // resources/skeleton/config/ground_models.cfg [concrete]
    ob_ground_model gm;
    gm.ms = 1.2f;
    gm.mc = 0.75f;
    gm.va = 3.0f;
    gm.vs = 6.0f;
    gm.t2 = 0.01f;
    gm.alpha = 2.0f;
    gm.strength = 1.0f;
    return gm;
}

void WriteError(char* err, size_t len, const std::string& msg)
{
    if (err && len > 0)
    {
        std::snprintf(err, len, "%s", msg.c_str());
    }
}

} // namespace

struct ob_world
{
    std::vector<std::string> node_ids;
    std::vector<Node> nodes, rest_nodes;
    std::vector<Beam> beams, rest_beams;
    std::vector<std::vector<std::uint32_t>> groups;
    std::vector<Plane> planes;
    std::vector<Collider> colliders;
    V3 gravity{0.0f, 0.0f, -9.81f};
    double step = kDefaultStep;      // seconds per step; the laws take it as an input
    float node_radius = 0.0f;        // contact radius of every node
    float node_damping = 0.0f;       // 1/s: a force -node_damping m v on every node (off by default)
    // Nodes welded to a collider: the collider carries them, and the force
    // that takes is put on the collider.
    struct Attachment
    {
        std::uint32_t node = 0;
        int collider = -1;
        D3 local; // the node's position in the collider's frame when attached
        // A weld until released. A release makes the grip a spring-damper
        // whose stiffness fades with `strength` to zero at release_rate per
        // second, as a gripper opens: an instant release sends a stress wave
        // through a bent structure that a real one never sees.
        float strength = 1.0f;
        float release_rate = 0.0f;
        D3 hold; // the force the weld last applied: an opening grip starts from it
    };
    std::vector<Attachment> attachments;
    // Twist stiffness between two cross-sections (St-Venant torsion, as in
    // discrete elastic rods). A lattice of axial beams resists twist only
    // through diagonals that lie nearly along its axis, far less than the
    // body it stands for, and a bent bay can roll over rather than strain
    // its fibres; this supplies the torsion the lattice lacks.
    struct Twist
    {
        std::vector<std::uint32_t> a, b;
        std::vector<double> rest;           // per node pair: b's roll from a about the axis at rest
        std::vector<std::uint32_t> beams;   // the beams joining the sections: all torn, no twist
        double k = 0.0, c = 0.0;            // N m / rad, N m s / rad
        double roll = 0.0;                  // last step's roll from rest, radians
    };
    std::vector<Twist> twists;
    double carry = 0.0;
    unsigned long long steps = 0, contacts = 0;
    unsigned int skipped = 0;
    double last_advance_us = 0.0;

    void Reset()
    {
        nodes = rest_nodes;
        beams = rest_beams;
        carry = 0.0;
        steps = 0;
        contacts = 0;
        last_advance_us = 0.0;
        attachments.clear();
        for (Twist& tw : twists)
            tw.roll = 0.0;
        for (Collider& c : colliders)
        {
            c.sum_force = c.sum_moment = D3{};
            c.steps = 0;
        }
    }

    // primitiveCollision's dry branch (Collisions.cpp, static ground) against a
    // surface moving at `surface_velocity`: the force the surface gives node i.
    V3 SurfaceContact(std::size_t i, const ob_ground_model& gm, V3 normal, float penetration,
        V3 surface_velocity)
    {
        const Node& n = nodes[i];
        const V3 vrel = n.v - surface_velocity;
        const float vnormal = Dot(vrel, normal);
        const float fnormal = Dot(n.f, normal);
        const float reaction = RoR::ContactPenetration::NormalReaction(fnormal, vnormal, 0.0f - penetration,
            n.m, static_cast<float>(step), RoR::ContactPenetration::kMaxStaticSeparationSpeed);
        if (!(reaction > 0.0f))
            return V3{};
        ++contacts;
        const V3 slipf = n.f - normal * fnormal;
        V3 slip = vrel - normal * vnormal;
        const float slipv = Len(slip);
        if (slipv > 0.0f)
            slip = slip * (1.0f / slipv);
        const float g_reaction = reaction * gm.strength * n.friction_coef;
        const float ms_g = gm.ms * g_reaction;
        if (slipv < gm.va && g_reaction > 0.0f && Dot(slipf, slipf) <= ms_g * ms_g)
        {
            const RoR::ContactFrictionResponse::Result s = RoR::ContactFrictionResponse::NoReversal(
                RoR::ContactFrictionResponse::StaticSmoothing(slipv, gm.va, ms_g), n.m, slipv, step);
            const float ff = s.valid() ? -static_cast<float>(s.opposing_magnitude) : 0.0f;
            return normal * reaction + slip * ff - slipf;
        }
        const float g = gm.mc + (gm.ms - gm.mc) * std::exp(-std::pow(slipv / gm.vs, gm.alpha));
        float ff = -(g + std::min(gm.t2 * slipv, 5.0f)) * g_reaction;
        const RoR::ContactFrictionResponse::Result s = RoR::ContactFrictionResponse::SlidingNoReversal(
            -ff, n.m, slipv, Dot(slipf, slip), step);
        ff = s.valid() ? -static_cast<float>(s.opposing_magnitude) : 0.0f;
        return normal * reaction + slip * ff;
    }

    void BreakBeam(std::size_t i)
    {
        Beam& beam = beams[i];
        beam.broken = true;
        beam.stress = 0.0f;
        if (beam.group < 0)
            return;
        // BeamNG breakGroup: every beam of the group breaks with it.
        for (std::uint32_t member : groups[static_cast<std::size_t>(beam.group)])
        {
            beams[member].broken = true;
            beams[member].stress = 0.0f;
        }
    }

    void CalibratedBeam(std::size_t i)
    {
        Beam& beam = beams[i];
        Node& p1 = nodes[beam.a];
        Node& p2 = nodes[beam.b];
        RoR::CalibratedBeamProductionStep::Input input;
        input.endpoints.endpoint_1_position_m = {{p1.x.x, p1.x.y, p1.x.z}};
        input.endpoints.endpoint_2_position_m = {{p2.x.x, p2.x.y, p2.x.z}};
        input.endpoints.endpoint_1_velocity_mps = {{p1.v.x, p1.v.y, p1.v.z}};
        input.endpoints.endpoint_2_velocity_mps = {{p2.v.x, p2.v.y, p2.v.z}};
        input.reference_length_m = beam.L;
        input.damping_coefficient = beam.d;
        input.time_step = static_cast<float>(step); // the game passes its float step
        input.mass_1 = p1.m;
        input.mass_2 = p2.m;
        input.movable_1 = !p1.fixed;
        input.movable_2 = !p2.fixed;
        input.is_plain_axial_beam = true;
        const RoR::CalibratedBeamProductionStep::Result response =
            RoR::CalibratedBeamProductionStep::Step(beam.runtime, input);
        if (!response.IsValid())
        {
            // The game disables the beam (bm_disabled): it carries no force.
            beam.broken = true;
            beam.stress = 0.0f;
            return;
        }
        beam.stress = static_cast<float>(response.material.axial_force_n);
        if (response.material.yielded)
            beam.deformed = true;
        const V3 f{static_cast<float>(response.material.forces.endpoint_1[0]),
            static_cast<float>(response.material.forces.endpoint_1[1]),
            static_cast<float>(response.material.forces.endpoint_1[2])};
        p1.f = p1.f + f;
        p2.f = p2.f - f;
        if (response.material.fractured)
            BreakBeam(i);
    }

    // The ordinary path of Actor::CalcBeams (ActorForcesEuler.cpp).
    void LegacyBeam(std::size_t i)
    {
        Beam& beam = beams[i];
        Node& p1 = nodes[beam.a];
        Node& p2 = nodes[beam.b];
        const V3 dis = p1.x - p2.x;
        float dislen = Dot(dis, dis);
        if (!RoR::BeamAxialResponse::HasUsableLength(dislen))
        {
            beam.stress = 0.0f;
            return;
        }
        const float inverted_dislen = 1.0f / std::sqrt(dislen);
        dislen *= inverted_dislen;
        const float v = Dot(p1.v - p2.v, dis) * inverted_dislen;
        const float difftoBeamL = dislen - beam.L;
        const float k = beam.k;
        const RoR::BeamAxialResponse::DampingResult damping = RoR::BeamAxialResponse::ComputeDamping(
            v, beam.d, static_cast<float>(step), p1.m, p2.m, !p1.fixed, !p2.fixed);
        float slen = -k * difftoBeamL + damping.force;
        beam.stress = slen;
        float len = std::abs(slen);
        if (len > beam.minmaxposnegstress)
        {
            using RoR::BeamAxialResponse::ApplyRestLengthDeformation;
            using RoR::BeamAxialResponse::RestLengthDeformationDirection;
            if (k != 0.0f)
            {
                if (slen > beam.maxposstress && difftoBeamL < 0.0f) // compression
                {
                    const float yield_length = beam.maxposstress / k;
                    const float deform = difftoBeamL + yield_length * (1.0f - beam.plastic_coef);
                    const float Lold = beam.L;
                    const RoR::BeamAxialResponse::RestLengthDeformationResult deformation =
                        ApplyRestLengthDeformation(beam.L, deform, MIN_BEAM_LENGTH,
                            RestLengthDeformationDirection::COMPRESSION, beam.strength, k);
                    if (deformation.accepted)
                        beam.L = deformation.next_rest_length;
                    slen = slen - (slen - beam.maxposstress) * 0.5f;
                    len = slen;
                    if (deformation.accepted && deformation.applied_delta < 0.0f)
                    {
                        beam.deformed = true;
                        beam.maxposstress *= Lold / beam.L;
                        beam.minmaxposnegstress = std::min(beam.maxposstress, -beam.maxnegstress);
                        beam.minmaxposnegstress = std::min(beam.minmaxposnegstress, beam.strength);
                    }
                }
                else if (slen < beam.maxnegstress && difftoBeamL > 0.0f) // expansion
                {
                    const float yield_length = beam.maxnegstress / k;
                    const float deform = difftoBeamL + yield_length * (1.0f - beam.plastic_coef);
                    const float Lold = beam.L;
                    const RoR::BeamAxialResponse::RestLengthDeformationResult deformation =
                        ApplyRestLengthDeformation(beam.L, deform, MIN_BEAM_LENGTH,
                            RestLengthDeformationDirection::EXPANSION, beam.strength, k, beam.ceiling);
                    if (deformation.accepted)
                        beam.L = deformation.next_rest_length;
                    slen = slen - (slen - beam.maxnegstress) * 0.5f;
                    len = -slen;
                    if (deformation.accepted && deformation.applied_delta > 0.0f)
                    {
                        beam.deformed = true;
                        beam.maxnegstress *= beam.L / Lold;
                        beam.minmaxposnegstress = std::min(beam.maxposstress, -beam.maxnegstress);
                        beam.minmaxposnegstress = std::min(beam.minmaxposnegstress, beam.strength);
                        beam.strength = deformation.next_strength;
                    }
                }
            }
            // No cab triangles here, so the cab-node exception never applies.
            if (len > beam.strength)
            {
                BreakBeam(i);
                return;
            }
        }
        const V3 f = dis * (slen * inverted_dislen);
        p1.f = p1.f + f;
        p2.f = p2.f - f;
    }

    void Contacts(double t)
    {
        for (Collider& c : colliders)
        {
            ++c.steps;
        }
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            Node& n = nodes[i];
            if (n.fixed || !n.collision)
                continue;
            for (const Plane& p : planes)
            {
                const float penetration = node_radius - Dot(n.x - p.point, p.normal);
                if (penetration < 0.0f)
                    continue;
                n.f = n.f + SurfaceContact(i, p.gm, p.normal, penetration, V3{});
            }
            for (std::size_t ci = 0; ci < colliders.size(); ++ci)
            {
                Collider& c = colliders[ci];
                if (n.held_by == static_cast<int>(ci))
                    continue;
                const D3 center = c.position + c.velocity * t;
                const D3 p = ToD(n.x);
                const D3 local = RotateT(c.rotation, p - center);
                D3 normal_local;
                double penetration = 0.0;
                if (c.shape == OB_SHAPE_SPHERE || c.shape == OB_SHAPE_CAPSULE)
                {
                    D3 axis_point;
                    if (c.shape == OB_SHAPE_CAPSULE)
                        axis_point.z = std::max(-c.size[1], std::min(c.size[1], local.z));
                    const D3 d = local - axis_point;
                    const double dist = std::sqrt(Dot(d, d));
                    const double reach = c.size[0] + node_radius;
                    if (!(dist < reach))
                    {
                        if (n.released_from == static_cast<int>(ci))
                            n.released_from = -1; // clear of the collider that let it go
                        continue;
                    }
                    normal_local = dist > 1.0e-9 ? d * (1.0 / dist) : D3{0.0, 0.0, 1.0};
                    penetration = reach - dist;
                }
                else
                {
                    const double l[3] = {local.x, local.y, local.z};
                    int best = -1;
                    double best_depth = 0.0;
                    for (int a = 0; a < 3; ++a)
                    {
                        const double depth = c.size[a] + node_radius - std::abs(l[a]);
                        if (!(depth > 0.0))
                        {
                            best = -1;
                            break;
                        }
                        if (best < 0 || depth < best_depth)
                        {
                            best = a;
                            best_depth = depth;
                        }
                    }
                    if (best < 0)
                    {
                        if (n.released_from == static_cast<int>(ci))
                            n.released_from = -1;
                        continue;
                    }
                    double nl[3] = {0.0, 0.0, 0.0};
                    nl[best] = l[best] < 0.0 ? -1.0 : 1.0;
                    normal_local = {nl[0], nl[1], nl[2]};
                    penetration = best_depth;
                }
                if (n.released_from == static_cast<int>(ci))
                    continue; // still inside the collider that let it go
                const D3 normal = Rotate(c.rotation, normal_local);
                const D3 surface_velocity = c.velocity + Cross(c.angular_velocity, p - center);
                const V3 f = SurfaceContact(i, c.gm, ToF(normal), static_cast<float>(penetration),
                    ToF(surface_velocity));
                n.f = n.f + f;
                const D3 on_collider = ToD(f) * -1.0;
                c.sum_force = c.sum_force + on_collider;
                c.sum_moment = c.sum_moment + Cross(p, on_collider);
            }
        }
    }

    // A welded node lands, after this step's kick and drift, on the point of
    // its collider it was attached at; the force that takes goes on the collider.
    void Attachments(double t)
    {
        for (Attachment& a : attachments)
        {
            Node& n = nodes[a.node];
            Collider& c = colliders[static_cast<std::size_t>(a.collider)];
            const double t1 = t + step;
            const D3 arm = Rotate(c.rotation, a.local);
            // Within an advance the collider's orientation is held; its spin
            // moves the carried point to first order.
            D3 f_needed;
            if (a.release_rate > 0.0f)
            {
                // Opening: the weld's last force, fading, and a critically
                // damped spring to the carried point, stiff at first (natural
                // frequency 0.5 / step) and softening to nothing. The spring
                // and damper act on the end-of-step state (backward Euler), so
                // they add no explicit stiffness or damping to a node whose
                // beams already use the step's stability margin.
                const double m = static_cast<double>(n.m);
                const double w0 = 0.5 / step;
                const double k = m * w0 * w0 * static_cast<double>(a.strength);
                const double damp = 2.0 * std::sqrt(k * m);
                const D3 point = c.position + c.velocity * t1 + arm + Cross(c.angular_velocity, arm) * t1;
                const D3 point_velocity = c.velocity + Cross(c.angular_velocity, arm);
                const D3 v = ToD(n.v), f_other = ToD(n.f);
                const D3 rhs = v + (f_other + a.hold * static_cast<double>(a.strength) + (point - ToD(n.x)) * k +
                    point_velocity * damp) * (step / m);
                const D3 v_new = rhs * (1.0 / (1.0 + step * step * k / m + step * damp / m));
                f_needed = (v_new - v) * (m / step) - f_other;
            }
            else
            {
                const D3 target = c.position + c.velocity * t1 + arm + Cross(c.angular_velocity, arm) * t1;
                const D3 v_new = (target - ToD(n.x)) * (1.0 / step);
                f_needed = (v_new - ToD(n.v)) * (static_cast<double>(n.m) / step) - ToD(n.f);
                a.hold = f_needed;
            }
            n.f = n.f + ToF(f_needed);
            const D3 on_collider = f_needed * -1.0;
            c.sum_force = c.sum_force + on_collider;
            c.sum_moment = c.sum_moment + Cross(ToD(n.x), on_collider);
        }
    }

    // A section's centre and mean velocity, its nodes' offsets from the centre
    // across the line joining two centres, and its spin about that line.
    struct SectionFrame
    {
        D3 centre, mean_v;
        std::vector<D3> u;
        double inertia_arm = 0.0; // sum of |u|^2
        double spin = 0.0;
    };
    static void Frame(const std::vector<std::uint32_t>& ids, const std::vector<Node>& ns, SectionFrame& f)
    {
        f.centre = f.mean_v = D3{};
        for (std::uint32_t i : ids)
        {
            f.centre = f.centre + ToD(ns[i].x);
            f.mean_v = f.mean_v + ToD(ns[i].v);
        }
        const double inv = 1.0 / static_cast<double>(ids.size());
        f.centre = f.centre * inv;
        f.mean_v = f.mean_v * inv;
    }
    static void Across(const std::vector<std::uint32_t>& ids, const std::vector<Node>& ns, D3 axis, SectionFrame& f)
    {
        f.u.resize(ids.size());
        f.inertia_arm = 0.0;
        double spin = 0.0;
        for (std::size_t j = 0; j < ids.size(); ++j)
        {
            D3 d = ToD(ns[ids[j]].x) - f.centre;
            d = d - axis * Dot(d, axis);
            f.u[j] = d;
            f.inertia_arm += Dot(d, d);
            spin += Dot(Cross(d, ToD(ns[ids[j]].v) - f.mean_v), axis);
        }
        f.spin = f.inertia_arm > 0.0 ? spin / f.inertia_arm : 0.0;
    }
    // b's roll about the axis relative to a, less each pair's rest roll.
    static double Roll(const SectionFrame& fa, const SectionFrame& fb, D3 axis, const std::vector<double>& rest)
    {
        double sn = 0.0, cs = 0.0;
        for (std::size_t j = 0; j < fa.u.size(); ++j)
        {
            const D3 ua = fa.u[j] * std::cos(rest[j]) + Cross(axis, fa.u[j]) * std::sin(rest[j]);
            sn += Dot(Cross(ua, fb.u[j]), axis);
            cs += Dot(ua, fb.u[j]);
        }
        return std::atan2(sn, cs);
    }

    // Equal and opposite torques about the line joining the centres, put on
    // each section as forces across it that sum to zero.
    void Twists()
    {
        SectionFrame fa, fb;
        for (Twist& tw : twists)
        {
            bool joined = tw.beams.empty();
            for (std::uint32_t b : tw.beams)
                joined = joined || !beams[b].broken;
            if (!joined)
                continue;
            Frame(tw.a, nodes, fa);
            Frame(tw.b, nodes, fb);
            D3 axis = fb.centre - fa.centre;
            const double len = std::sqrt(Dot(axis, axis));
            if (!(len > 1.0e-9))
                continue;
            axis = axis * (1.0 / len);
            Across(tw.a, nodes, axis, fa);
            Across(tw.b, nodes, axis, fb);
            if (!(fa.inertia_arm > 0.0 && fb.inertia_arm > 0.0))
                continue;
            tw.roll = Roll(fa, fb, axis, tw.rest);
            const double torque_b = -tw.k * tw.roll - tw.c * (fb.spin - fa.spin);
            for (std::size_t j = 0; j < tw.a.size(); ++j)
            {
                nodes[tw.a[j]].f = nodes[tw.a[j]].f + ToF(Cross(axis, fa.u[j]) * (-torque_b / fa.inertia_arm));
                nodes[tw.b[j]].f = nodes[tw.b[j]].f + ToF(Cross(axis, fb.u[j]) * (torque_b / fb.inertia_arm));
            }
        }
    }

    // Fades releasing grips and drops the ones that have let go.
    void UpdateReleases()
    {
        bool done = false;
        for (Attachment& a : attachments)
        {
            if (a.release_rate > 0.0f)
            {
                a.strength -= a.release_rate * static_cast<float>(step);
                if (a.strength <= 0.5f * a.release_rate * static_cast<float>(step))
                    a.strength = 0.0f; // done: float steps must not leave a sliver of grip
                if (a.strength <= 0.0f)
                {
                    nodes[a.node].held_by = -1;
                    nodes[a.node].released_from = a.collider;
                    done = true;
                }
            }
        }
        if (done)
            attachments.erase(std::remove_if(attachments.begin(), attachments.end(),
                [](const Attachment& a) { return a.strength <= 0.0f; }), attachments.end());
    }

    void Step(double t)
    {
        for (Node& n : nodes)
            n.f = gravity * n.m;
        for (std::size_t i = 0; i < beams.size(); ++i)
        {
            if (beams[i].broken)
                continue;
            if (beams[i].calibrated)
                CalibratedBeam(i);
            else
                LegacyBeam(i);
        }
        Twists();
        if (node_damping > 0.0f)
        {
            for (Node& n : nodes)
                n.f = n.f - n.v * (node_damping * n.m);
        }
        Contacts(t);
        Attachments(t);
        UpdateReleases();
        const float h = static_cast<float>(step);
        for (Node& n : nodes)
        {
            if (n.fixed)
                continue;
            n.v = n.v + n.f * (h / n.m);
            n.x = n.x + n.v * h;
        }
        ++steps;
    }
};

namespace {

bool BuildWorld(const OpenBeamCore::Structure& s, const double* position, const double* quaternion,
    ob_world& w, std::string& error)
{
    double r[9];
    QuatToMatrix(quaternion, r);
    const D3 origin = Load(position);
    w.node_ids.reserve(s.nodes.size());
    w.rest_nodes.reserve(s.nodes.size());
    for (const OpenBeamCore::StructureNode& sn : s.nodes)
    {
        if (!(sn.mass > 0.0) || !std::isfinite(sn.mass))
        {
            error = "node '" + sn.id + "' has no positive nodeWeight";
            return false;
        }
        Node n;
        n.x = ToF(Rotate(r, D3{sn.position[0], sn.position[1], sn.position[2]}) + origin);
        n.m = static_cast<float>(sn.mass);
        n.friction_coef = static_cast<float>(sn.friction_coef);
        n.fixed = sn.fixed;
        n.collision = sn.collision;
        w.node_ids.push_back(sn.id);
        w.rest_nodes.push_back(n);
    }

    std::map<std::string, int> group_ids;
    for (const OpenBeamCore::StructureBeam& sb : s.beams)
    {
        Beam b;
        b.a = static_cast<std::uint32_t>(sb.node_a);
        b.b = static_cast<std::uint32_t>(sb.node_b);
        const float geometric = Len(w.rest_nodes[b.a].x - w.rest_nodes[b.b].x);
        // ActorSpawner: L and refL both carry the precompression; a
        // precompressionRange (metres) supersedes the multiplier.
        const float rest = sb.has_precompression_range
            ? geometric + static_cast<float>(sb.precompression_range)
            : geometric * static_cast<float>(sb.precompression);
        if (!(rest > 0.0f))
            continue;
        b.L = b.refL = rest;
        b.k = static_cast<float>(sb.spring);
        b.d = static_cast<float>(sb.damping);
        b.maxposstress = static_cast<float>(sb.deform);
        b.maxnegstress = -static_cast<float>(sb.deform);
        b.strength = static_cast<float>(sb.strength);
        b.minmaxposnegstress = RoR::BeamAxialResponse::YieldOrBreakGate(b.maxposstress, b.maxnegstress, b.strength);
        if (sb.has_deform_limit_expansion && sb.deform_limit_expansion > 0.0)
        {
            b.ceiling.declared = true;
            b.ceiling.maximum_rest_length = rest * static_cast<float>(sb.deform_limit_expansion);
        }
        // JBeamToRigDef.cpp opts every non-negative spring into the calibrated
        // material; RigDef_CalibratedBeamMaterial.h derives it with every
        // field marked authored. A beam it refuses keeps the legacy law.
        if (sb.spring >= 0.0)
        {
            RoR::CalibratedBeamJBeamDerivation::AuthoredBeam authored;
            authored.spring_n_per_m = b.k;
            authored.has_spring = true;
            authored.deform_n = b.maxposstress;
            authored.has_deform = true;
            authored.rest_length_m = b.L;
            authored.has_strength = true;
            authored.strength_n = b.strength;
            authored.strength_unbounded = sb.strength_unbounded;
            authored.deform_unbounded = sb.deform_unbounded;
            const RoR::CalibratedBeamJBeamDerivation::DeclaredBasis basis;
            RoR::CalibratedBeamMaterialAdapter::Configuration configuration;
            RoR::CalibratedBeamJBeamDerivation::DerivationError derivation_error;
            if (RoR::CalibratedBeamJBeamDerivation::DeriveFromAuthoredBeam(
                    authored, basis, configuration, derivation_error))
            {
                b.calibrated = RoR::CalibratedBeamMaterialAdapter::TryConfigure(b.runtime, configuration);
            }
        }
        if (!sb.break_group.empty())
        {
            const auto it = group_ids.emplace(sb.break_group, static_cast<int>(group_ids.size())).first;
            b.group = it->second;
            if (w.groups.size() <= static_cast<std::size_t>(b.group))
                w.groups.resize(static_cast<std::size_t>(b.group) + 1);
            w.groups[static_cast<std::size_t>(b.group)].push_back(static_cast<std::uint32_t>(w.rest_beams.size()));
        }
        w.rest_beams.push_back(b);
    }
    w.skipped = static_cast<unsigned int>(s.skipped_beams);
    w.Reset();
    return true;
}

ob_world* Create(const std::string& text, const std::string& path, const double* position,
    const double* quaternion, char* err, size_t err_len)
{
    OpenBeamCore::Structure structure;
    std::string error;
    if (!OpenBeamCore::LoadJBeamStructure(text, path, structure, error))
    {
        WriteError(err, err_len, error);
        return nullptr;
    }
    ob_world* world = new ob_world();
    if (!BuildWorld(structure, position, quaternion, *world, error))
    {
        delete world;
        WriteError(err, err_len, error);
        return nullptr;
    }
    WriteError(err, err_len, "");
    return world;
}

inline std::uint64_t Fnv(std::uint64_t h, const void* data, std::size_t n)
{
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i)
    {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

extern "C" {

int ob_core_api_version(void) { return OB_CORE_API_VERSION; }

ob_ground_model ob_ground_model_concrete(void) { return Concrete(); }

ob_world* ob_world_create_from_jbeam_text(const char* text, const char* virtual_path,
    const double position[3], const double quaternion[4], char* err, size_t err_len)
{
    if (!text)
    {
        WriteError(err, err_len, "no JBeam text");
        return nullptr;
    }
    try
    {
        return Create(text, virtual_path ? virtual_path : "openbeam/structure.jbeam", position, quaternion,
            err, err_len);
    }
    catch (const std::exception& e)
    {
        WriteError(err, err_len, std::string("exception: ") + e.what());
        return nullptr;
    }
}

ob_world* ob_world_create_from_jbeam_file(const char* path, const double position[3],
    const double quaternion[4], char* err, size_t err_len)
{
    if (!path)
    {
        WriteError(err, err_len, "no path");
        return nullptr;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        WriteError(err, err_len, std::string("cannot read ") + path);
        return nullptr;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    std::string name(path);
    std::replace(name.begin(), name.end(), '\\', '/');
    return ob_world_create_from_jbeam_text(buffer.str().c_str(), name.c_str(), position, quaternion, err,
        err_len);
}

void ob_world_destroy(ob_world* world) { delete world; }

ob_world* ob_world_clone(const ob_world* world) { return world ? new ob_world(*world) : nullptr; }

void ob_world_reset(ob_world* world)
{
    if (world)
        world->Reset();
}

void ob_world_set_gravity(ob_world* world, const double gravity[3])
{
    if (world && gravity)
        world->gravity = ToF(Load(gravity));
}

int ob_world_add_plane(ob_world* world, const double point[3], const double normal[3],
    const ob_ground_model* ground_model)
{
    if (!world || !point || !normal)
        return -1;
    D3 n = Load(normal);
    const double len = std::sqrt(Dot(n, n));
    if (!(len > 1.0e-12))
        return -1;
    Plane p;
    p.point = ToF(Load(point));
    p.normal = ToF(n * (1.0 / len));
    p.gm = ground_model ? *ground_model : Concrete();
    world->planes.push_back(p);
    return static_cast<int>(world->planes.size() - 1);
}

int ob_world_add_collider(ob_world* world, ob_shape shape, const double size[3],
    const ob_ground_model* ground_model)
{
    if (!world || !size)
        return -1;
    Collider c;
    c.shape = shape;
    for (int i = 0; i < 3; ++i)
        c.size[i] = size[i];
    if (shape == OB_SHAPE_SPHERE)
    {
        if (!(size[0] > 0.0))
            return -1;
    }
    else if (shape == OB_SHAPE_BOX)
    {
        if (!(size[0] > 0.0 && size[1] > 0.0 && size[2] > 0.0))
            return -1;
    }
    else if (shape == OB_SHAPE_CAPSULE)
    {
        if (!(size[0] > 0.0 && size[1] >= 0.0))
            return -1;
    }
    else
    {
        return -1;
    }
    c.gm = ground_model ? *ground_model : Concrete();
    world->colliders.push_back(c);
    return static_cast<int>(world->colliders.size() - 1);
}

void ob_world_set_collider_state(ob_world* world, int collider, const double position[3],
    const double quaternion[4], const double linear_velocity[3], const double angular_velocity[3])
{
    if (!world || collider < 0 || static_cast<std::size_t>(collider) >= world->colliders.size())
        return;
    Collider& c = world->colliders[static_cast<std::size_t>(collider)];
    c.position = Load(position);
    QuatToMatrix(quaternion, c.rotation);
    c.velocity = Load(linear_velocity);
    c.angular_velocity = Load(angular_velocity);
}

int ob_world_set_beam_material(ob_world* world, int beam, const ob_beam_material* material)
{
    if (!world || !material || beam < 0 || static_cast<std::size_t>(beam) >= world->beams.size())
        return -1;
    RoR::CalibratedBeamMaterialAdapter::Configuration configuration;
    configuration.cross_section_area_m2 = material->area_m2;
    configuration.material.elastic_modulus = material->elastic_modulus_pa;
    configuration.material.yield_stress = material->yield_stress_pa;
    configuration.material.hardening_modulus = material->hardening_modulus_pa;
    configuration.material.damage_onset_plastic_strain = material->damage_onset_plastic_strain;
    configuration.material.damage_driver_capacity_density = material->damage_capacity_j_per_m3;
    // Configure a copy first: a refused material leaves the beam as it was.
    Beam configured = world->rest_beams[static_cast<std::size_t>(beam)];
    configured.runtime = RoR::CalibratedBeamMaterialAdapter::Runtime{};
    if (!RoR::CalibratedBeamMaterialAdapter::TryConfigure(configured.runtime, configuration))
        return -1;
    configured.calibrated = true;
    // The axial stiffness the legacy readouts and the stability estimate use.
    configured.k = static_cast<float>(material->elastic_modulus_pa * material->area_m2 / configured.L);
    world->rest_beams[static_cast<std::size_t>(beam)] = configured;
    Beam& live = world->beams[static_cast<std::size_t>(beam)];
    const Beam history = live;
    live = configured;
    live.L = history.L;
    live.broken = history.broken;
    live.deformed = false;
    return 0;
}

int ob_world_load_materials(ob_world* world, const char* path, char* err, size_t err_len)
{
    if (!world || !path)
        return -1;
    std::FILE* file = std::fopen(path, "rb");
    if (!file)
    {
        WriteError(err, err_len, std::string("cannot open ") + path);
        return -1;
    }
    std::string text;
    char chunk[4096];
    for (std::size_t n; (n = std::fread(chunk, 1, sizeof(chunk), file)) > 0;)
        text.append(chunk, n);
    std::fclose(file);
    int configured = 0, line_number = 0;
    std::size_t start = 0;
    while (start < text.size())
    {
        std::size_t end = text.find('\n', start);
        if (end == std::string::npos)
            end = text.size();
        std::string line = text.substr(start, end - start);
        start = end + 1;
        ++line_number;
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos)
            line.resize(hash);
        char word[16] = {0};
        if (std::sscanf(line.c_str(), " %15s", word) == 1 && std::strcmp(word, "twist") == 0)
        {
            long twist_first = 0, per = 0, sections = 0;
            double gj = 0.0, ratio = 0.0;
            if (std::sscanf(line.c_str(), " twist %ld %ld %ld %lf %lf", &twist_first, &per, &sections, &gj, &ratio) != 5 ||
                ob_world_add_rod_twist(world, static_cast<int>(twist_first), static_cast<int>(per),
                    static_cast<int>(sections), gj, ratio) < 0)
            {
                WriteError(err, err_len, std::string(path) + ":" + std::to_string(line_number) +
                    ": expected 'twist first_node nodes_per_section sections gj_n_m2 damping_ratio' within the nodes");
                return -1;
            }
            continue;
        }
        long first = 0, count = 0;
        ob_beam_material m{};
        const int read = std::sscanf(line.c_str(), "%ld %ld %lf %lf %lf %lf %lf %lf", &first, &count, &m.area_m2,
            &m.elastic_modulus_pa, &m.yield_stress_pa, &m.hardening_modulus_pa, &m.damage_onset_plastic_strain,
            &m.damage_capacity_j_per_m3);
        if (read <= 0)
            continue; // blank or comment
        if (read != 8 || first < 0 || count < 0 ||
            static_cast<std::size_t>(first + count) > world->beams.size())
        {
            WriteError(err, err_len, std::string(path) + ":" + std::to_string(line_number) +
                ": expected 'first count area E yield hardening damage_onset capacity' within the beams");
            return -1;
        }
        for (long b = first; b < first + count; ++b)
        {
            if (ob_world_set_beam_material(world, static_cast<int>(b), &m) != 0)
            {
                WriteError(err, err_len, std::string(path) + ":" + std::to_string(line_number) +
                    ": the material refused these parameters");
                return -1;
            }
            ++configured;
        }
    }
    return configured;
}

int ob_world_add_twist(ob_world* world, const int* section_a, const int* section_b, int count, double stiffness,
    double damping)
{
    if (!world || !section_a || !section_b || count < 3 || !(stiffness >= 0.0) || !(damping >= 0.0) ||
        !std::isfinite(stiffness) || !std::isfinite(damping))
        return -1;
    ob_world::Twist tw;
    const int n = static_cast<int>(world->nodes.size());
    for (int j = 0; j < count; ++j)
    {
        if (section_a[j] < 0 || section_a[j] >= n || section_b[j] < 0 || section_b[j] >= n ||
            section_a[j] == section_b[j])
            return -1;
        tw.a.push_back(static_cast<std::uint32_t>(section_a[j]));
        tw.b.push_back(static_cast<std::uint32_t>(section_b[j]));
    }
    // Each pair's rest roll, from the structure as created.
    ob_world::SectionFrame fa, fb;
    ob_world::Frame(tw.a, world->rest_nodes, fa);
    ob_world::Frame(tw.b, world->rest_nodes, fb);
    D3 axis = fb.centre - fa.centre;
    const double len = std::sqrt(Dot(axis, axis));
    if (!(len > 1.0e-9))
        return -1;
    axis = axis * (1.0 / len);
    ob_world::Across(tw.a, world->rest_nodes, axis, fa);
    ob_world::Across(tw.b, world->rest_nodes, axis, fb);
    for (int j = 0; j < count; ++j)
    {
        if (!(Dot(fa.u[j], fa.u[j]) > 1.0e-18 && Dot(fb.u[j], fb.u[j]) > 1.0e-18))
            return -1; // a node on the axis has no roll
        tw.rest.push_back(std::atan2(Dot(Cross(fa.u[j], fb.u[j]), axis), Dot(fa.u[j], fb.u[j])));
    }
    for (std::size_t i = 0; i < world->beams.size(); ++i)
    {
        const Beam& b = world->beams[i];
        const auto in = [&b](const std::vector<std::uint32_t>& ids) {
            return std::find(ids.begin(), ids.end(), b.a) != ids.end() ||
                std::find(ids.begin(), ids.end(), b.b) != ids.end();
        };
        if (in(tw.a) && in(tw.b))
            tw.beams.push_back(static_cast<std::uint32_t>(i));
    }
    tw.k = stiffness;
    tw.c = damping;
    world->twists.push_back(std::move(tw));
    return static_cast<int>(world->twists.size() - 1);
}

int ob_world_add_rod_twist(ob_world* world, int first_node, int per_section, int sections, double gj,
    double damping_ratio)
{
    if (!world || first_node < 0 || per_section < 3 || sections < 2 || !(gj >= 0.0) || !(damping_ratio >= 0.0) ||
        !std::isfinite(gj) || !std::isfinite(damping_ratio) ||
        static_cast<long long>(first_node) + static_cast<long long>(per_section) * sections >
            static_cast<long long>(world->nodes.size()))
        return -1;
    std::vector<int> a(static_cast<std::size_t>(per_section)), b(a.size());
    for (int i = 0; i + 1 < sections; ++i)
    {
        double inertia[2] = {0.0, 0.0};
        D3 centre[2];
        for (int side = 0; side < 2; ++side)
        {
            for (int j = 0; j < per_section; ++j)
                centre[side] = centre[side] + ToD(world->rest_nodes[first_node + (i + side) * per_section + j].x);
            centre[side] = centre[side] * (1.0 / per_section);
        }
        D3 axis = centre[1] - centre[0];
        const double distance = std::sqrt(Dot(axis, axis));
        if (!(distance > 1.0e-9))
            return -1;
        axis = axis * (1.0 / distance);
        for (int side = 0; side < 2; ++side)
        {
            for (int j = 0; j < per_section; ++j)
            {
                const Node& n = world->rest_nodes[first_node + (i + side) * per_section + j];
                D3 d = ToD(n.x) - centre[side];
                d = d - axis * Dot(d, axis);
                inertia[side] += static_cast<double>(n.m) * Dot(d, d);
            }
        }
        // Torsion GJ over the length between the centres; damping a fraction
        // of the pair's critical damping.
        const double k = gj / distance;
        const double reduced = inertia[0] + inertia[1] > 0.0 ? inertia[0] * inertia[1] / (inertia[0] + inertia[1]) : 0.0;
        for (int j = 0; j < per_section; ++j)
        {
            a[j] = first_node + i * per_section + j;
            b[j] = first_node + (i + 1) * per_section + j;
        }
        if (ob_world_add_twist(world, a.data(), b.data(), per_section, k,
                2.0 * damping_ratio * std::sqrt(k * reduced)) < 0)
            return -1;
    }
    return sections - 1;
}

int ob_world_twist_count(const ob_world* world) { return world ? static_cast<int>(world->twists.size()) : 0; }

void ob_world_get_twist(const ob_world* world, double* roll)
{
    if (!world || !roll)
        return;
    for (std::size_t i = 0; i < world->twists.size(); ++i)
        roll[i] = world->twists[i].roll;
}

int ob_world_set_step(ob_world* world, double step)
{
    if (!world || !std::isfinite(step) || !(step >= 1.0e-6 && step <= kDefaultStep))
        return -1;
    world->step = step;
    world->carry = 0.0;
    return 0;
}

double ob_world_get_step(const ob_world* world) { return world ? world->step : 0.0; }

void ob_world_set_node_radius(ob_world* world, double radius)
{
    if (world && radius >= 0.0 && std::isfinite(radius))
        world->node_radius = static_cast<float>(radius);
}

void ob_world_set_node_damping(ob_world* world, double per_second)
{
    if (world && per_second >= 0.0 && std::isfinite(per_second))
        world->node_damping = static_cast<float>(per_second);
}

int ob_world_find_node(const ob_world* world, const char* id)
{
    if (!world || !id)
        return -1;
    for (std::size_t i = 0; i < world->node_ids.size(); ++i)
    {
        if (world->node_ids[i] == id)
            return static_cast<int>(i);
    }
    return -1;
}

int ob_world_detach_node(ob_world* world, int node)
{
    if (!world)
        return -1;
    const std::size_t before = world->attachments.size();
    for (const ob_world::Attachment& a : world->attachments)
    {
        if (static_cast<int>(a.node) == node)
        {
            world->nodes[a.node].held_by = -1;
            world->nodes[a.node].released_from = a.collider;
        }
    }
    world->attachments.erase(std::remove_if(world->attachments.begin(), world->attachments.end(),
        [node](const ob_world::Attachment& a) { return static_cast<int>(a.node) == node; }),
        world->attachments.end());
    return world->attachments.size() < before ? 0 : -1;
}

int ob_world_attach_node(ob_world* world, int node, int collider)
{
    if (!world || node < 0 || static_cast<std::size_t>(node) >= world->nodes.size() || collider < 0 ||
        static_cast<std::size_t>(collider) >= world->colliders.size() ||
        world->nodes[static_cast<std::size_t>(node)].fixed)
        return -1;
    ob_world_detach_node(world, node);
    const Collider& c = world->colliders[static_cast<std::size_t>(collider)];
    ob_world::Attachment a;
    a.node = static_cast<std::uint32_t>(node);
    a.collider = collider;
    a.local = RotateT(c.rotation, ToD(world->nodes[static_cast<std::size_t>(node)].x) - c.position);
    world->attachments.push_back(a);
    world->nodes[static_cast<std::size_t>(node)].held_by = collider;
    world->nodes[static_cast<std::size_t>(node)].released_from = -1;
    return 0;
}

int ob_world_release_node(ob_world* world, int node, double seconds)
{
    if (!world || !std::isfinite(seconds) || seconds < 0.0)
        return -1;
    if (seconds == 0.0)
        return ob_world_detach_node(world, node);
    int found = -1;
    for (ob_world::Attachment& a : world->attachments)
    {
        if (static_cast<int>(a.node) == node)
        {
            a.release_rate = static_cast<float>(a.strength / seconds);
            found = 0;
        }
    }
    return found;
}

void ob_world_detach_all(ob_world* world)
{
    if (!world)
        return;
    for (const ob_world::Attachment& a : world->attachments)
    {
        world->nodes[a.node].held_by = -1;
        world->nodes[a.node].released_from = a.collider;
    }
    world->attachments.clear();
}

int ob_world_attached_count(const ob_world* world)
{
    return world ? static_cast<int>(world->attachments.size()) : 0;
}

int ob_world_advance(ob_world* world, double dt)
{
    if (!world || !(dt > 0.0) || !std::isfinite(dt))
        return 0;
    const auto t0 = std::chrono::steady_clock::now();
    world->carry += dt;
    const double h = world->step;
    long n = static_cast<long>(std::floor(world->carry / h + 1.0e-6));
    n = std::max(0L, std::min(n, kMaxStepsPerAdvance));
    world->carry = std::max(0.0, world->carry - static_cast<double>(n) * h);
    if (world->carry > h)
        world->carry = 0.0; // clamped by kMaxStepsPerAdvance: drop the backlog
    for (Collider& c : world->colliders)
    {
        c.sum_force = c.sum_moment = D3{};
        c.steps = 0;
    }
    for (long i = 0; i < n; ++i)
        world->Step(static_cast<double>(i) * h);
    world->last_advance_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    return static_cast<int>(n);
}

void ob_world_get_collider_wrench(const ob_world* world, int collider, const double reference_point[3],
    double force[3], double torque[3])
{
    D3 f, t;
    if (world && collider >= 0 && static_cast<std::size_t>(collider) < world->colliders.size())
    {
        const Collider& c = world->colliders[static_cast<std::size_t>(collider)];
        if (c.steps > 0)
        {
            const double inv = 1.0 / static_cast<double>(c.steps);
            f = c.sum_force * inv;
            t = (c.sum_moment - Cross(Load(reference_point), c.sum_force)) * inv;
        }
    }
    if (force)
    {
        force[0] = f.x;
        force[1] = f.y;
        force[2] = f.z;
    }
    if (torque)
    {
        torque[0] = t.x;
        torque[1] = t.y;
        torque[2] = t.z;
    }
}

int ob_world_node_count(const ob_world* world) { return world ? static_cast<int>(world->nodes.size()) : 0; }

int ob_world_beam_count(const ob_world* world) { return world ? static_cast<int>(world->beams.size()) : 0; }

const char* ob_world_node_id(const ob_world* world, int node)
{
    if (!world || node < 0 || static_cast<std::size_t>(node) >= world->node_ids.size())
        return "";
    return world->node_ids[static_cast<std::size_t>(node)].c_str();
}

double ob_world_total_mass(const ob_world* world)
{
    double m = 0.0;
    if (world)
        for (const Node& n : world->nodes)
            m += n.m;
    return m;
}

void ob_world_get_node_positions(const ob_world* world, double* xyz)
{
    if (!world || !xyz)
        return;
    for (std::size_t i = 0; i < world->nodes.size(); ++i)
    {
        xyz[3 * i + 0] = world->nodes[i].x.x;
        xyz[3 * i + 1] = world->nodes[i].x.y;
        xyz[3 * i + 2] = world->nodes[i].x.z;
    }
}

void ob_world_get_node_velocities(const ob_world* world, double* xyz)
{
    if (!world || !xyz)
        return;
    for (std::size_t i = 0; i < world->nodes.size(); ++i)
    {
        xyz[3 * i + 0] = world->nodes[i].v.x;
        xyz[3 * i + 1] = world->nodes[i].v.y;
        xyz[3 * i + 2] = world->nodes[i].v.z;
    }
}

void ob_world_get_beam_nodes(const ob_world* world, int* pairs)
{
    if (!world || !pairs)
        return;
    for (std::size_t i = 0; i < world->beams.size(); ++i)
    {
        pairs[2 * i + 0] = static_cast<int>(world->beams[i].a);
        pairs[2 * i + 1] = static_cast<int>(world->beams[i].b);
    }
}

void ob_world_get_beam_state(const ob_world* world, float* stress, float* rest_length, float* strength,
    unsigned char* broken)
{
    if (!world)
        return;
    for (std::size_t i = 0; i < world->beams.size(); ++i)
    {
        const Beam& b = world->beams[i];
        if (stress)
            stress[i] = b.stress;
        if (rest_length)
            rest_length[i] = b.L;
        if (strength)
            strength[i] = b.strength;
        if (broken)
            broken[i] = b.broken ? 1 : 0;
    }
}

void ob_world_get_beam_damage(const ob_world* world, float* plastic_strain, float* damage)
{
    if (!world)
        return;
    for (std::size_t i = 0; i < world->beams.size(); ++i)
    {
        const Beam& b = world->beams[i];
        float strain = 0.0f;
        float d = b.broken ? 1.0f : 0.0f;
        if (b.calibrated)
        {
            strain = static_cast<float>(b.runtime.state.plastic_strain);
            d = std::max(d, static_cast<float>(b.runtime.state.damage));
        }
        else if (b.refL > 0.0f)
        {
            strain = (b.L - b.refL) / b.refL;
        }
        if (plastic_strain)
            plastic_strain[i] = strain;
        if (damage)
            damage[i] = d;
    }
}

void ob_world_get_stats(const ob_world* world, ob_stats* stats)
{
    if (!stats)
        return;
    std::memset(stats, 0, sizeof(*stats));
    if (!world)
        return;
    stats->steps = world->steps;
    stats->contacts = world->contacts;
    stats->beams_skipped = world->skipped;
    stats->last_advance_us = world->last_advance_us;
    for (const Beam& b : world->beams)
    {
        if (b.broken)
            ++stats->beams_broken;
        if (b.deformed)
            ++stats->beams_deformed;
    }
}

unsigned long long ob_world_state_digest(const ob_world* world)
{
    std::uint64_t h = 1469598103934665603ULL;
    if (!world)
        return h;
    for (const Node& n : world->nodes)
    {
        h = Fnv(h, &n.x, sizeof(n.x));
        h = Fnv(h, &n.v, sizeof(n.v));
    }
    for (const Beam& b : world->beams)
    {
        h = Fnv(h, &b.L, sizeof(b.L));
        h = Fnv(h, &b.stress, sizeof(b.stress));
        const unsigned char flags = static_cast<unsigned char>((b.broken ? 1 : 0) | (b.deformed ? 2 : 0));
        h = Fnv(h, &flags, 1);
    }
    return h;
}

} // extern "C"
