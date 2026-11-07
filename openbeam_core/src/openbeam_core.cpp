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

constexpr float kDt = RoR::FixedStep::SECONDS_F;
constexpr double kDtD = RoR::FixedStep::SECONDS;
// Ten simulated seconds per call at most, so a bad dt cannot hang the host.
constexpr long kMaxStepsPerAdvance = 20000;

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
            n.m, kDt, RoR::ContactPenetration::kMaxStaticSeparationSpeed);
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
                RoR::ContactFrictionResponse::StaticSmoothing(slipv, gm.va, ms_g), n.m, slipv, kDtD);
            const float ff = s.valid() ? -static_cast<float>(s.opposing_magnitude) : 0.0f;
            return normal * reaction + slip * ff - slipf;
        }
        const float g = gm.mc + (gm.ms - gm.mc) * std::exp(-std::pow(slipv / gm.vs, gm.alpha));
        float ff = -(g + std::min(gm.t2 * slipv, 5.0f)) * g_reaction;
        const RoR::ContactFrictionResponse::Result s = RoR::ContactFrictionResponse::SlidingNoReversal(
            -ff, n.m, slipv, Dot(slipf, slip), kDtD);
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
        input.time_step = kDt;
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
            v, beam.d, kDt, p1.m, p2.m, !p1.fixed, !p2.fixed);
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
                const float penetration = -Dot(n.x - p.point, p.normal);
                if (penetration < 0.0f)
                    continue;
                n.f = n.f + SurfaceContact(i, p.gm, p.normal, penetration, V3{});
            }
            for (Collider& c : colliders)
            {
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
                    if (!(dist < c.size[0]))
                        continue;
                    normal_local = dist > 1.0e-9 ? d * (1.0 / dist) : D3{0.0, 0.0, 1.0};
                    penetration = c.size[0] - dist;
                }
                else
                {
                    const double l[3] = {local.x, local.y, local.z};
                    int best = -1;
                    double best_depth = 0.0;
                    for (int a = 0; a < 3; ++a)
                    {
                        const double depth = c.size[a] - std::abs(l[a]);
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
                        continue;
                    double nl[3] = {0.0, 0.0, 0.0};
                    nl[best] = l[best] < 0.0 ? -1.0 : 1.0;
                    normal_local = {nl[0], nl[1], nl[2]};
                    penetration = best_depth;
                }
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
        Contacts(t);
        for (Node& n : nodes)
        {
            if (n.fixed)
                continue;
            n.v = n.v + n.f * (kDt / n.m);
            n.x = n.x + n.v * kDt;
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

int ob_world_advance(ob_world* world, double dt)
{
    if (!world || !(dt > 0.0) || !std::isfinite(dt))
        return 0;
    const auto t0 = std::chrono::steady_clock::now();
    world->carry += dt;
    long n = static_cast<long>(std::floor(world->carry / kDtD + 1.0e-6));
    n = std::max(0L, std::min(n, kMaxStepsPerAdvance));
    world->carry = std::max(0.0, world->carry - static_cast<double>(n) * kDtD);
    if (world->carry > kDtD)
        world->carry = 0.0; // clamped by kMaxStepsPerAdvance: drop the backlog
    for (Collider& c : world->colliders)
    {
        c.sum_force = c.sum_moment = D3{};
        c.steps = 0;
    }
    for (long i = 0; i < n; ++i)
        world->Step(static_cast<double>(i) * kDtD);
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
