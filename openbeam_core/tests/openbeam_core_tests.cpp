/*
    This source file is part of Rigs of Rods / OpenBeam
    SPDX-License-Identifier: GPL-3.0-or-later
*/

// Contract tests for openbeam_core.h, through the C ABI only.

#include "openbeam_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                \
    do                                                                             \
    {                                                                              \
        if (!(cond))                                                               \
        {                                                                          \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
            ++g_failures;                                                          \
        }                                                                          \
    } while (0)

const double kG = 9.81;
const double kH = 0.0005;

ob_world* Make(const std::string& body, const double* position = nullptr)
{
    const std::string text = "{\"core_test\":{\"slotType\":\"main\"," + body + "}}";
    char err[512] = {0};
    ob_world* w = ob_world_create_from_jbeam_text(text.c_str(), "vehicles/core_test/core_test.jbeam",
        position, nullptr, err, sizeof(err));
    if (!w)
        std::printf("create failed: %s\n", err);
    return w;
}

std::vector<double> Positions(const ob_world* w)
{
    std::vector<double> p(3 * static_cast<std::size_t>(ob_world_node_count(w)));
    ob_world_get_node_positions(w, p.data());
    return p;
}

// A cube of eight nodes, every pair joined (28 beams).
std::string Cube(double side, double z0, double node_weight, const char* beam_params)
{
    std::string nodes = "\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],{\"nodeWeight\":" +
        std::to_string(node_weight) + "},";
    std::vector<std::string> ids;
    for (int i = 0; i < 8; ++i)
    {
        const std::string id = "n" + std::to_string(i);
        ids.push_back(id);
        char row[160];
        std::snprintf(row, sizeof(row), "[\"%s\",%.6f,%.6f,%.6f],", id.c_str(), (i & 1) * side,
            ((i >> 1) & 1) * side, z0 + ((i >> 2) & 1) * side);
        nodes += row;
    }
    nodes.back() = ']';
    std::string beams = std::string(",\"beams\":[[\"id1:\",\"id2:\"],") + beam_params + ",";
    for (int i = 0; i < 8; ++i)
        for (int j = i + 1; j < 8; ++j)
            beams += "[\"" + ids[i] + "\",\"" + ids[j] + "\"],";
    beams.back() = ']';
    return nodes + beams;
}

void TestLoad()
{
    ob_world* w = Make(Cube(0.5, 1.0, 10.0, "{\"beamSpring\":200000,\"beamDamp\":50}"));
    CHECK(w != nullptr);
    if (!w)
        return;
    CHECK(ob_core_api_version() == OB_CORE_API_VERSION);
    CHECK(ob_world_node_count(w) == 8);
    CHECK(ob_world_beam_count(w) == 28);
    CHECK(std::abs(ob_world_total_mass(w) - 80.0) < 1e-9);
    CHECK(std::string(ob_world_node_id(w, 3)) == "n3");
    ob_world_destroy(w);

    char err[256] = {0};
    CHECK(ob_world_create_from_jbeam_text("{not jbeam", "x.jbeam", nullptr, nullptr, err, sizeof(err)) == nullptr);
    CHECK(std::string(err).find("syntax") != std::string::npos);
}

// Semi-implicit Euler from rest: after n steps v = -g n h, z = z0 - g h^2 n (n+1) / 2.
void TestFreeFall()
{
    ob_world* w = Make("\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],[\"a\",0,0,10]]");
    CHECK(w != nullptr);
    if (!w)
        return;
    const int n = ob_world_advance(w, 1.0);
    CHECK(n == 2000);
    const double z = Positions(w)[2];
    const double expected = 10.0 - kG * kH * kH * n * (n + 1) / 2.0;
    CHECK(std::abs(z - expected) < 1e-3);
    // dt carries: 4 x 0.3 ms is two steps with 0.2 ms left over.
    ob_world_reset(w);
    int total = 0;
    for (int i = 0; i < 4; ++i)
        total += ob_world_advance(w, 0.0003);
    CHECK(total == 2);
    ob_world_destroy(w);
}

void TestRestsOnGround()
{
    ob_world* w = Make(Cube(0.5, 0.3, 10.0, "{\"beamSpring\":200000,\"beamDamp\":100}"));
    CHECK(w != nullptr);
    if (!w)
        return;
    const double point[3] = {0, 0, 0}, normal[3] = {0, 0, 1};
    CHECK(ob_world_add_plane(w, point, normal, nullptr) == 0);
    for (int i = 0; i < 300; ++i)
        ob_world_advance(w, 0.01);
    const std::vector<double> p = Positions(w);
    std::vector<double> v(p.size());
    ob_world_get_node_velocities(w, v.data());
    double lowest = 1e9, fastest = 0.0;
    for (std::size_t i = 0; i < 8; ++i)
    {
        lowest = std::fmin(lowest, p[3 * i + 2]);
        fastest = std::fmax(fastest, std::sqrt(v[3 * i] * v[3 * i] + v[3 * i + 1] * v[3 * i + 1] +
            v[3 * i + 2] * v[3 * i + 2]));
    }
    std::printf("rest: lowest node z=%.5f m, fastest node %.5f m/s\n", lowest, fastest);
    CHECK(lowest > -0.005 && lowest < 0.005);
    CHECK(fastest < 0.01);
    ob_stats s;
    ob_world_get_stats(w, &s);
    CHECK(s.steps == 6000);
    CHECK(s.beams_broken == 0);
    ob_world_destroy(w);
}

// Every impulse the collider gives the cube comes back as its wrench:
// sum(F h n) = -(change of the cube's momentum), with gravity off.
void TestColliderMomentum()
{
    ob_world* w = Make(Cube(0.5, 0.0, 10.0, "{\"beamSpring\":200000,\"beamDamp\":100}"));
    CHECK(w != nullptr);
    if (!w)
        return;
    const double zero[3] = {0, 0, 0};
    ob_world_set_gravity(w, zero);
    // Nodes are points: a sphere must reach them, here within 0.354 m of its path.
    const double radius[3] = {0.45, 0, 0};
    const int c = ob_world_add_collider(w, OB_SHAPE_SPHERE, radius, nullptr);
    CHECK(c == 0);
    double center[3] = {-0.6, 0.25, 0.25};
    const double vel[3] = {3.0, 0, 0}, quat[4] = {1, 0, 0, 0};
    double impulse[3] = {0, 0, 0}, angular[3] = {0, 0, 0};
    double max_force = 0.0;
    const double host_dt = 0.002;
    for (int i = 0; i < 500; ++i)
    {
        ob_world_set_collider_state(w, c, center, quat, vel, zero);
        const int n = ob_world_advance(w, host_dt);
        double f[3], t[3];
        ob_world_get_collider_wrench(w, c, center, f, t);
        for (int k = 0; k < 3; ++k)
        {
            impulse[k] += f[k] * n * kH;
            angular[k] += t[k] * n * kH;
        }
        max_force = std::fmax(max_force, std::abs(f[0]));
        center[0] += vel[0] * host_dt;
    }
    std::vector<double> v(24);
    ob_world_get_node_velocities(w, v.data());
    double momentum[3] = {0, 0, 0};
    for (int i = 0; i < 8; ++i)
        for (int k = 0; k < 3; ++k)
            momentum[k] += 10.0 * v[3 * i + k];
    std::printf("collider: cube momentum %.4f kg m/s, collider impulse %.4f N s, peak force %.1f N\n",
        momentum[0], impulse[0], max_force);
    CHECK(momentum[0] > 10.0);
    CHECK(std::abs(momentum[0] + impulse[0]) < 1e-3 * std::abs(momentum[0]) + 1e-6);
    CHECK(std::abs(momentum[1] + impulse[1]) < 1e-3 * std::abs(momentum[0]) + 1e-6);
    CHECK(std::abs(momentum[2] + impulse[2]) < 1e-3 * std::abs(momentum[0]) + 1e-6);
    ob_world_destroy(w);
}

const char* kHang =
    "\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],{\"nodeWeight\":25},"
    "[\"top\",0,0,2,{\"fixed\":true}],[\"bob\",0,0,1]],"
    "\"beams\":[[\"id1:\",\"id2:\"],%s,[\"top\",\"bob\"]]";

ob_world* Hang(const char* params)
{
    char body[512];
    std::snprintf(body, sizeof(body), kHang, params);
    return Make(body);
}

// Legacy path: an unbounded deform keeps the beam elastic; 245 N > 100 N breaks it.
void TestLegacyBreak()
{
    ob_world* w = Hang("{\"beamSpring\":100000,\"beamDamp\":50,\"beamDeform\":\"FLT_MAX\",\"beamStrength\":100}");
    CHECK(w != nullptr);
    if (!w)
        return;
    ob_world_advance(w, 0.5);
    unsigned char broken = 0;
    ob_world_get_beam_state(w, nullptr, nullptr, nullptr, &broken);
    CHECK(broken == 1);
    ob_stats s;
    ob_world_get_stats(w, &s);
    CHECK(s.beams_broken == 1 && s.beams_deformed == 0);
    // Broken, the bob falls freely from where it was.
    CHECK(Positions(w)[5] < 0.5);
    ob_world_destroy(w);

    // Strong enough: hangs at the static stretch m g / k.
    w = Hang("{\"beamSpring\":100000,\"beamDamp\":200,\"beamDeform\":\"FLT_MAX\",\"beamStrength\":10000}");
    CHECK(w != nullptr);
    if (!w)
        return;
    ob_world_advance(w, 5.0);
    const double stretch = 1.0 - (2.0 - Positions(w)[5]);
    std::printf("legacy hang: stretch %.6f m (m g / k = %.6f)\n", -stretch, 25 * kG / 100000.0);
    CHECK(std::abs(-stretch - 25 * kG / 100000.0) < 2e-4);
    ob_world_destroy(w);
}

// Calibrated path (a finite deform): the derived material (A = 1 cm^2, so
// E = 1 GPa, yield 1.5 MPa, hardening 1 % of E) yields under 245 N and
// hardens until it carries it, near 0.1 of plastic strain; 1471 N exceeds
// what its damage capacity allows and fractures it.
void TestCalibratedYieldAndFracture()
{
    ob_world* w = Hang("{\"beamSpring\":100000,\"beamDamp\":50,\"beamDeform\":150,\"beamStrength\":200}");
    CHECK(w != nullptr);
    if (!w)
        return;
    ob_world_advance(w, 5.0);
    ob_stats s;
    ob_world_get_stats(w, &s);
    const double stretch = (2.0 - Positions(w)[5]) - 1.0;
    std::printf("calibrated 245 N: deformed=%u broken=%u stretch=%.4f m\n", s.beams_deformed, s.beams_broken,
        stretch);
    CHECK(s.beams_deformed == 1);
    CHECK(s.beams_broken == 0);
    CHECK(stretch > 0.05 && stretch < 0.4);
    // The yield is permanent and reported: stretched for good, not torn.
    float plastic = 0.0f, damage = -1.0f;
    ob_world_get_beam_damage(w, &plastic, &damage);
    std::printf("calibrated 245 N: plastic strain %.4f, damage %.3f\n", plastic, damage);
    CHECK(plastic > 0.02f && plastic < 0.4f);
    CHECK(damage >= 0.0f && damage < 1.0f);
    ob_world_reset(w);
    ob_world_get_beam_damage(w, &plastic, &damage);
    CHECK(plastic == 0.0f && damage == 0.0f);
    ob_world_destroy(w);

    w = Make(
        "\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],"
        "[\"top\",0,0,2,{\"fixed\":true,\"nodeWeight\":25}],[\"bob\",0,0,1,{\"nodeWeight\":150}]],"
        "\"beams\":[[\"id1:\",\"id2:\"],{\"beamSpring\":100000,\"beamDamp\":50,\"beamDeform\":150,"
        "\"beamStrength\":200},[\"top\",\"bob\"]]");
    CHECK(w != nullptr);
    if (!w)
        return;
    double broke_at = -1.0;
    for (int i = 0; i < 500 && broke_at < 0.0; ++i)
    {
        ob_world_advance(w, 0.01);
        ob_world_get_stats(w, &s);
        if (s.beams_broken > 0)
            broke_at = 0.01 * (i + 1);
    }
    std::printf("calibrated 1471 N: broke after %.2f s\n", broke_at);
    CHECK(broke_at > 0.0);
    float torn_damage = 0.0f;
    ob_world_get_beam_damage(w, nullptr, &torn_damage);
    CHECK(torn_damage == 1.0f);
    ob_world_destroy(w);
}

void TestBreakGroup()
{
    ob_world* w = Make(
        "\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],{\"nodeWeight\":25},"
        "[\"a\",0,0,2,{\"fixed\":true}],[\"b\",0,0,1],[\"c\",1,0,2,{\"fixed\":true}],[\"d\",1,0,1,{\"fixed\":true}]],"
        "\"beams\":[[\"id1:\",\"id2:\"],"
        "{\"beamSpring\":100000,\"beamDamp\":50,\"beamDeform\":\"FLT_MAX\",\"beamStrength\":100,\"breakGroup\":\"g\"},"
        "[\"a\",\"b\"],"
        "{\"beamStrength\":\"FLT_MAX\"},[\"c\",\"d\"]]");
    CHECK(w != nullptr);
    if (!w)
        return;
    ob_world_advance(w, 0.5);
    unsigned char broken[2] = {0, 0};
    ob_world_get_beam_state(w, nullptr, nullptr, nullptr, broken);
    CHECK(broken[0] == 1);
    CHECK(broken[1] == 1); // same group, still carried
    ob_world_destroy(w);
}

void TestCloneResetDigest()
{
    ob_world* w = Make(Cube(0.5, 0.4, 10.0, "{\"beamSpring\":200000,\"beamDamp\":50}"));
    CHECK(w != nullptr);
    if (!w)
        return;
    const double point[3] = {0, 0, 0}, normal[3] = {0, 0, 1};
    ob_world_add_plane(w, point, normal, nullptr);
    const unsigned long long initial = ob_world_state_digest(w);
    ob_world_advance(w, 0.3);
    ob_world* c = ob_world_clone(w);
    ob_world_advance(w, 1.0);
    ob_world_advance(c, 1.0);
    CHECK(ob_world_state_digest(w) == ob_world_state_digest(c));
    CHECK(ob_world_state_digest(w) != initial);
    ob_world_reset(w);
    CHECK(ob_world_state_digest(w) == initial);
    ob_world_destroy(c);
    ob_world_destroy(w);
}

void TestPlacement()
{
    const double pos[3] = {5, 0, 0};
    // 90 degrees about z.
    const double q[4] = {std::sqrt(0.5), 0, 0, std::sqrt(0.5)};
    const std::string text = "{\"p\":{\"slotType\":\"main\",\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],[\"a\",1,0,0]]}}";
    char err[256];
    ob_world* w = ob_world_create_from_jbeam_text(text.c_str(), "p.jbeam", pos, q, err, sizeof(err));
    CHECK(w != nullptr);
    if (!w)
        return;
    const std::vector<double> p = Positions(w);
    CHECK(std::abs(p[0] - 5.0) < 1e-6 && std::abs(p[1] - 1.0) < 1e-6 && std::abs(p[2]) < 1e-6);
    ob_world_destroy(w);
}


// A chain of light, stiff segments, like a cable: unstable at the game's
// 0.5 ms step, stable at 0.05 ms.
std::string Chain(int n, double dx, double node_weight, double spring)
{
    std::string nodes = "\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],{\"nodeWeight\":" +
        std::to_string(node_weight) + "},";
    std::string beams = ",\"beams\":[[\"id1:\",\"id2:\"],{\"beamSpring\":" + std::to_string(spring) +
        ",\"beamDamp\":0.05,\"beamDeform\":\"FLT_MAX\",\"beamStrength\":\"FLT_MAX\"},";
    for (int i = 0; i < n; ++i)
    {
        char row[96];
        std::snprintf(row, sizeof(row), "[\"c%d\",%.6f,0,1%s],", i, i * dx, i == 0 ? ",{\"fixed\":true}" : "");
        nodes += row;
        if (i + 1 < n)
            beams += "[\"c" + std::to_string(i) + "\",\"c" + std::to_string(i + 1) + "\"],";
    }
    nodes.back() = ']';
    beams.back() = ']';
    return nodes + beams;
}

double FastestNode(const ob_world* w)
{
    std::vector<double> v(3 * static_cast<std::size_t>(ob_world_node_count(w)));
    ob_world_get_node_velocities(w, v.data());
    double fastest = 0.0;
    for (double x : v)
        fastest = std::isfinite(x) ? std::max(fastest, std::abs(x)) : 1e30;
    return fastest;
}

void TestStep()
{
    // 1 g nodes, 20000 N/m: stable only well below 0.5 ms.
    ob_world* w = Make(Chain(21, 0.025, 0.001, 20000.0));
    CHECK(w != nullptr);
    if (!w)
        return;
    CHECK(ob_world_get_step(w) == kH);
    CHECK(ob_world_set_step(w, 0.0) == -1 && ob_world_set_step(w, 0.001) == -1);
    ob_world* coarse = ob_world_clone(w);
    ob_world_advance(coarse, 0.2);
    const double coarse_v = FastestNode(coarse);
    CHECK(ob_world_set_step(w, 0.00005) == 0 && ob_world_get_step(w) == 0.00005);
    CHECK(ob_world_advance(w, 0.01) == 200);
    ob_world_advance(w, 1.99);
    const double fine_v = FastestNode(w);
    std::printf("cable chain, 1 g nodes, 20000 N/m: fastest node %.3g m/s at 0.5 ms, %.3g m/s at 0.05 ms\n",
        coarse_v, fine_v);
    CHECK(coarse_v > 100.0);
    CHECK(fine_v < 10.0);
    ob_world_destroy(coarse);
    ob_world_destroy(w);
}

void TestNodeRadius()
{
    ob_world* w = Make(Cube(0.5, 0.1, 10.0, "{\"beamSpring\":200000,\"beamDamp\":200}"));
    CHECK(w != nullptr);
    if (!w)
        return;
    const double origin[3] = {0, 0, 0}, up[3] = {0, 0, 1};
    ob_world_add_plane(w, origin, up, nullptr);
    ob_world_set_node_radius(w, 0.02);
    ob_world_advance(w, 2.0);
    const std::vector<double> p = Positions(w);
    double lowest = 1e9;
    for (std::size_t i = 2; i < p.size(); i += 3)
        lowest = std::min(lowest, p[i]);
    std::printf("node radius 0.02 m: lowest node z=%.4f m\n", lowest);
    CHECK(std::abs(lowest - 0.02) < 0.003);
    ob_world_destroy(w);
}

void TestAttachment()
{
    // One 2 kg node welded to a sphere that rises at 1 m/s: it rises with it,
    // and the sphere carries its weight.
    ob_world* w = Make("\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],{\"nodeWeight\":2},[\"tip\",0,0,1]]");
    CHECK(w != nullptr);
    if (!w)
        return;
    CHECK(ob_world_find_node(w, "tip") == 0 && ob_world_find_node(w, "none") == -1);
    const double size[3] = {0.05, 0, 0};
    const int c = ob_world_add_collider(w, OB_SHAPE_SPHERE, size, nullptr);
    double center[3] = {0, 0, 1.2};
    const double q[4] = {1, 0, 0, 0}, v[3] = {0, 0, 1}, zero[3] = {0, 0, 0};
    ob_world_set_collider_state(w, c, center, q, zero, zero);
    CHECK(ob_world_attach_node(w, 0, c) == 0 && ob_world_attached_count(w) == 1);
    CHECK(ob_world_attach_node(w, 0, 5) == -1);
    for (int i = 0; i < 100; ++i)
    {
        ob_world_set_collider_state(w, c, center, q, v, zero);
        ob_world_advance(w, 0.01);
        center[2] += 0.01;
    }
    double force[3], torque[3];
    ob_world_get_collider_wrench(w, c, center, force, torque);
    const std::vector<double> p = Positions(w);
    std::printf("attached node: z=%.4f m (sphere at %.4f), force on sphere %.3f N\n", p[2], center[2], force[2]);
    CHECK(std::abs(p[2] - (center[2] - 0.2)) < 1e-4);
    CHECK(std::abs(force[2] + 2.0 * kG) < 2e-3 * 2.0 * kG); // binary32 node state: 0.1 %
    CHECK(ob_world_detach_node(w, 0) == 0 && ob_world_detach_node(w, 0) == -1);
    ob_world_attach_node(w, 0, c);
    ob_world_reset(w);
    CHECK(ob_world_attached_count(w) == 0);
    ob_world_destroy(w);
}

void TestReleaseStable()
{
    // A 1 g, 20000 N/m chain at a 0.2 ms step uses most of its stability
    // margin (fastest mode times the step 1.8, limit 2). Two nodes welded to
    // a collider and let go over 0.05 s: the chain swings down as it would,
    // the opening grip adding nothing that destabilises it.
    ob_world* w = Make(Chain(21, 0.025, 0.001, 20000.0));
    CHECK(w != nullptr);
    if (!w)
        return;
    CHECK(ob_world_set_step(w, 0.0002) == 0);
    const double size[3] = {0.001, 0.001, 0.001}, q[4] = {1, 0, 0, 0}, zero[3] = {0, 0, 0};
    const int c = ob_world_add_collider(w, OB_SHAPE_BOX, size, nullptr);
    const double at[3] = {0.25, 0, 1.2};
    ob_world_set_collider_state(w, c, at, q, zero, zero);
    CHECK(ob_world_attach_node(w, 1, c) == 0 && ob_world_attach_node(w, 2, c) == 0);
    ob_world_advance(w, 0.5);
    CHECK(ob_world_release_node(w, 1, 0.05) == 0 && ob_world_release_node(w, 2, 0.05) == 0);
    double fastest = 0.0;
    for (int i = 0; i < 150; ++i)
    {
        ob_world_advance(w, 0.002);
        fastest = std::max(fastest, FastestNode(w));
    }
    std::printf("chain let go over 0.05 s at a 0.2 ms step: fastest node %.3g m/s, %d still attached\n", fastest,
        ob_world_attached_count(w));
    CHECK(fastest < 5.0);
    CHECK(ob_world_attached_count(w) == 0);
    ob_world_destroy(w);
}

// Two triangles 0.1 m apart joined by axial beams only: the far one is turned
// 0.3 rad about the axis by a welded collider. With a twist element of
// 0.2 N m/rad the collider feels 0.06 N m more torque than without it, and
// the element reads the roll.
double TwistTorque(bool with_twist, double* roll)
{
    std::string body = "\"nodes\":[[\"id\",\"posX\",\"posY\",\"posZ\"],{\"nodeWeight\":0.01},";
    for (int s = 0; s < 2; ++s)
        for (int j = 0; j < 3; ++j)
        {
            const double a = 2.0 * 3.14159265358979 * j / 3.0;
            char row[128];
            std::snprintf(row, sizeof(row), "[\"s%d_%d\",%.6f,%.6f,%.6f],", s, j, 0.1 * s, 0.02 * std::cos(a),
                0.02 * std::sin(a));
            body += row;
        }
    body.back() = ']';
    body += ",\"beams\":[[\"id1:\",\"id2:\"],{\"beamSpring\":20000,\"beamDamp\":20},";
    for (int j = 0; j < 3; ++j)
    {
        body += "[\"s0_" + std::to_string(j) + "\",\"s1_" + std::to_string(j) + "\"],";
        for (int s = 0; s < 2; ++s)
            body += "[\"s" + std::to_string(s) + "_" + std::to_string(j) + "\",\"s" + std::to_string(s) + "_" +
                std::to_string((j + 1) % 3) + "\"],";
    }
    body.back() = ']';
    ob_world* w = Make(body);
    CHECK(w != nullptr);
    if (!w)
        return 0.0;
    const double zero3[3] = {0, 0, 0};
    ob_world_set_gravity(w, zero3);
    ob_world_set_step(w, 0.0001);
    if (with_twist)
    {
        CHECK(ob_world_add_rod_twist(w, 0, 3, 3, 0.02, 0.1) == -1); // only two sections
        CHECK(ob_world_add_rod_twist(w, 0, 3, 2, 0.02, 0.1) == 1);  // GJ 0.02 N m^2 over 0.1 m
        CHECK(ob_world_twist_count(w) == 1);
    }
    const double size[3] = {0.001, 0.001, 0.001}, q0[4] = {1, 0, 0, 0};
    const int a = ob_world_add_collider(w, OB_SHAPE_BOX, size, nullptr);
    const int b = ob_world_add_collider(w, OB_SHAPE_BOX, size, nullptr);
    const double pa[3] = {0, 0, 0}, pb[3] = {0.1, 0, 0};
    ob_world_set_collider_state(w, a, pa, q0, zero3, zero3);
    ob_world_set_collider_state(w, b, pb, q0, zero3, zero3);
    for (int j = 0; j < 3; ++j)
    {
        ob_world_attach_node(w, j, a);
        ob_world_attach_node(w, 3 + j, b);
    }
    const double target = 0.3, dt = 0.001;
    double angle = 0.0;
    for (int i = 0; i < 1500; ++i)
    {
        const double rate = i < 1000 ? target / 1.0 : 0.0;
        const double q[4] = {std::cos(angle / 2), std::sin(angle / 2), 0, 0}, spin[3] = {rate, 0, 0};
        ob_world_set_collider_state(w, a, pa, q0, zero3, zero3);
        ob_world_set_collider_state(w, b, pb, q, zero3, spin);
        ob_world_advance(w, dt);
        angle = std::min(target, angle + rate * dt);
    }
    double force[3], torque[3];
    ob_world_get_collider_wrench(w, b, pb, force, torque);
    if (roll)
        ob_world_get_twist(w, roll);
    ob_world_destroy(w);
    return torque[0];
}

void TestTwist()
{
    double roll = 0.0;
    const double without = TwistTorque(false, nullptr);
    const double with = TwistTorque(true, &roll);
    std::printf("twist 0.3 rad: torque on the turned end %.5f N m with GJ/L = 0.2 N m/rad, %.5f N m without; "
                "roll %.4f rad\n", with, without, roll);
    CHECK(std::abs((with - without) - (-0.2 * 0.3)) < 0.01 * 0.06);
    CHECK(std::abs(roll - 0.3) < 1e-3);
}

} // namespace

int main()
{
    TestLoad();
    TestFreeFall();
    TestRestsOnGround();
    TestColliderMomentum();
    TestLegacyBreak();
    TestCalibratedYieldAndFracture();
    TestBreakGroup();
    TestCloneResetDigest();
    TestPlacement();
    TestStep();
    TestNodeRadius();
    TestAttachment();
    TestReleaseStable();
    TestTwist();
    if (g_failures)
    {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all openbeam_core checks passed\n");
    return 0;
}
