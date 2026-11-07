/*
    This source file is part of Rigs of Rods / OpenBeam
    SPDX-License-Identifier: GPL-3.0-or-later
*/

// OpenBeam soft body as a MuJoCo engine plugin ("openbeam.softbody").
//
// MuJoCo simulates the rigid world; one OpenBeam world (openbeam_core.h)
// simulates one JBeam structure. Per mj_step, in the passive-force stage:
//   1. each collider geom's pose and 6-D velocity is handed to OpenBeam;
//   2. OpenBeam advances one MuJoCo timestep in 0.5 ms steps, its nodes
//      meeting the model's planes and those geoms with the game's contact law;
//   3. the force and torque the soft body put on each collider over that
//      interval go into qfrc_passive on the collider's body (mj_applyFT),
//      so MuJoCo integrates the same interval with them.
// The advance runs once per simulation time: an mj_forward at an unchanged
// time (a paused viewer) re-applies the last wrench without stepping.
// RK4 evaluates forces at intermediate times and is not supported.
//
// Optional sensor (<sensor><plugin instance=.../></sensor>, 6 values):
// beams broken, beams deformed, centroid x y z, OpenBeam steps run.
// The visualize callback draws every beam: green at rest, yellow under load,
// orange to red where it has yielded for good, and torn beams as red stubs.

#include "openbeam_core.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#define OB_MUJOCO_EXPORT extern "C" __declspec(dllexport)
#else
#define OB_MUJOCO_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace {

const char* const kAttributes[] = {"jbeam", "pos", "quat", "colliders", "grounds", "stress_scale",
    "strain_scale", "beam_width"};
constexpr int kSensorDim = 6;

struct ColliderBinding
{
    int geom = -1;
    int body = -1;
    int core_id = -1;
};

struct Instance
{
    ob_world* world = nullptr;
    std::vector<ColliderBinding> colliders;
    double last_time = -1.0;
    bool advanced = false;
    bool planes_added = false; // planes are static: posed once, kept across resets
    double stress_scale = 20000.0; // N of live load drawn full yellow
    double strain_scale = 0.05;    // plastic strain drawn full red
    double beam_width = 0.012;
    std::vector<double> scratch_positions;
    std::vector<int> pairs;
    std::vector<float> stress, rest, plastic, damage;
    std::vector<unsigned char> broken;

    ~Instance() { ob_world_destroy(world); }
};

std::string Config(const mjModel* m, int instance, const char* key)
{
    const char* value = mj_getPluginConfig(m, instance, key);
    return value ? std::string(value) : std::string();
}

bool ParseNumbers(const std::string& text, int count, double* out)
{
    std::istringstream in(text);
    for (int i = 0; i < count; ++i)
        if (!(in >> out[i]))
            return false;
    return true;
}

std::vector<std::string> Words(const std::string& text)
{
    std::istringstream in(text);
    std::vector<std::string> words;
    std::string w;
    while (in >> w)
        words.push_back(w);
    return words;
}

// MuJoCo's geom_xmat is row major; its quaternion is (w, x, y, z) as OpenBeam's.
void GeomQuat(const mjData* d, int geom, double q[4])
{
    mju_mat2Quat(q, d->geom_xmat + 9 * geom);
}

Instance* Get(const mjData* d, int instance)
{
    return reinterpret_cast<Instance*>(d->plugin_data[instance]);
}

void BindScene(const mjModel* m, int instance, Instance& inst)
{
    // Ground: the named planes, or every plane of the model.
    const std::vector<std::string> grounds = Words(Config(m, instance, "grounds"));
    for (int g = 0; g < m->ngeom; ++g)
    {
        if (m->geom_type[g] != mjGEOM_PLANE)
            continue;
        const char* name = mj_id2name(m, mjOBJ_GEOM, g);
        if (!grounds.empty() && (!name || std::find(grounds.begin(), grounds.end(), name) == grounds.end()))
            continue;
        inst.colliders.push_back(ColliderBinding{g, -1, -1}); // planes: core_id stays -1
    }
    for (const std::string& name : Words(Config(m, instance, "colliders")))
    {
        const int g = mj_name2id(m, mjOBJ_GEOM, name.c_str());
        if (g < 0)
        {
            mju_warning("openbeam.softbody: collider geom '%s' not found", name.c_str());
            continue;
        }
        const mjtNum* s = m->geom_size + 3 * g;
        double size[3] = {0, 0, 0};
        ob_shape shape;
        switch (m->geom_type[g])
        {
        case mjGEOM_SPHERE: shape = OB_SHAPE_SPHERE; size[0] = s[0]; break;
        case mjGEOM_CAPSULE: shape = OB_SHAPE_CAPSULE; size[0] = s[0]; size[1] = s[1]; break;
        case mjGEOM_BOX: shape = OB_SHAPE_BOX; size[0] = s[0]; size[1] = s[1]; size[2] = s[2]; break;
        default:
            mju_warning("openbeam.softbody: collider geom '%s' is not a sphere, capsule or box", name.c_str());
            continue;
        }
        const int id = ob_world_add_collider(inst.world, shape, size, nullptr);
        if (id >= 0)
            inst.colliders.push_back(ColliderBinding{g, m->geom_bodyid[g], id});
    }
}

void AddPlanes(const mjModel* m, const mjData* d, Instance& inst)
{
    for (const ColliderBinding& c : inst.colliders)
    {
        if (c.core_id >= 0)
            continue;
        const mjtNum* mat = d->geom_xmat + 9 * c.geom;
        const double normal[3] = {mat[2], mat[5], mat[8]}; // the plane's local z
        ob_world_add_plane(inst.world, d->geom_xpos + 3 * c.geom, normal, nullptr);
    }
}

int Init(const mjModel* m, mjData* d, int instance)
{
    const std::string path = Config(m, instance, "jbeam");
    if (path.empty())
    {
        mju_warning("openbeam.softbody: the 'jbeam' attribute is required");
        return -1;
    }
    double pos[3] = {0, 0, 0}, quat[4] = {1, 0, 0, 0};
    const std::string pos_text = Config(m, instance, "pos"), quat_text = Config(m, instance, "quat");
    if ((!pos_text.empty() && !ParseNumbers(pos_text, 3, pos)) ||
        (!quat_text.empty() && !ParseNumbers(quat_text, 4, quat)))
    {
        mju_warning("openbeam.softbody: 'pos' needs 3 numbers and 'quat' 4");
        return -1;
    }
    // A relative path that is not found from the working directory is tried
    // against OPENBEAM_ASSET_DIR (MuJoCo does not tell a plugin where its
    // model file was).
    std::string resolved = path;
    if (FILE* probe = std::fopen(resolved.c_str(), "rb"))
        std::fclose(probe);
    else if (const char* dir = std::getenv("OPENBEAM_ASSET_DIR"))
        resolved = std::string(dir) + "/" + path;
    char err[512] = {0};
    ob_world* world = ob_world_create_from_jbeam_file(resolved.c_str(), pos, quat, err, sizeof(err));
    if (!world)
    {
        mju_warning("openbeam.softbody: %s", err);
        return -1;
    }
    Instance* inst = new Instance();
    inst->world = world;
    double value = 0.0;
    if (ParseNumbers(Config(m, instance, "stress_scale"), 1, &value) && value > 0.0)
        inst->stress_scale = value;
    if (ParseNumbers(Config(m, instance, "strain_scale"), 1, &value) && value > 0.0)
        inst->strain_scale = value;
    if (ParseNumbers(Config(m, instance, "beam_width"), 1, &value) && value > 0.0)
        inst->beam_width = value;
    const double gravity[3] = {m->opt.gravity[0], m->opt.gravity[1], m->opt.gravity[2]};
    ob_world_set_gravity(world, gravity);
    BindScene(m, instance, *inst);
    // The planes are posed on the first compute, once MuJoCo has run its
    // kinematics (an mjData made here would re-enter this init).
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(inst);
    return 0;
}

void Destroy(mjData* d, int instance)
{
    delete Get(d, instance);
    d->plugin_data[instance] = 0;
}

void Copy(mjData* dest, const mjModel*, const mjData* src, int instance)
{
    const Instance* from = Get(src, instance);
    Instance* to = new Instance(*from);
    to->world = ob_world_clone(from->world);
    delete Get(dest, instance);
    dest->plugin_data[instance] = reinterpret_cast<uintptr_t>(to);
}

void Reset(const mjModel*, mjtNum*, void* plugin_data, int)
{
    Instance* inst = static_cast<Instance*>(plugin_data);
    if (!inst)
        return;
    ob_world_reset(inst->world);
    inst->last_time = -1.0;
    inst->advanced = false;
}

void ApplyWrenches(const mjModel* m, mjData* d, Instance& inst)
{
    for (const ColliderBinding& c : inst.colliders)
    {
        if (c.core_id < 0)
            continue;
        const mjtNum* center = d->geom_xpos + 3 * c.geom;
        double force[3], torque[3];
        ob_world_get_collider_wrench(inst.world, c.core_id, center, force, torque);
        mj_applyFT(m, d, force, torque, center, c.body, d->qfrc_passive);
    }
}

void ComputePassive(const mjModel* m, mjData* d, Instance& inst)
{
    if (!inst.planes_added)
    {
        AddPlanes(m, d, inst);
        inst.planes_added = true;
    }
    if (!inst.advanced || d->time != inst.last_time)
    {
        for (const ColliderBinding& c : inst.colliders)
        {
            if (c.core_id < 0)
                continue;
            mjtNum velocity[6]; // (angular, linear), world frame, at the geom centre
            mj_objectVelocity(m, d, mjOBJ_GEOM, c.geom, velocity, 0);
            double q[4];
            GeomQuat(d, c.geom, q);
            ob_world_set_collider_state(inst.world, c.core_id, d->geom_xpos + 3 * c.geom, q, velocity + 3, velocity);
        }
        ob_world_advance(inst.world, m->opt.timestep);
        inst.last_time = d->time;
        inst.advanced = true;
    }
    ApplyWrenches(m, d, inst);
}

void ComputeSensors(const mjModel* m, mjData* d, int instance, Instance& inst)
{
    ob_stats stats;
    ob_world_get_stats(inst.world, &stats);
    const int n = ob_world_node_count(inst.world);
    inst.scratch_positions.resize(3 * static_cast<std::size_t>(n));
    ob_world_get_node_positions(inst.world, inst.scratch_positions.data());
    double centroid[3] = {0, 0, 0};
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k)
            centroid[k] += inst.scratch_positions[3 * static_cast<std::size_t>(i) + k] / std::max(1, n);
    for (int s = 0; s < m->nsensor; ++s)
    {
        if (m->sensor_plugin[s] != instance)
            continue;
        mjtNum* out = d->sensordata + m->sensor_adr[s];
        out[0] = stats.beams_broken;
        out[1] = stats.beams_deformed;
        out[2] = centroid[0];
        out[3] = centroid[1];
        out[4] = centroid[2];
        out[5] = static_cast<mjtNum>(stats.steps);
    }
}

void Compute(const mjModel* m, mjData* d, int instance, int capability_bit)
{
    Instance* inst = Get(d, instance);
    if (!inst)
        return;
    if (capability_bit & mjPLUGIN_PASSIVE)
        ComputePassive(m, d, *inst);
    if (capability_bit & mjPLUGIN_SENSOR)
        ComputeSensors(m, d, instance, *inst);
}

void Visualize(const mjModel*, mjData* d, const mjvOption*, mjvScene* scn, int instance)
{
    Instance* inst = Get(d, instance);
    if (!inst)
        return;
    const int nn = ob_world_node_count(inst->world), nb = ob_world_beam_count(inst->world);
    const std::size_t n = static_cast<std::size_t>(nb);
    inst->scratch_positions.resize(3 * static_cast<std::size_t>(nn));
    inst->pairs.resize(2 * n);
    inst->stress.resize(n);
    inst->rest.resize(n);
    inst->plastic.resize(n);
    inst->damage.resize(n);
    inst->broken.resize(n);
    ob_world_get_node_positions(inst->world, inst->scratch_positions.data());
    ob_world_get_beam_nodes(inst->world, inst->pairs.data());
    ob_world_get_beam_state(inst->world, inst->stress.data(), inst->rest.data(), nullptr, inst->broken.data());
    ob_world_get_beam_damage(inst->world, inst->plastic.data(), inst->damage.data());
    auto connector = [&](const double* from, const double* to, const float rgba[4], double width) {
        if (scn->ngeom >= scn->maxgeom)
            return;
        mjvGeom* g = scn->geoms + scn->ngeom;
        mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
        g->category = mjCAT_DECOR;
        mjv_connector(g, mjGEOM_CAPSULE, width, from, to);
        ++scn->ngeom;
    };
    for (std::size_t b = 0; b < n; ++b)
    {
        const double* a = inst->scratch_positions.data() + 3 * inst->pairs[2 * b];
        const double* c = inst->scratch_positions.data() + 3 * inst->pairs[2 * b + 1];
        if (inst->broken[b])
        {
            // Torn: a red stub snapped back towards each end node.
            static const float kTorn[4] = {1.0f, 0.05f, 0.05f, 1.0f};
            const double span = std::sqrt((c[0] - a[0]) * (c[0] - a[0]) + (c[1] - a[1]) * (c[1] - a[1]) +
                (c[2] - a[2]) * (c[2] - a[2]));
            if (span <= 0.0)
                continue;
            const double f = std::min(0.25, 0.3 * inst->rest[b] / span);
            const double a_end[3] = {a[0] + f * (c[0] - a[0]), a[1] + f * (c[1] - a[1]), a[2] + f * (c[2] - a[2])};
            const double c_end[3] = {c[0] + f * (a[0] - c[0]), c[1] + f * (a[1] - c[1]), c[2] + f * (a[2] - c[2])};
            connector(a, a_end, kTorn, 1.5 * inst->beam_width);
            connector(c, c_end, kTorn, 1.5 * inst->beam_width);
            continue;
        }
        // Severity 0 (green) to 0.5 (yellow) for live load; 0.6 (orange) to
        // 0.95 (red) once the beam has yielded or taken damage.
        float s = 0.5f * static_cast<float>(std::min(1.0, std::abs(inst->stress[b]) / inst->stress_scale));
        const float permanent = std::max(static_cast<float>(std::min(1.0,
            std::abs(inst->plastic[b]) / inst->strain_scale)), inst->damage[b]);
        if (std::abs(inst->plastic[b]) > 1e-4f || inst->damage[b] > 0.0f)
            s = 0.6f + 0.35f * permanent;
        const float rgba[4] = {std::min(1.0f, 2.0f * s), std::min(1.0f, 2.0f * (1.0f - s)), 0.1f, 1.0f};
        connector(a, c, rgba, inst->beam_width);
    }
}

} // namespace

// The OpenBeam world behind one plugin instance of `d`, for hosts that want
// the full core API (tests, recorders). Null if the instance is not ours.
OB_MUJOCO_EXPORT ob_world* openbeam_mujoco_world(const mjData* d, int instance)
{
    if (!d || instance < 0)
        return nullptr;
    const Instance* inst = Get(d, instance);
    return inst ? inst->world : nullptr;
}

mjPLUGIN_LIB_INIT(openbeam_softbody)
{
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = "openbeam.softbody";
    plugin.capabilityflags = mjPLUGIN_PASSIVE | mjPLUGIN_SENSOR;
    plugin.needstage = mjSTAGE_ACC;
    plugin.nattribute = sizeof(kAttributes) / sizeof(kAttributes[0]);
    plugin.attributes = kAttributes;
    plugin.nstate = +[](const mjModel*, int) { return 0; };
    plugin.nsensordata = +[](const mjModel*, int, int) { return kSensorDim; };
    plugin.init = Init;
    plugin.destroy = Destroy;
    plugin.copy = Copy;
    plugin.reset = Reset;
    plugin.compute = Compute;
    plugin.visualize = Visualize;
    mjp_registerPlugin(&plugin);
}
