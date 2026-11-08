/*
    This source file is part of Rigs of Rods / OpenBeam
    SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef OPENBEAM_CORE_H_
#define OPENBEAM_CORE_H_

/* OpenBeam soft-body core: a C ABI for embedding OpenBeam's node/beam
 * physics in another simulator (README.md).
 *
 * One ob_world holds one JBeam structure. It steps at OpenBeam's fixed
 * 0.5 ms with the game's ordinary beam law (Actor::CalcBeams: damper clamp,
 * plastic rest-length deformation, breaking, break groups) and the game's
 * static contact law (primitiveCollision's dry branch) against host planes
 * and host colliders. The host simulates everything else and owns the
 * colliders: it sets their pose and velocity before ob_world_advance and
 * applies the returned reaction wrench to its own bodies after it.
 *
 * It is not the game: no wheels, hydros, shocks, Lua, flexbodies, cab
 * triangles, inter-actor contact or calibrated materials.
 *
 * Units are SI. Coordinates are the host's; a JBeam is authored Z-up, so
 * the default gravity is (0, 0, -9.81). Quaternions are (w, x, y, z).
 * Every function is safe to call with a null world and then does nothing.
 * A world is not thread-safe; distinct worlds are independent.
 */

#include <stddef.h>

#if defined(_WIN32)
#  if defined(OPENBEAM_CORE_BUILD_SHARED)
#    define OB_API __declspec(dllexport)
#  elif defined(OPENBEAM_CORE_STATIC)
#    define OB_API
#  else
#    define OB_API __declspec(dllimport)
#  endif
#else
#  define OB_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 2: adds ob_world_get_beam_damage, a per-world step, node contact radius and
 * node damping, node attachments and physical beam materials; every
 * version-1 function is unchanged. */
#define OB_CORE_API_VERSION 2

typedef struct ob_world ob_world;

typedef enum ob_shape
{
    OB_SHAPE_SPHERE = 0,  /* size[0] = radius */
    OB_SHAPE_BOX = 1,     /* size = half extents along local x, y, z */
    OB_SHAPE_CAPSULE = 2  /* size[0] = radius, size[1] = half length along local z */
} ob_shape;

/* A ground model in the game's ground_models.cfg terms. */
typedef struct ob_ground_model
{
    float ms;       /* static friction coefficient */
    float mc;       /* sliding friction coefficient */
    float va;       /* adhesion velocity, m/s */
    float vs;       /* Stribeck velocity, m/s */
    float t2;       /* hydrodynamic friction, s/m */
    float alpha;    /* Stribeck exponent */
    float strength; /* ground strength multiplier */
} ob_ground_model;

/* A beam's material, in physical units, for the calibrated elastoplastic
 * damage law that the game derives from beamSpring and beamDeform (with a
 * declared 1 cm^2 area, 1 % hardening and damage from 2 % plastic strain).
 * Setting it directly lets a structure carry a real material. */
typedef struct ob_beam_material
{
    double area_m2;                     /* the beam's cross-section */
    double elastic_modulus_pa;          /* Young's modulus */
    double yield_stress_pa;             /* where permanent set begins */
    double hardening_modulus_pa;        /* stress per unit plastic strain after yield */
    double damage_onset_plastic_strain; /* where damage begins to accumulate */
    double damage_capacity_j_per_m3;    /* damage-driver work per volume before it tears */
} ob_beam_material;

typedef struct ob_stats
{
    unsigned long long steps;       /* steps run since create or reset */
    unsigned long long contacts;    /* node contacts applied, cumulative */
    unsigned int beams_broken;      /* beams broken now */
    unsigned int beams_deformed;    /* beams whose rest length moved */
    unsigned int beams_skipped;     /* authored beams of a type the core does not run */
    double last_advance_us;         /* wall time of the last ob_world_advance */
} ob_stats;

OB_API int ob_core_api_version(void);
/* The concrete ground model (resources/skeleton/config/ground_models.cfg). */
OB_API ob_ground_model ob_ground_model_concrete(void);

/* Creates a world from a JBeam document. The part with "slotType": "main"
 * is the root; other parts in the same document fill its slots. The rest
 * pose is placed by (position, quaternion) before the first step; either
 * may be null for identity. Returns null on failure with a message in err. */
OB_API ob_world* ob_world_create_from_jbeam_text(const char* text, const char* virtual_path,
    const double position[3], const double quaternion[4], char* err, size_t err_len);
OB_API ob_world* ob_world_create_from_jbeam_file(const char* path,
    const double position[3], const double quaternion[4], char* err, size_t err_len);
OB_API void ob_world_destroy(ob_world* world);
/* A deep copy, scene and colliders included. */
OB_API ob_world* ob_world_clone(const ob_world* world);
/* Back to the placed rest pose: beams unbroken, rest lengths restored,
 * velocities zero, statistics cleared. The scene is kept. */
OB_API void ob_world_reset(ob_world* world);

OB_API void ob_world_set_gravity(ob_world* world, const double gravity[3]);
/* Returns the plane index. */
OB_API int ob_world_add_plane(ob_world* world, const double point[3], const double normal[3],
    const ob_ground_model* ground_model);
/* Returns the collider id, or -1 for an unknown shape or bad size. */
OB_API int ob_world_add_collider(ob_world* world, ob_shape shape, const double size[3],
    const ob_ground_model* ground_model);
/* The collider's pose and velocity at the start of the next advance. Within
 * the advance it moves on that velocity; its orientation is held. */
OB_API void ob_world_set_collider_state(ob_world* world, int collider, const double position[3],
    const double quaternion[4], const double linear_velocity[3], const double angular_velocity[3]);

/* The integration step in seconds, from 1e-6 to the game's 0.0005 (the
 * default). Light, stiff structures such as cables need a smaller step to stay
 * stable; every law takes the step as an input. Returns 0, or -1 if out of
 * range. Clears the carried remainder. */
OB_API int ob_world_set_step(ob_world* world, double step);
OB_API double ob_world_get_step(const ob_world* world);
/* Every node touches planes and colliders at this distance (metres, default
 * 0), so a structure of points has thickness: a cable its radius. */
OB_API void ob_world_set_node_radius(ob_world* world, double radius);
/* A force -damping m v on every node, per second (default 0, as in the game).
 * Beam dampers mostly damp a structure's fast axial vibration; this damps its
 * slow swing and settling too, as a cable's internal friction does. Stable
 * while damping times the step is below 2. */
OB_API void ob_world_set_node_damping(ob_world* world, double per_second);
/* Welds a node to a collider where it is now, in the collider's pose as last
 * set: from the next step the collider carries the node, and the force that
 * takes is put on the collider, so its wrench includes it. For grasps and for
 * connectors fixed to a cable's end. Returns 0, or -1 for a bad node or
 * collider or a fixed node. ob_world_reset removes every attachment. */
OB_API int ob_world_attach_node(ob_world* world, int node, int collider);
/* Releases a node. A welded node does not touch its collider, and a released
 * one ignores it until it is clear of it, as an opening gripper lets go,
 * rather than being thrown out of the overlap. Returns 0, or -1 if the node
 * was not attached. */
OB_API int ob_world_detach_node(ob_world* world, int node);
/* Lets a node go over `seconds`: the grip's force fades linearly to zero, as a
 * gripper opens, then the node is released as by ob_world_detach_node. An
 * instant release of a bent structure sends a stress wave through it that a
 * real release never does. 0 releases at once. Returns 0, or -1 if the node
 * was not attached. */
OB_API int ob_world_release_node(ob_world* world, int node, double seconds);
OB_API void ob_world_detach_all(ob_world* world);
OB_API int ob_world_attached_count(const ob_world* world);
/* The index of the node with this JBeam id, or -1. */
OB_API int ob_world_find_node(const ob_world* world, const char* id);

/* Advances by dt seconds of host time: as many steps (ob_world_set_step) as
 * fit, with the remainder carried to the next call. Returns the steps run. */
OB_API int ob_world_advance(ob_world* world, double dt);
/* The force and the torque about reference_point that the soft body put on
 * the collider, averaged over the last advance (zero if no step ran). The
 * host applies them to its body as they are. */
OB_API void ob_world_get_collider_wrench(const ob_world* world, int collider,
    const double reference_point[3], double force[3], double torque[3]);

OB_API int ob_world_node_count(const ob_world* world);
OB_API int ob_world_beam_count(const ob_world* world);
OB_API const char* ob_world_node_id(const ob_world* world, int node);
OB_API double ob_world_total_mass(const ob_world* world);
OB_API void ob_world_get_node_positions(const ob_world* world, double* xyz);
OB_API void ob_world_get_node_velocities(const ob_world* world, double* xyz);
OB_API void ob_world_get_beam_nodes(const ob_world* world, int* pairs);
/* Any output may be null. stress: the beam's axial force, positive in
 * compression, as the game's beam_t::stress. strength: the current break
 * force. */
OB_API void ob_world_get_beam_state(const ob_world* world, float* stress, float* rest_length,
    float* strength, unsigned char* broken);
/* Gives a beam a physical material (calibrated elastoplastic damage law),
 * replacing what its JBeam row derived; beam indices follow the JBeam's beam
 * order. Its history restarts; the change survives ob_world_reset. Returns 0,
 * or -1 for a bad beam or parameters the material refuses. */
OB_API int ob_world_set_beam_material(ob_world* world, int beam, const ob_beam_material* material);
/* Reads beam materials from a text file, one line per range of beams:
 *   first count area_m2 E_pa yield_pa hardening_pa damage_onset capacity_j_per_m3
 * and twist for rods (ob_world_add_rod_twist):
 *   twist first_node nodes_per_section sections gj_n_m2 damping_ratio
 * with '#' starting a comment (make_cable.py writes these). Returns the number
 * of beams configured, or -1 with a message in err. */
OB_API int ob_world_load_materials(ob_world* world, const char* path, char* err, size_t err_len);
/* Twist stiffness between two cross-sections of `count` nodes each (count >=
 * 3, paired in order): St-Venant torsion, as in discrete elastic rods. A
 * torque -stiffness * roll - damping * roll rate acts about the line joining
 * the sections' centres, where roll is section b's rotation about that line
 * relative to section a since the structure was created; it reaches the nodes
 * as forces across each section that sum to zero. A lattice of axial beams
 * resists twist only through diagonals lying nearly along its axis, far less
 * than a cable does, and a bent bay can roll over instead of straining its
 * fibres; this supplies the torsion it lacks. It fades out where the pair
 * bends between 60 and 90 degrees (a kink tighter than about the spacing of the
 * sections, which the lattice does not resolve), and is inactive once every
 * beam joining the two sections is torn. stiffness in N m per radian, damping in N m s per
 * radian. Returns the element's index, or -1. Survives ob_world_reset. */
OB_API int ob_world_add_twist(ob_world* world, const int* section_a, const int* section_b, int count,
    double stiffness, double damping);
/* Twist along a rod of `sections` cross-sections of `per_section` nodes each,
 * numbered section by section from first_node: one element per neighbouring
 * pair, of stiffness gj / (the distance between their centres) and damping
 * damping_ratio times the pair's critical damping. Returns the elements added,
 * or -1. */
OB_API int ob_world_add_rod_twist(ob_world* world, int first_node, int per_section, int sections,
    double gj_n_m2, double damping_ratio);
OB_API int ob_world_twist_count(const ob_world* world);
/* Each twist element's roll from rest at the last step, radians. */
OB_API void ob_world_get_twist(const ob_world* world, double* roll);
/* Each beam's permanent damage, for drawing and analysis; either output may
 * be null. plastic_strain: the permanent axial strain, positive when the beam
 * has been stretched for good and negative when crushed (calibrated beams:
 * the material's plastic strain; other beams: the change of rest length over
 * the rest length at creation). damage: 0 intact to 1 torn (calibrated beams:
 * the material's damage variable; other beams: 1 once broken). */
OB_API void ob_world_get_beam_damage(const ob_world* world, float* plastic_strain, float* damage);
OB_API void ob_world_get_stats(const ob_world* world, ob_stats* stats);
/* FNV-1a over the node state bytes and beam rest lengths and flags: equal
 * digests mean the same simulation state, bit for bit. */
OB_API unsigned long long ob_world_state_digest(const ob_world* world);

#ifdef __cplusplus
}
#endif

#endif /* OPENBEAM_CORE_H_ */
