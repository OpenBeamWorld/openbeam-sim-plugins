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

#define OB_CORE_API_VERSION 1

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

typedef struct ob_stats
{
    unsigned long long steps;       /* 0.5 ms steps run since create or reset */
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

/* Advances by dt seconds of host time: as many 0.5 ms steps as fit, with
 * the remainder carried to the next call. Returns the steps run. */
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
OB_API void ob_world_get_stats(const ob_world* world, ob_stats* stats);
/* FNV-1a over the node state bytes and beam rest lengths and flags: equal
 * digests mean the same simulation state, bit for bit. */
OB_API unsigned long long ob_world_state_digest(const ob_world* world);

#ifdef __cplusplus
}
#endif

#endif /* OPENBEAM_CORE_H_ */
