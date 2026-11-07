/*
    This source file is part of Rigs of Rods / OpenBeam
    SPDX-License-Identifier: GPL-3.0-or-later
*/

#pragma once

// The node/beam structure the core simulates, read from a JBeam document by
// the engine's own parser, resolver and structural tier
// (engine resources/beamng/JBeamSyntax, JBeamPartResolver, JBeamStructuralIR),
// with the defaults the game's lowering applies (JBeamToRigDef.cpp).

#include <cstddef>
#include <string>
#include <vector>

namespace OpenBeamCore {

struct StructureNode
{
    std::string id;
    double position[3] = {0.0, 0.0, 0.0}; //!< authored, BeamNG SI, Z up
    double mass = 25.0;                    //!< nodeWeight, kg
    double friction_coef = 1.0;            //!< frictionCoef
    bool fixed = false;                    //!< `fixed`: pinned to the world
    bool collision = true;                 //!< `collision`
};

struct StructureBeam
{
    std::size_t node_a = 0, node_b = 0;
    double spring = 0.0;                 //!< beamSpring, N/m
    double damping = 0.0;                //!< beamDamp, N s/m
    double deform = 0.0;                 //!< beamDeform, N (FLT_MAX when unbounded)
    double strength = 0.0;               //!< beamStrength, N (FLT_MAX when unbounded)
    bool deform_unbounded = false;
    bool strength_unbounded = false;
    double precompression = 1.0;         //!< beamPrecompression multiplier
    bool has_precompression_range = false;
    double precompression_range = 0.0;   //!< precompressionRange, m (wins over the multiplier)
    bool has_deform_limit_expansion = false;
    double deform_limit_expansion = 0.0; //!< deformLimitExpansion, multiple of the rest length
    std::string break_group;             //!< empty = no group
};

struct Structure
{
    std::vector<StructureNode> nodes;
    std::vector<StructureBeam> beams;
    std::size_t skipped_beams = 0; //!< authored beams of a type the core does not run
};

/// False with a message in `error` when the document does not parse, resolve
/// or lower. `virtual_path` names the document in diagnostics and must end in
/// `.jbeam`.
bool LoadJBeamStructure(const std::string& text, const std::string& virtual_path,
    Structure& out, std::string& error);

} // namespace OpenBeamCore
