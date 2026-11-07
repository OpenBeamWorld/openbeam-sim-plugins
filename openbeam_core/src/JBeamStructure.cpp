/*
    This source file is part of Rigs of Rods / OpenBeam
    SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "JBeamStructure.h"

#include "JBeamPartResolver.h"
#include "JBeamStructuralIR.h"
#include "JBeamSyntax.h"

#include <limits>
#include <sstream>

namespace OpenBeamCore {

namespace {

// JBeamToRigDef.cpp: what an unauthored field lowers to.
constexpr double kDefaultSpring = 4300000.0;
constexpr double kDefaultDamping = 580.0;
constexpr double kDefaultDeform = 220000.0;

constexpr double kUnbounded = static_cast<double>(std::numeric_limits<float>::max());

template <typename Diagnostic, typename Severity>
std::string FirstErrors(const std::vector<Diagnostic>& diagnostics, Severity error_severity, const char* stage)
{
    std::ostringstream out;
    out << stage << " refused the document";
    int shown = 0;
    for (const Diagnostic& d : diagnostics)
    {
        if (shown == 3)
            break;
        if (d.severity != error_severity)
            continue;
        out << (shown == 0 ? ": " : "; ") << d.detail;
        ++shown;
    }
    return out.str();
}

// A structure need not be a vehicle. The refNodes frame orients a vehicle (its
// camera and forward axis) and the core never reads it, so a document whose
// only error is the missing frame is admitted; any other error refuses it.
bool OnlyMissingRefNodes(const RoR::BeamNG::JBeamStructuralIR& ir)
{
    using namespace RoR::BeamNG;
    bool missing = false;
    for (const JBeamStructuralDiagnostic& d : ir.diagnostics)
    {
        if (d.severity == JBeamStructuralSeverity::WARNING)
            continue;
        if (d.code != JBeamStructuralDiagnosticCode::MISSING_REF_NODES)
            return false;
        missing = true;
    }
    return missing;
}

} // namespace

bool LoadJBeamStructure(const std::string& text, const std::string& virtual_path,
    Structure& out, std::string& error)
{
    using namespace RoR::BeamNG;

    const JBeamParseResult parsed = ParseJBeam(text, virtual_path);
    if (!parsed.IsValid())
    {
        error = "JBeam syntax error";
        if (!parsed.diagnostics.empty())
        {
            const JBeamDiagnostic& d = parsed.diagnostics.front();
            std::ostringstream msg;
            msg << "JBeam syntax error at line " << d.span.begin.line << ": " << d.message;
            error = msg.str();
        }
        return false;
    }

    std::vector<JBeamPackageSource> sources(1);
    sources[0].package_path = virtual_path;
    sources[0].document = parsed.root;
    const JBeamPackageIndex index = BuildJBeamPackageIndex(sources);
    if (!index.IsValid())
    {
        error = FirstErrors(index.diagnostics, JBeamResolveSeverity::ERROR_SEVERITY, "the JBeam part index");
        return false;
    }
    const JBeamResolvedGraph graph = ResolveJBeamPartGraph(index);
    if (!graph.IsValid())
    {
        error = FirstErrors(graph.diagnostics, JBeamResolveSeverity::ERROR_SEVERITY, "the JBeam part resolver");
        return false;
    }
    const JBeamStructuralIR ir = BuildJBeamStructuralIR(graph);
    if (!ir.IsValid() && !OnlyMissingRefNodes(ir))
    {
        error = FirstErrors(ir.diagnostics, JBeamStructuralSeverity::ERROR_SEVERITY, "the JBeam structural tier");
        return false;
    }

    Structure result;
    result.nodes.reserve(ir.nodes.size());
    for (const JBeamStructuralNode& n : ir.nodes)
    {
        StructureNode node;
        node.id = n.id;
        node.position[0] = n.x;
        node.position[1] = n.y;
        node.position[2] = n.z;
        node.mass = n.node_weight;
        node.friction_coef = n.friction_coef;
        node.fixed = n.fixed;
        node.collision = n.collision;
        result.nodes.push_back(node);
    }
    for (const JBeamStructuralBeam& b : ir.beams)
    {
        // The core runs the ordinary axial beam. SUPPORT, BOUNDED, the zero-rest
        // role and the preserved-disabled special types are counted, not run.
        if (b.status != JBeamStructuralBeamStatus::ENABLED || b.beam_type != "NORMAL" ||
            b.jbeam_zero_rest_spring || b.node_a_index >= result.nodes.size() ||
            b.node_b_index >= result.nodes.size() || b.node_a_index == b.node_b_index)
        {
            ++result.skipped_beams;
            continue;
        }
        StructureBeam beam;
        beam.node_a = b.node_a_index;
        beam.node_b = b.node_b_index;
        beam.spring = b.has_spring ? b.spring : kDefaultSpring;
        beam.damping = b.has_damping ? b.damping : kDefaultDamping;
        beam.deform_unbounded = b.has_deform && b.deform_unbounded;
        beam.deform = beam.deform_unbounded ? kUnbounded : (b.has_deform ? b.deform : kDefaultDeform);
        beam.strength_unbounded = !b.has_strength || b.strength_unbounded;
        beam.strength = beam.strength_unbounded ? kUnbounded : b.strength;
        if (b.has_precompression)
            beam.precompression = b.precompression;
        beam.has_precompression_range = b.has_precompression_range;
        beam.precompression_range = b.precompression_range;
        beam.has_deform_limit_expansion = b.has_deform_limit_expansion;
        beam.deform_limit_expansion = b.deform_limit_expansion;
        beam.break_group = b.break_group;
        result.beams.push_back(beam);
    }
    if (result.nodes.empty())
    {
        error = "the JBeam document has no nodes";
        return false;
    }
    out = std::move(result);
    return true;
}

} // namespace OpenBeamCore
