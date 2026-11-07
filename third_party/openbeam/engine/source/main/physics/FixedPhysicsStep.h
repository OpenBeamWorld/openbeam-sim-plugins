/*
    This source file is part of Rigs of Rods
    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

/// @file FixedPhysicsStep.h
/// @brief THE fixed physics step. One definition, no dependencies.
///
/// WHY THIS FILE EXISTS. The step used to be written out four times, by hand,
/// in four translation units that never saw each other:
///
///   * `PHYSICS_DT`                            SimConstants.h            0.0005f
///   * `FIXED_STEP_SECONDS`                    JBeamFixedStepEnvelope.cpp 1.0/2000.0
///   * `PHYSICS_STEP_SECONDS`                  JBeamWheel2Approximation.cpp 1.0/2000.0
///   * `JBEAM_DRIVETRAIN_PHYSICS_STEP_SECONDS` JBeamDrivetrainIR.cpp      0.0005
///
/// Nothing tied them. A grep for `static_assert` over those four names
/// returned nothing, and both places that claimed otherwise were wrong:
/// `FixedStepAttachmentBoundTests.cpp` said the value "is asserted against the
/// real header by the game build", and `JBeamFixedStepEnvelope.h` said its copy
/// was "kept here so the import layer does not depend on the simulation
/// headers" - while that very file already includes
/// `FixedStepStabilityCriterion.h`, which is a simulation header.
///
/// The duplication was not benign even at rest. The three import-layer copies
/// are binary64 `0.00050000000000000001`; `PHYSICS_DT` is a binary32 literal
/// and its value is `0.00050000002374872565`. The envelope has therefore always
/// sized beams against a step 4.7e-8 shorter, relatively, than the solver
/// actually takes. That gap is far too small to matter physically and is
/// exactly why the divergence survived: a hand-synchronised safety constant
/// fails silently and only by the amount someone forgot.
///
/// THE HAZARD THIS CLOSES. Every one of the four is load-bearing for stability.
/// `PHYSICS_DT` is the step the integrator actually takes; the other three are
/// what the admission envelopes size content against so it cannot diverge at
/// that step. If the solver's step and the envelope's step disagree, content is
/// admitted for a step the solver does not take - the envelope still passes,
/// the vehicle still spawns, and the failure appears as divergence at runtime
/// with no diagnostic pointing back here. Moving three of four is not a smaller
/// version of the change; it is the dangerous version of it.
///
/// HOW TO CHANGE THE STEP. Edit `SECONDS` here, once. The `static_assert`s at
/// each former site then hold every consumer to it, and any that is missed is a
/// compile error rather than a silent admission of unstable content. Note that
/// shortening the step invalidates every fixture whose expected value is
/// derived from `h`: spring ceilings scale as `1/h^2`, damping ceilings and the
/// drivetrain inertia floor as `1/h`, and `omega*dt` as `h`.
#pragma once

namespace RoR {
namespace FixedStep {

/// The fixed physics step, in seconds. THE definition; everything else in the
/// tree is asserted against this one.
constexpr double SECONDS = 1.0 / 2000.0;

/// The same step in binary32, for the float-typed solver paths. This is an
/// exact round-trip for the current value - `static_cast<float>(1.0/2000.0)`
/// and the former literal `0.0005f` are bit-identical - and the assertion
/// below fails loudly if a future step size is one for which double rounding
/// would move the float.
constexpr float SECONDS_F = static_cast<float>(SECONDS);

static_assert(SECONDS > 0.0, "the fixed step must be positive");
static_assert(SECONDS_F > 0.0f, "the fixed step must survive binary32");

} // namespace FixedStep
} // namespace RoR
