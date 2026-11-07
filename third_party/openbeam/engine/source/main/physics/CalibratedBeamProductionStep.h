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

/// @file
/// @brief Strict-FP production boundary for opted-in calibrated beams.

#pragma once

#include "BeamAxialKinematics.h"
#include "BeamAxialResponse.h"
#include "ProgressiveBeamDamping.h"
#include "CalibratedBeamMaterialAdapter.h"

#include <cstddef>
#include <cstdint>

namespace RoR {
namespace CalibratedBeamProductionStep {

struct Input
{
    BeamAxialKinematics::Input endpoints;
    double reference_length_m = 0.0;
    float damping_coefficient = 0.0f;
    /// BeamNG progressive (velocity-split) damping. Unauthored leaves
    /// `damping_coefficient` as the single linear rate. Already clamped at
    /// import to the coefficient the fixed-step envelope charged this beam,
    /// so selecting by velocity cannot present more damping than the linear
    /// rate did. See ProgressiveBeamDamping.h.
    ProgressiveBeamDamping::Rates progressive_damping;
    float time_step = 0.0f;
    float mass_1 = 0.0f;
    float mass_2 = 0.0f;
    bool movable_1 = false;
    bool movable_2 = false;
    bool is_plain_axial_beam = false;
};

struct Result
{
    BeamAxialKinematics::Result kinematics;
    BeamAxialResponse::DampingResult damping;
    CalibratedBeamMaterialAdapter::StepResult material;

    bool IsValid() const
    {
        return kinematics.IsValid() && material.IsValid();
    }
};

/// Computes geometry, bounded damping, the material update, and the exact
/// equal-and-opposite force pair in one strict-FP translation unit. Geometry
/// failure latches the opted-in runtime exactly once; material history remains
/// unchanged apart from that fault metadata.
Result Step(
    CalibratedBeamMaterialAdapter::Runtime& runtime,
    const Input& input);

/// The same step reusing `proof` across steps and, when `history_proven`,
/// skipping the re-check of a history the caller proved (see
/// CalibratedBeamMaterialAdapter::Step). Same results as Step above.
Result Step(
    CalibratedBeamMaterialAdapter::Runtime& runtime,
    const Input& input,
    CalibratedBeamMaterialAdapter::ConfigurationProof& proof,
    bool history_proven);

/// Stable, caller-owned identity for one beam in a production network.
/// Zero is reserved so an omitted/uninitialized identity fails closed.
using StableKey = std::uint64_t;

enum class NetworkEntryStatus : std::uint32_t
{
    NOT_EVALUATED = 0U,
    ADVANCED,
    FAULT_LATCHED,
    INACTIVE
};

/// One caller-owned production-network entry. `runtime` is the unique owner
/// associated with the prediction. `predicted_runtime`, `status`, and `result`
/// are published only on successful prediction; structural refusal leaves all
/// three fields untouched.
struct NetworkEntry
{
    StableKey stable_key = 0U;
    CalibratedBeamMaterialAdapter::Runtime* runtime = nullptr;
    Input input;
    CalibratedBeamMaterialAdapter::Runtime predicted_runtime;
    NetworkEntryStatus status = NetworkEntryStatus::NOT_EVALUATED;
    Result result;
    /// This entry's configuration admission, reused by the in-place range
    /// across steps while it names the runtime's configuration (see
    /// CalibratedBeamMaterialAdapter::ConfigurationProof). Derived state:
    /// never published, serialized or compared.
    CalibratedBeamMaterialAdapter::ConfigurationProof configuration_proof;
};

/// One preallocated transaction slot. This is deliberately public so callers
/// can own fixed-capacity storage without an allocator hidden in the physics
/// boundary.
struct NetworkWorkspaceEntry
{
    StableKey stable_key = 0U;
    std::size_t original_index = 0U;
    CalibratedBeamMaterialAdapter::Runtime* runtime_owner = nullptr;
    CalibratedBeamMaterialAdapter::Runtime predicted_runtime;
    Input input;
    NetworkEntryStatus predicted_status =
        NetworkEntryStatus::NOT_EVALUATED;
    Result predicted_result;
};

struct NetworkWorkspace
{
    NetworkWorkspaceEntry* entries = nullptr;
    std::size_t capacity = 0U;

    NetworkWorkspace() = default;
    NetworkWorkspace(
        NetworkWorkspaceEntry* workspace_entries,
        std::size_t workspace_capacity)
        : entries(workspace_entries),
          capacity(workspace_capacity)
    {
    }
};

enum class NetworkStatus : std::uint32_t
{
    COMPLETED = 0U,
    STRUCTURAL_REFUSAL
};

enum class NetworkError : std::uint32_t
{
    NONE = 0U,
    NULL_ENTRIES,
    NULL_WORKSPACE,
    INSUFFICIENT_WORKSPACE,
    ZERO_STABLE_KEY,
    DUPLICATE_STABLE_KEY,
    NULL_RUNTIME_OWNER,
    DUPLICATE_RUNTIME_OWNER
};

/// Explicit transaction receipt. Per-beam authored/numerical failures do not
/// refuse a structurally valid batch: they appear as FAULT_LATCHED entries and
/// are counted separately while unrelated beams still advance.
struct NetworkBatchResult
{
    NetworkStatus status = NetworkStatus::STRUCTURAL_REFUSAL;
    NetworkError error = NetworkError::NONE;
    std::size_t requested_count = 0U;
    std::size_t predicted_count = 0U;
    std::size_t advanced_count = 0U;
    std::size_t fault_latched_count = 0U;
    std::size_t inactive_count = 0U;
    std::size_t admission_comparison_count = 0U;

    bool IsCompleted() const
    {
        return status == NetworkStatus::COMPLETED &&
            error == NetworkError::NONE;
    }
};

/// Predicts a calibrated-beam network using only caller-owned storage without
/// mutating any runtime owner.
///
/// Basic scalar validation happens before any workspace or entry mutation.
/// Identity/owner uniqueness is then checked with deterministic, allocation-
/// free O(N log N) heapsorts in the caller-owned workspace. A duplicate found
/// in that phase may leave workspace reordered, but entries and runtime owners
/// remain untouched. Once admitted, runtime copies are evaluated in ascending
/// stable-key order. Only after every prediction completes are predicted
/// runtimes, statuses, and results published to the entries.
NetworkBatchResult PredictNetwork(
    NetworkEntry* entries,
    std::size_t entry_count,
    NetworkWorkspace workspace);

/// PredictNetwork in three phases, so a caller can evaluate disjoint index
/// ranges of one admitted batch on different threads:
///
///   AdmitNetwork         checks and stages the batch (serial);
///   EvaluateNetworkRange advances the staged entries in [begin, end)
///                        (ranges of one staged batch may run concurrently:
///                        each reads and writes only its own entries and
///                        never a runtime owner);
///   PublishNetwork       completes the batch (serial, after every range).
///
/// Together they are PredictNetwork: the same inputs, the same arithmetic
/// per entry, the same published values and counters, in any range split.
struct NetworkStaging
{
    /// The entries arrived strictly ordered by stable key and by runtime
    /// owner, so they are evaluated in place and no workspace copy exists.
    bool in_place = false;
    std::size_t entry_count = 0U;
};

struct NetworkRangeCounters
{
    std::size_t predicted_count = 0U;
    std::size_t advanced_count = 0U;
    std::size_t fault_latched_count = 0U;
    std::size_t inactive_count = 0U;
};

/// False on a structural refusal, with `batch.error` set exactly as
/// PredictNetwork would have set it.
bool AdmitNetwork(
    NetworkEntry* entries,
    std::size_t entry_count,
    NetworkWorkspace workspace,
    NetworkBatchResult& batch,
    NetworkStaging& staging);

void EvaluateNetworkRange(
    NetworkEntry* entries,
    NetworkWorkspace workspace,
    const NetworkStaging& staging,
    std::size_t begin,
    std::size_t end,
    NetworkRangeCounters& counters);

/// One entry of an in-place staged batch (`staging.in_place`): exactly the
/// step EvaluateNetworkRange takes for that entry, so a caller can snapshot
/// an entry's input, evaluate it and record its result in one pass while
/// the entry is still in cache, instead of three passes over the range.
/// Entries are independent: any order, any split, same values and counters.
void EvaluateNetworkEntryInPlace(
    NetworkEntry& entry,
    NetworkRangeCounters& counters);

/// How many consecutive entries EvaluateNetworkEntriesInPlace takes at once.
static const std::size_t IN_PLACE_BATCH_LANES = 4U;

/// EvaluateNetworkEntryInPlace for `count` consecutive entries of an
/// in-place staged batch, IN_PLACE_BATCH_LANES at a time. The entries whose
/// step stays on the ordinary path - an enabled, unfaulted runtime whose
/// configuration proof is reused on a proven, unfractured history of a
/// plain axial beam, finite geometry, an elastic update and every check of
/// the scalar step passing - advance through one lane-by-lane pass of
/// exactly the scalar step's arithmetic (the same IEEE operations in the
/// same order on the same inputs, so the same bits), which lets independent
/// entries overlap their divisions and square roots. Every other entry, and
/// any lane a check refuses, is evaluated alone by EvaluateNetworkEntryInPlace
/// from its untouched entry. Same values and counters as that function.
void EvaluateNetworkEntriesInPlace(
    NetworkEntry* entries,
    std::size_t count,
    NetworkRangeCounters& counters);

void PublishNetwork(
    NetworkEntry* entries,
    NetworkWorkspace workspace,
    const NetworkStaging& staging,
    const NetworkRangeCounters& counters,
    NetworkBatchResult& batch);

/// Commits one completed prediction only when its caller-supplied identity and
/// owner still match. This supports production loops whose earlier topology
/// decisions can make a later predicted beam ineligible for the current step.
/// Invalid/fault-latched predictions are committable so a consumed failure
/// preserves its canonical runtime fault metadata.
bool CommitNetworkPrediction(
    const NetworkEntry& entry,
    StableKey expected_stable_key,
    CalibratedBeamMaterialAdapter::Runtime* expected_runtime);

/// Atomically advances every runtime in a calibrated-beam network using only
/// caller-owned storage.
///
/// Basic scalar validation happens before workspace mutation; uniqueness
/// validation may reorder workspace as described by PredictNetwork(). Entries
/// and runtime owners remain untouched on every structural refusal. Once
/// admitted, runtime copies are evaluated in ascending stable-key order. Only
/// after every prediction completes are results published and all copied
/// runtimes committed. The implementation performs no dynamic allocation.
NetworkBatchResult StepNetwork(
    NetworkEntry* entries,
    std::size_t entry_count,
    NetworkWorkspace workspace);

const char* ToString(NetworkStatus status);
const char* ToString(NetworkError error);
const char* ToString(NetworkEntryStatus status);

} // namespace CalibratedBeamProductionStep
} // namespace RoR
