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

#include "CalibratedBeamProductionStep.h"

#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#endif
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace RoR {
namespace CalibratedBeamProductionStep {
namespace {

CalibratedBeamMaterialAdapter::Error MapKinematicsError(
    BeamAxialKinematics::Error error)
{
    switch (error)
    {
    case BeamAxialKinematics::Error::NONFINITE_INPUT:
        return CalibratedBeamMaterialAdapter::Error::NONFINITE_INPUT;
    case BeamAxialKinematics::Error::INVALID_LENGTH:
        return CalibratedBeamMaterialAdapter::Error::INVALID_CURRENT_LENGTH;
    case BeamAxialKinematics::Error::NUMERIC_OVERFLOW:
        return CalibratedBeamMaterialAdapter::Error::NUMERIC_OVERFLOW;
    case BeamAxialKinematics::Error::NONE:
    default:
        return CalibratedBeamMaterialAdapter::Error::NUMERIC_OVERFLOW;
    }
}

struct StableKeyLess
{
    std::size_t* comparison_count = nullptr;

    bool operator()(
        const NetworkWorkspaceEntry& left,
        const NetworkWorkspaceEntry& right) const
    {
        ++(*comparison_count);
        return left.stable_key < right.stable_key;
    }
};

struct RuntimeOwnerLess
{
    std::size_t* comparison_count = nullptr;

    bool operator()(
        const NetworkWorkspaceEntry& left,
        const NetworkWorkspaceEntry& right) const
    {
        ++(*comparison_count);
        return std::less<CalibratedBeamMaterialAdapter::Runtime*>()(
            left.runtime_owner,
            right.runtime_owner);
    }
};

template <typename Less>
void SiftDown(
    NetworkWorkspaceEntry* entries,
    std::size_t root,
    std::size_t count,
    Less less)
{
    if (count < 2U)
        return;
    while (root <= (count - 2U) / 2U)
    {
        const std::size_t left_child = root * 2U + 1U;
        std::size_t greater_child = left_child;
        const std::size_t right_child = left_child + 1U;
        if (right_child < count &&
            less(entries[left_child], entries[right_child]))
        {
            greater_child = right_child;
        }
        if (!less(entries[root], entries[greater_child]))
            return;
        std::swap(entries[root], entries[greater_child]);
        root = greater_child;
    }
}

template <typename Less>
void HeapSort(
    NetworkWorkspaceEntry* entries,
    std::size_t count,
    Less less)
{
    if (count < 2U)
        return;

    for (std::size_t parent = count / 2U; parent > 0U; --parent)
        SiftDown(entries, parent - 1U, count, less);
    for (std::size_t remaining = count; remaining > 1U; --remaining)
    {
        std::swap(entries[0], entries[remaining - 1U]);
        SiftDown(entries, 0U, remaining - 1U, less);
    }
}

bool HasStrictStableKeyOrder(
    const NetworkWorkspaceEntry* entries,
    std::size_t count,
    std::size_t* comparison_count)
{
    for (std::size_t index = 1U; index < count; ++index)
    {
        ++(*comparison_count);
        if (!(entries[index - 1U].stable_key < entries[index].stable_key))
            return false;
    }
    return true;
}

bool HasStrictRuntimeOwnerOrder(
    const NetworkWorkspaceEntry* entries,
    std::size_t count,
    std::size_t* comparison_count)
{
    const std::less<CalibratedBeamMaterialAdapter::Runtime*> less;
    for (std::size_t index = 1U; index < count; ++index)
    {
        ++(*comparison_count);
        if (!less(entries[index - 1U].runtime_owner,
                  entries[index].runtime_owner))
        {
            return false;
        }
    }
    return true;
}

} // namespace

Result Step(
    CalibratedBeamMaterialAdapter::Runtime& runtime,
    const Input& input,
    CalibratedBeamMaterialAdapter::ConfigurationProof& proof,
    bool history_proven)
{
    Result result;
    if (!runtime.enabled)
    {
        result.material.error =
            CalibratedBeamMaterialAdapter::Error::DISABLED;
        return result;
    }
    if (runtime.faulted)
    {
        result.material.error =
            CalibratedBeamMaterialAdapter::Error::FAULT_LATCHED;
        result.material.material_error = runtime.last_material_error;
        return result;
    }

    result.kinematics = BeamAxialKinematics::Compute(input.endpoints);
    if (!result.kinematics.IsValid())
    {
        result.material = CalibratedBeamMaterialAdapter::LatchFailure(
            runtime,
            MapKinematicsError(result.kinematics.error));
        return result;
    }

    // The progressive split is a coefficient SELECTION, taken here because
    // this is the first point where the axial velocity exists. It is
    // memoryless, so it adds no per-beam state and cannot desynchronise a
    // replay, and every rate was clamped at import to the envelope charge, so
    // the clamp inside ComputeDamping below still sees a coefficient no
    // larger than the one the envelope admitted.
    result.damping = BeamAxialResponse::ComputeDamping(
        result.kinematics.axial_relative_velocity_runtime_mps,
        ProgressiveBeamDamping::Coefficient(
            result.kinematics.axial_relative_velocity_runtime_mps,
            input.progressive_damping,
            input.damping_coefficient),
        input.time_step,
        input.mass_1,
        input.mass_2,
        input.movable_1,
        input.movable_2);

    CalibratedBeamMaterialAdapter::StepInput material_input;
    material_input.reference_length_m = input.reference_length_m;
    material_input.current_length_m =
        result.kinematics.current_length_m;
    material_input.damping_force_n = result.damping.force;
    material_input.direction = {{
        input.endpoints.endpoint_1_position_m[0] -
            input.endpoints.endpoint_2_position_m[0],
        input.endpoints.endpoint_1_position_m[1] -
            input.endpoints.endpoint_2_position_m[1],
        input.endpoints.endpoint_1_position_m[2] -
            input.endpoints.endpoint_2_position_m[2]
    }};
    material_input.is_plain_axial_beam = input.is_plain_axial_beam;
    result.material = CalibratedBeamMaterialAdapter::Internal::
        StepFromValidatedKinematics(
        runtime,
        material_input,
        proof,
        history_proven,
        result.kinematics);
    return result;
}

Result Step(
    CalibratedBeamMaterialAdapter::Runtime& runtime,
    const Input& input)
{
    CalibratedBeamMaterialAdapter::ConfigurationProof proof;
    return Step(runtime, input, proof, false);
}

bool AdmitNetwork(
    NetworkEntry* entries,
    std::size_t entry_count,
    NetworkWorkspace workspace,
    NetworkBatchResult& batch,
    NetworkStaging& staging)
{
    batch = NetworkBatchResult();
    batch.requested_count = entry_count;
    staging = NetworkStaging();
    staging.entry_count = entry_count;

    // An empty network is a valid no-op and requires no backing pointers.
    if (entry_count == 0U)
    {
        staging.in_place = true;
        return true;
    }

    // Basic scalar admission is deliberately read-only. Workspace is scratch
    // after this phase; entries and runtime owners remain immutable until every
    // prediction has completed.
    if (entries == nullptr)
    {
        batch.error = NetworkError::NULL_ENTRIES;
        return false;
    }
    if (workspace.entries == nullptr)
    {
        batch.error = NetworkError::NULL_WORKSPACE;
        return false;
    }
    if (workspace.capacity < entry_count)
    {
        batch.error = NetworkError::INSUFFICIENT_WORKSPACE;
        return false;
    }

    for (std::size_t index = 0U; index < entry_count; ++index)
    {
        const NetworkEntry& entry = entries[index];
        if (entry.stable_key == 0U)
        {
            batch.error = NetworkError::ZERO_STABLE_KEY;
            return false;
        }
        if (entry.runtime == nullptr)
        {
            batch.error = NetworkError::NULL_RUNTIME_OWNER;
            return false;
        }
    }

    // The production caller submits beams in index order: keys and owners
    // are then both strictly increasing, no sort is needed, and every entry
    // can be evaluated where it is. Strict order also proves both
    // uniqueness properties, so this is the same admission as the sorted
    // path, minus two copies of every entry.
    bool ordered = true;
    for (std::size_t index = 1U; index < entry_count && ordered; ++index)
    {
        batch.admission_comparison_count += 2U;
        ordered =
            entries[index - 1U].stable_key < entries[index].stable_key &&
            std::less<CalibratedBeamMaterialAdapter::Runtime*>()(
                entries[index - 1U].runtime, entries[index].runtime);
    }
    if (ordered)
    {
        staging.in_place = true;
        return true;
    }

    for (std::size_t index = 0U; index < entry_count; ++index)
    {
        NetworkWorkspaceEntry& slot = workspace.entries[index];
        slot.stable_key = entries[index].stable_key;
        slot.original_index = index;
        slot.runtime_owner = entries[index].runtime;
        slot.predicted_runtime = *entries[index].runtime;
        slot.input = entries[index].input;
        slot.predicted_status = NetworkEntryStatus::NOT_EVALUATED;
        slot.predicted_result = Result();
    }

    StableKeyLess stable_key_less;
    stable_key_less.comparison_count = &batch.admission_comparison_count;
    if (!HasStrictStableKeyOrder(
            workspace.entries,
            entry_count,
            &batch.admission_comparison_count))
    {
        HeapSort(workspace.entries, entry_count, stable_key_less);
        for (std::size_t index = 1U; index < entry_count; ++index)
        {
            ++batch.admission_comparison_count;
            if (workspace.entries[index - 1U].stable_key ==
                workspace.entries[index].stable_key)
            {
                batch.error = NetworkError::DUPLICATE_STABLE_KEY;
                return false;
            }
        }
    }

    RuntimeOwnerLess runtime_owner_less;
    runtime_owner_less.comparison_count = &batch.admission_comparison_count;
    if (!HasStrictRuntimeOwnerOrder(
            workspace.entries,
            entry_count,
            &batch.admission_comparison_count))
    {
        HeapSort(workspace.entries, entry_count, runtime_owner_less);
        for (std::size_t index = 1U; index < entry_count; ++index)
        {
            ++batch.admission_comparison_count;
            if (workspace.entries[index - 1U].runtime_owner ==
                workspace.entries[index].runtime_owner)
            {
                batch.error = NetworkError::DUPLICATE_RUNTIME_OWNER;
                return false;
            }
        }
        HeapSort(workspace.entries, entry_count, stable_key_less);
    }
    return true;
}

namespace {

inline void PrefetchForRead(const void* address)
{
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
    _mm_prefetch(static_cast<const char*>(address), _MM_HINT_T0);
#elif defined(__GNUC__) || defined(__clang__)
    __builtin_prefetch(address, 0, 3);
#else
    (void)address;
#endif
}

NetworkEntryStatus ClassifyPrediction(
    const Result& result,
    const CalibratedBeamMaterialAdapter::Runtime& predicted_runtime,
    NetworkRangeCounters& counters)
{
    ++counters.predicted_count;
    if (result.IsValid())
    {
        ++counters.advanced_count;
        return NetworkEntryStatus::ADVANCED;
    }
    if (predicted_runtime.faulted)
    {
        ++counters.fault_latched_count;
        return NetworkEntryStatus::FAULT_LATCHED;
    }
    ++counters.inactive_count;
    return NetworkEntryStatus::INACTIVE;
}

} // namespace

void EvaluateNetworkEntryInPlace(
    NetworkEntry& entry,
    NetworkRangeCounters& counters)
{
    // The history this step starts from is proven when it is, field for
    // field, the state the previous step published for this entry and that
    // prediction advanced unfractured: that state passed the kernel's
    // post-update check when it was published, under the configuration the
    // entry's proof names (the adapter re-checks it anyway if that
    // configuration has changed). A history reset or restored behind the
    // network compares unequal and is checked in full, exactly as every
    // history was before.
    const CalibratedBeamMaterialAdapter::Runtime& owner = *entry.runtime;
    const bool history_proven =
        entry.status == NetworkEntryStatus::ADVANCED &&
        !entry.predicted_runtime.state.fractured &&
        CalibratedBeamMaterial::IdenticalState(
            entry.predicted_runtime.state, owner.state);
    entry.predicted_runtime = owner;
    entry.result = Step(
        entry.predicted_runtime,
        entry.input,
        entry.configuration_proof,
        history_proven);
    entry.status = ClassifyPrediction(
        entry.result, entry.predicted_runtime, counters);
}

namespace {

// GCC keeps small constant-count loops at -O2 unless asked; Clang unrolls
// them itself (and would warn if a forced unroll failed).
#if defined(__GNUC__) && !defined(__clang__)
#define ROR_CALIBRATED_LANES _Pragma("GCC unroll 4")
#else
#define ROR_CALIBRATED_LANES
#endif

constexpr std::size_t kLanes = IN_PLACE_BATCH_LANES;

inline bool FiniteD(double value)
{
    return CalibratedBeamMaterial::IsFinite(value);
}

inline bool FiniteF(float value)
{
    return BeamAxialResponse::IsFinite(value);
}

} // namespace

void EvaluateNetworkEntriesInPlace(
    NetworkEntry* entries,
    std::size_t count,
    NetworkRangeCounters& counters)
{
    namespace A = CalibratedBeamMaterialAdapter;
    namespace M = CalibratedBeamMaterial;
    // Each stage below runs over every lane before the next stage starts, so
    // the lanes' independent division and square-root chains overlap. Every
    // lane computes EXACTLY the scalar step's operations, in its order, on
    // its inputs: BeamAxialKinematics::Compute, ComputeDamping, and the
    // adapter's validated elastic path (UpdateWithValidatedParameters with a
    // proven history and its post-update check, then the force pair). The
    // `ok` flag collects every refusal the scalar step can make on that
    // path, and more conservatively refuses the plastic branch outright: a
    // refused lane is evaluated alone, from its untouched entry, by the
    // scalar step. Nothing is written to an entry before its commit.
    for (std::size_t first = 0U; first < count; first += kLanes)
    {
        const std::size_t lanes = std::min(kLanes, count - first);
        NetworkEntry* const batch = entries + first;
        bool ok[kLanes] = {};
        const A::Runtime* owner[kLanes] = {};
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (l >= lanes)
                continue;
            const NetworkEntry& entry = batch[l];
            owner[l] = entry.runtime;
            const A::Runtime& runtime = *entry.runtime;
            // The scalar step's gates for this path: history_proven exactly
            // as EvaluateNetworkEntryInPlace computes it, a live runtime, an
            // unfractured state (a fractured one returns early), a plain
            // axial beam, and a configuration proof StepImpl would reuse.
            const bool history_proven =
                entry.status == NetworkEntryStatus::ADVANCED &&
                !entry.predicted_runtime.state.fractured &&
                M::IdenticalState(entry.predicted_runtime.state, runtime.state);
            ok[l] = history_proven && runtime.enabled && !runtime.faulted &&
                !runtime.state.fractured && entry.input.is_plain_axial_beam &&
                entry.configuration_proof.admitted &&
                A::IdenticalConfiguration(entry.configuration_proof.configuration,
                    runtime.configuration);
        }

        // --- BeamAxialKinematics::Compute ---------------------------------
        double d[3][kLanes] = {};
        double rv[3][kLanes] = {};
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            const BeamAxialKinematics::Input& e = batch[l].input.endpoints;
            ok[l] =
                FiniteD(e.endpoint_1_position_m[0]) && FiniteD(e.endpoint_1_position_m[1]) &&
                FiniteD(e.endpoint_1_position_m[2]) &&
                FiniteD(e.endpoint_2_position_m[0]) && FiniteD(e.endpoint_2_position_m[1]) &&
                FiniteD(e.endpoint_2_position_m[2]) &&
                FiniteD(e.endpoint_1_velocity_mps[0]) && FiniteD(e.endpoint_1_velocity_mps[1]) &&
                FiniteD(e.endpoint_1_velocity_mps[2]) &&
                FiniteD(e.endpoint_2_velocity_mps[0]) && FiniteD(e.endpoint_2_velocity_mps[1]) &&
                FiniteD(e.endpoint_2_velocity_mps[2]);
            for (std::size_t c = 0U; c < 3U; ++c)
            {
                d[c][l] = e.endpoint_1_position_m[c] - e.endpoint_2_position_m[c];
                rv[c][l] = e.endpoint_1_velocity_mps[c] - e.endpoint_2_velocity_mps[c];
            }
        }
        double scale[kLanes] = {};
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            ok[l] = ok[l] &&
                FiniteD(d[0][l]) && FiniteD(d[1][l]) && FiniteD(d[2][l]) &&
                FiniteD(rv[0][l]) && FiniteD(rv[1][l]) && FiniteD(rv[2][l]);
            scale[l] = std::max(
                std::abs(d[0][l]),
                std::max(std::abs(d[1][l]), std::abs(d[2][l])));
            ok[l] = ok[l] && FiniteD(scale[l]) && scale[l] > 0.0;
        }
        double s[3][kLanes] = {};
        double squared[kLanes] = {};
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            s[0][l] = d[0][l] / scale[l];
            s[1][l] = d[1][l] / scale[l];
            s[2][l] = d[2][l] / scale[l];
            squared[l] = s[0][l] * s[0][l] + s[1][l] * s[1][l] + s[2][l] * s[2][l];
            ok[l] = FiniteD(squared[l]) && squared[l] > 0.0;
        }
        double inverse[kLanes] = {};
        double length[kLanes] = {};
        const double minimum_length = std::sqrt(
            static_cast<double>(BeamAxialResponse::MIN_LENGTH_SQUARED));
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            inverse[l] = 1.0 / std::sqrt(squared[l]);
            length[l] = scale[l] / inverse[l];
            ok[l] = FiniteD(inverse[l]) && inverse[l] > 0.0 && FiniteD(length[l]) &&
                !(length[l] <= minimum_length);
        }
        double dir[3][kLanes] = {};
        double axial[kLanes] = {};
        float length_rt[kLanes] = {};
        float axial_rt[kLanes] = {};
        const double runtime_maximum =
            static_cast<double>(std::numeric_limits<float>::max());
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            dir[0][l] = s[0][l] * inverse[l];
            dir[1][l] = s[1][l] * inverse[l];
            dir[2][l] = s[2][l] * inverse[l];
            ok[l] = FiniteD(dir[0][l]) && FiniteD(dir[1][l]) && FiniteD(dir[2][l]);
            axial[l] = rv[0][l] * dir[0][l] + rv[1][l] * dir[1][l] + rv[2][l] * dir[2][l];
            ok[l] = ok[l] && FiniteD(axial[l]) &&
                !(length[l] > runtime_maximum || std::abs(axial[l]) > runtime_maximum);
            if (!ok[l])
                continue;
            length_rt[l] = static_cast<float>(length[l]);
            axial_rt[l] = static_cast<float>(axial[l]);
            ok[l] = FiniteF(length_rt[l]) && FiniteF(axial_rt[l]);
        }

        // --- ComputeDamping (the same inline functions, per lane) ----------
        BeamAxialResponse::DampingResult damping[kLanes];
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            const Input& in = batch[l].input;
            damping[l] = BeamAxialResponse::ComputeDamping(
                axial_rt[l],
                ProgressiveBeamDamping::Coefficient(
                    axial_rt[l], in.progressive_damping, in.damping_coefficient),
                in.time_step,
                in.mass_1,
                in.mass_2,
                in.movable_1,
                in.movable_2);
        }

        // --- StepImpl on validated kinematics, up to the material update --
        double reference[kLanes] = {};
        double damping_force[kLanes] = {};
        double strain[kLanes] = {};
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            reference[l] = batch[l].input.reference_length_m;
            damping_force[l] = damping[l].force;
            ok[l] = FiniteD(reference[l]) && FiniteD(length[l]) && FiniteD(damping_force[l]) &&
                !(reference[l] <= 0.0) && !(length[l] <= 0.0);
            const double length_delta = length[l] - reference[l];
            strain[l] = length_delta / reference[l];
            ok[l] = ok[l] && FiniteD(length_delta) && FiniteD(strain[l]);
        }

        // --- UpdateWithValidatedParameters, proven history, elastic -------
        double stress[kLanes] = {};
        double stored[kLanes] = {};
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            const M::Parameters& p = owner[l]->configuration.material;
            const M::State& prev = owner[l]->state;
            const double fracture = batch[l].configuration_proof.fracture_plastic_strain;
            const double total = strain[l];
            bool good = FiniteD(total);
            // TryStoredEnergy(previous last strain, previous state)
            const double prev_elastic = prev.last_total_strain - prev.plastic_strain;
            const double prev_elastic_energy = 0.5 * p.elastic_modulus * prev_elastic * prev_elastic;
            const double hardening_energy = 0.5 * p.hardening_modulus *
                prev.accumulated_plastic_strain * prev.accumulated_plastic_strain;
            const double prev_undamaged = prev_elastic_energy + hardening_energy;
            const double prev_stored = (1.0 - prev.damage) * prev_undamaged;
            good = good && FiniteD(prev_elastic_energy) && FiniteD(hardening_energy) &&
                FiniteD(prev_undamaged) && FiniteD(prev_stored) && prev_stored >= 0.0;
            // TryFlowStress(previous accumulated plastic strain)
            const double flow = p.yield_stress + p.hardening_modulus * prev.accumulated_plastic_strain;
            good = good && FiniteD(flow) && flow > 0.0;
            const double prev_effective = p.elastic_modulus * (prev.last_total_strain - prev.plastic_strain);
            const double prev_nominal = (1.0 - prev.damage) * prev_effective;
            const double trial = p.elastic_modulus * (total - prev.plastic_strain);
            good = good && FiniteD(prev_effective) && FiniteD(prev_nominal) && FiniteD(trial);
            const double yield_scale = std::max(std::abs(trial), flow);
            const double yield_function = std::abs(trial) - flow;
            // The plastic branch is left to the scalar step.
            good = good && yield_function <= M::NumericTolerance(yield_scale);
            // Elastic response.
            const double elastic_strain = total - prev.plastic_strain;
            const double effective = trial;
            stress[l] = (1.0 - prev.damage) * trial;
            const double tangent = (1.0 - prev.damage) * p.elastic_modulus;
            // TryStoredEnergy(total strain, the response state)
            const double new_elastic = total - prev.plastic_strain;
            const double new_elastic_energy = 0.5 * p.elastic_modulus * new_elastic * new_elastic;
            const double new_hardening_energy = 0.5 * p.hardening_modulus *
                prev.accumulated_plastic_strain * prev.accumulated_plastic_strain;
            const double new_undamaged = new_elastic_energy + new_hardening_energy;
            stored[l] = (1.0 - prev.damage) * new_undamaged;
            good = good && FiniteD(new_elastic_energy) && FiniteD(new_hardening_energy) &&
                FiniteD(new_undamaged) && FiniteD(stored[l]) && stored[l] >= 0.0;
            const double work = stored[l] - prev_stored;
            const double peak = std::max(std::abs(prev_nominal), std::abs(stress[l]));
            // IsValidStateWithValidatedParameters(response state): the
            // previous state with this step's total strain, unfractured.
            good = good &&
                FiniteD(prev.plastic_strain) && FiniteD(prev.accumulated_plastic_strain) &&
                FiniteD(prev.damage) && FiniteD(prev.damage_driver_density) && FiniteD(total) &&
                !(prev.accumulated_plastic_strain < 0.0) && !(prev.damage < 0.0) &&
                !(prev.damage > 1.0) && !(prev.damage_driver_density < 0.0) &&
                !(prev.damage_driver_density > p.damage_driver_capacity_density);
            const double strain_scale = std::max(
                std::abs(prev.plastic_strain), prev.accumulated_plastic_strain);
            good = good && !(std::abs(prev.plastic_strain) >
                prev.accumulated_plastic_strain + M::NumericTolerance(strain_scale));
            const bool requires_fracture_event =
                prev.accumulated_plastic_strain > p.damage_onset_plastic_strain;
            good = good && !(!requires_fracture_event &&
                (prev.damage != 0.0 || prev.damage_driver_density != 0.0));
            // TryDamageDriver(accumulated plastic strain)
            double raw_damage_driver = 0.0;
            if (!(prev.accumulated_plastic_strain <= p.damage_onset_plastic_strain))
            {
                const double offset = prev.accumulated_plastic_strain - p.damage_onset_plastic_strain;
                const double onset_flow = p.yield_stress + p.hardening_modulus * p.damage_onset_plastic_strain;
                good = good && FiniteD(onset_flow) && onset_flow > 0.0;
                const double linear = onset_flow * offset;
                const double quadratic = 0.5 * p.hardening_modulus * offset * offset;
                raw_damage_driver = linear + quadratic;
                good = good && FiniteD(linear) && FiniteD(quadratic) &&
                    FiniteD(raw_damage_driver) && raw_damage_driver >= 0.0;
            }
            good = good && !(requires_fracture_event &&
                prev.accumulated_plastic_strain >= fracture);
            const double expected_work = std::min(raw_damage_driver, p.damage_driver_capacity_density);
            const double expected_damage = expected_work / p.damage_driver_capacity_density;
            good = good &&
                M::NearNumeric(prev.damage_driver_density, expected_work, p.damage_driver_capacity_density) &&
                M::NearNumeric(prev.damage, expected_damage, 1.0) && !(prev.damage >= 1.0);
            const double check_flow = p.yield_stress + p.hardening_modulus * prev.accumulated_plastic_strain;
            good = good && FiniteD(check_flow) && check_flow > 0.0;
            const double check_effective = p.elastic_modulus * (total - prev.plastic_strain);
            good = good && FiniteD(check_effective);
            const double elastic_evaluation_scale = p.elastic_modulus *
                (std::abs(total) + std::abs(prev.plastic_strain));
            good = good && FiniteD(elastic_evaluation_scale);
            const double check_scale = std::max(
                std::max(std::abs(check_effective), check_flow), elastic_evaluation_scale);
            good = good && std::abs(check_effective) <= check_flow + M::NumericTolerance(check_scale);
            // The update's own finiteness and sign checks (the plastic
            // multiplier, the dissipations and the fracture strain are zero
            // on this path).
            good = good && FiniteD(stress[l]) && FiniteD(effective) && FiniteD(elastic_strain) &&
                FiniteD(tangent) && FiniteD(stored[l]) && FiniteD(work) && FiniteD(peak) &&
                stored[l] >= 0.0 && peak >= 0.0;
            ok[l] = good;
        }

        // --- The adapter's energies and the equal-and-opposite pair --------
        double material_force[kLanes] = {};
        double stored_energy[kLanes] = {};
        double dissipated_energy[kLanes] = {};
        double axial_force[kLanes] = {};
        double force_1[3][kLanes] = {};
        ROR_CALIBRATED_LANES
        for (std::size_t l = 0U; l < kLanes; ++l)
        {
            if (!ok[l])
                continue;
            const double area = owner[l]->configuration.cross_section_area_m2;
            const double reference_volume = area * reference[l];
            material_force[l] = -stress[l] * area;
            const double dissipation_density_increment = 0.0 + 0.0;
            stored_energy[l] = stored[l] * reference_volume;
            dissipated_energy[l] = dissipation_density_increment * reference_volume;
            bool good = FiniteD(reference_volume) && !(reference_volume <= 0.0) &&
                FiniteD(material_force[l]) && FiniteD(dissipation_density_increment) &&
                FiniteD(stored_energy[l]) && FiniteD(dissipated_energy[l]);
            axial_force[l] = material_force[l] + damping_force[l];
            good = good && FiniteD(axial_force[l]) &&
                A::IsInRuntimeForceRange(axial_force[l]);
            for (std::size_t c = 0U; c < 3U; ++c)
            {
                force_1[c][l] = dir[c][l] * axial_force[l];
                good = good && A::IsInRuntimeForceRange(force_1[c][l]) &&
                    A::IsInRuntimeForceRange(-force_1[c][l]);
            }
            ok[l] = good;
        }

        // --- Commit, or evaluate the lane alone -----------------------------
        for (std::size_t l = 0U; l < lanes; ++l)
        {
            NetworkEntry& entry = batch[l];
            if (!ok[l])
            {
                EvaluateNetworkEntryInPlace(entry, counters);
                continue;
            }
            // The scalar step's writes: the predicted runtime is the owner
            // with the response state (the previous state with this step's
            // total strain) and cleared errors; the result carries the
            // kinematics, the damping and the adapter's step result.
            entry.predicted_runtime = *owner[l];
            A::Runtime& runtime = entry.predicted_runtime;
            runtime.state.last_total_strain = strain[l];
            runtime.last_error = A::Error::NONE;
            runtime.last_material_error = M::Error::NONE;
            Result& result = entry.result;
            result = Result();
            result.kinematics.current_length_m = length[l];
            result.kinematics.axial_relative_velocity_mps = axial[l];
            result.kinematics.unit_direction = {{dir[0][l], dir[1][l], dir[2][l]}};
            result.kinematics.current_length_runtime_m = length_rt[l];
            result.kinematics.axial_relative_velocity_runtime_mps = axial_rt[l];
            result.damping = damping[l];
            A::StepResult& material = result.material;
            material.total_strain = strain[l];
            material.nominal_stress_pa = stress[l];
            material.material_force_n = material_force[l];
            material.damping_force_n = damping_force[l];
            material.axial_force_n = axial_force[l];
            material.stored_energy_j = stored_energy[l];
            material.dissipated_energy_increment_j = dissipated_energy[l];
            for (std::size_t c = 0U; c < 3U; ++c)
            {
                material.forces.endpoint_1[c] = force_1[c][l];
                material.forces.endpoint_2[c] = -force_1[c][l];
            }
            entry.status = ClassifyPrediction(result, runtime, counters);
        }
    }
}

#undef ROR_CALIBRATED_LANES

void EvaluateNetworkRange(
    NetworkEntry* entries,
    NetworkWorkspace workspace,
    const NetworkStaging& staging,
    std::size_t begin,
    std::size_t end,
    NetworkRangeCounters& counters)
{
    if (end > staging.entry_count)
        end = staging.entry_count;
    if (staging.in_place)
    {
        for (std::size_t index = begin; index < end; index += IN_PLACE_BATCH_LANES)
        {
            const std::size_t lanes = std::min(IN_PLACE_BATCH_LANES, end - index);
            // The runtime owners live inside the beams, one cache miss each
            // in beam order; ask for the next batch's now.
            for (std::size_t ahead = index + lanes;
                 ahead < end && ahead < index + lanes + IN_PLACE_BATCH_LANES; ++ahead)
                PrefetchForRead(entries[ahead].runtime);
            EvaluateNetworkEntriesInPlace(entries + index, lanes, counters);
        }
        return;
    }
    // Inputs are read from the entries themselves (immutable during
    // evaluation) so a caller may snapshot them after admission.
    for (std::size_t index = begin; index < end; ++index)
    {
        NetworkWorkspaceEntry& slot = workspace.entries[index];
        slot.predicted_result = Step(
            slot.predicted_runtime, entries[slot.original_index].input);
        slot.predicted_status = ClassifyPrediction(
            slot.predicted_result, slot.predicted_runtime, counters);
    }
}

void PublishNetwork(
    NetworkEntry* entries,
    NetworkWorkspace workspace,
    const NetworkStaging& staging,
    const NetworkRangeCounters& counters,
    NetworkBatchResult& batch)
{
    batch.predicted_count = counters.predicted_count;
    batch.advanced_count = counters.advanced_count;
    batch.fault_latched_count = counters.fault_latched_count;
    batch.inactive_count = counters.inactive_count;
    if (!staging.in_place)
    {
        // Publish only after every prediction has completed. Runtime owners
        // remain untouched for callers that need to decide which entries are
        // consumed.
        for (std::size_t index = 0U; index < staging.entry_count; ++index)
        {
            const NetworkWorkspaceEntry& slot = workspace.entries[index];
            NetworkEntry& destination = entries[slot.original_index];
            destination.predicted_runtime = slot.predicted_runtime;
            destination.status = slot.predicted_status;
            destination.result = slot.predicted_result;
        }
    }
    batch.status = NetworkStatus::COMPLETED;
}

NetworkBatchResult PredictNetwork(
    NetworkEntry* entries,
    std::size_t entry_count,
    NetworkWorkspace workspace)
{
    NetworkBatchResult batch;
    NetworkStaging staging;
    if (!AdmitNetwork(entries, entry_count, workspace, batch, staging))
        return batch;
    NetworkRangeCounters counters;
    EvaluateNetworkRange(
        entries, workspace, staging, 0U, entry_count, counters);
    PublishNetwork(entries, workspace, staging, counters, batch);
    return batch;
}

bool CommitNetworkPrediction(
    const NetworkEntry& entry,
    StableKey expected_stable_key,
    CalibratedBeamMaterialAdapter::Runtime* expected_runtime)
{
    if (expected_stable_key == 0U || expected_runtime == nullptr ||
        entry.stable_key != expected_stable_key ||
        entry.runtime != expected_runtime ||
        (entry.status != NetworkEntryStatus::ADVANCED &&
         entry.status != NetworkEntryStatus::FAULT_LATCHED &&
         entry.status != NetworkEntryStatus::INACTIVE))
    {
        return false;
    }

    *expected_runtime = entry.predicted_runtime;
    return true;
}

NetworkBatchResult StepNetwork(
    NetworkEntry* entries,
    std::size_t entry_count,
    NetworkWorkspace workspace)
{
    NetworkBatchResult batch = PredictNetwork(
        entries,
        entry_count,
        workspace);
    if (!batch.IsCompleted())
        return batch;

    for (std::size_t index = 0U; index < entry_count; ++index)
    {
        NetworkEntry& entry = entries[index];
        *entry.runtime = entry.predicted_runtime;
    }
    return batch;
}

const char* ToString(NetworkStatus status)
{
    switch (status)
    {
    case NetworkStatus::COMPLETED:
        return "completed";
    case NetworkStatus::STRUCTURAL_REFUSAL:
    default:
        return "structural_refusal";
    }
}

const char* ToString(NetworkError error)
{
    switch (error)
    {
    case NetworkError::NONE:
        return "none";
    case NetworkError::NULL_ENTRIES:
        return "null_entries";
    case NetworkError::NULL_WORKSPACE:
        return "null_workspace";
    case NetworkError::INSUFFICIENT_WORKSPACE:
        return "insufficient_workspace";
    case NetworkError::ZERO_STABLE_KEY:
        return "zero_stable_key";
    case NetworkError::DUPLICATE_STABLE_KEY:
        return "duplicate_stable_key";
    case NetworkError::NULL_RUNTIME_OWNER:
        return "null_runtime_owner";
    case NetworkError::DUPLICATE_RUNTIME_OWNER:
        return "duplicate_runtime_owner";
    default:
        return "unknown";
    }
}

const char* ToString(NetworkEntryStatus status)
{
    switch (status)
    {
    case NetworkEntryStatus::NOT_EVALUATED:
        return "not_evaluated";
    case NetworkEntryStatus::ADVANCED:
        return "advanced";
    case NetworkEntryStatus::FAULT_LATCHED:
        return "fault_latched";
    case NetworkEntryStatus::INACTIVE:
        return "inactive";
    default:
        return "unknown";
    }
}

} // namespace CalibratedBeamProductionStep
} // namespace RoR
