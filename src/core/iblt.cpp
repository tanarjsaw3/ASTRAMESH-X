#include "astromesh/core/iblt.hpp"

#include <stdexcept>
#include <algorithm>
#include <cstring>
#include <cmath>

namespace astromesh::core {

namespace {

// SplitMix64 step
inline uint64_t splitmix64(uint64_t x) noexcept {
    x ^= x >> 30;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27;
    x *= 0x94d049bb133111ebULL;
    x ^= x >> 31;
    return x;
}

// 64-bit FNV-1a
inline uint64_t fnv1a_64(const uint8_t* data, size_t length, uint64_t seed) noexcept {
    constexpr uint64_t kFnvPrime = 0x100000001b3ULL;
    uint64_t hash = seed;
    for (size_t i = 0; i < length; ++i) {
        hash ^= static_cast<uint64_t>(data[i]);
        hash *= kFnvPrime;
    }
    return hash;
}

} // anonymous namespace

InvertibleBloomFilter::InvertibleBloomFilter(const IBLTConfig& config)
    : config_(config) {
    if (config_.num_cells == 0) {
        throw std::invalid_argument("IBLT num_cells must be strictly greater than 0");
    }
    if (config_.num_cells > MAX_ALLOCATION_CELLS) {
        throw std::invalid_argument("IBLT num_cells exceeds maximum allocation limit");
    }
    if (config_.num_probes == 0) {
        throw std::invalid_argument("IBLT num_probes must be strictly greater than 0");
    }
    if (config_.num_probes > config_.num_cells) {
        throw std::invalid_argument("IBLT num_probes cannot exceed num_cells");
    }
    if (config_.num_probes > 20) {
        throw std::invalid_argument("IBLT num_probes exceeds reasonable maximum limit (20)");
    }

    cells_.resize(config_.num_cells);
}

InvertibleBloomFilter::InvertibleBloomFilter(size_t num_cells, size_t num_probes, uint32_t protocol_id)
    : InvertibleBloomFilter(IBLTConfig{num_cells, num_probes, protocol_id}) {}

std::optional<InvertibleBloomFilter> InvertibleBloomFilter::try_create(
    size_t num_cells, size_t num_probes, uint32_t protocol_id) noexcept {
    try {
        return InvertibleBloomFilter(num_cells, num_probes, protocol_id);
    } catch (...) {
        return std::nullopt;
    }
}

void InvertibleBloomFilter::key_to_bits(const LandmarkKey& key, uint64_t& out_low, uint64_t& out_high) noexcept {
    out_low = (static_cast<uint64_t>(key.schema_version())) |
              (static_cast<uint64_t>(key.rover_id()) << 16) |
              (static_cast<uint64_t>(key.local_index()) << 32);
    out_high = key.descriptor_hash();
}

LandmarkKey InvertibleBloomFilter::bits_to_key(uint64_t low, uint64_t high) noexcept {
    uint16_t schema = static_cast<uint16_t>(low & 0xFFFF);
    uint16_t rover = static_cast<uint16_t>((low >> 16) & 0xFFFF);
    uint32_t index = static_cast<uint32_t>((low >> 32) & 0xFFFFFFFF);
    uint64_t hash = high;

    if (schema == LandmarkKey::INVALID_SCHEMA) {
        return LandmarkKey::invalid();
    }
    return LandmarkKey(schema, rover, index, hash);
}

uint64_t InvertibleBloomFilter::compute_checksum(const LandmarkKey& key) noexcept {
    uint64_t low = 0;
    uint64_t high = 0;
    key_to_bits(key, low, high);

    uint8_t buffer[16];
    for (size_t i = 0; i < 8; ++i) {
        buffer[i] = static_cast<uint8_t>((low >> (8 * i)) & 0xFF);
        buffer[8 + i] = static_cast<uint8_t>((high >> (8 * i)) & 0xFF);
    }

    constexpr uint64_t kChecksumSeed = 0x243f6a8885a308d3ULL;
    return splitmix64(fnv1a_64(buffer, sizeof(buffer), kChecksumSeed));
}

void InvertibleBloomFilter::get_cell_indices(const LandmarkKey& key, std::vector<size_t>& out_indices) const noexcept {
    out_indices.clear();
    out_indices.reserve(config_.num_probes);

    uint64_t low = 0;
    uint64_t high = 0;
    key_to_bits(key, low, high);

    uint8_t buffer[16];
    for (size_t i = 0; i < 8; ++i) {
        buffer[i] = static_cast<uint8_t>((low >> (8 * i)) & 0xFF);
        buffer[8 + i] = static_cast<uint8_t>((high >> (8 * i)) & 0xFF);
    }

    constexpr uint64_t kSeed1 = 0x1122334455667788ULL;
    constexpr uint64_t kSeed2 = 0x99AABBCCDDEEFF00ULL;

    uint64_t h1 = splitmix64(fnv1a_64(buffer, sizeof(buffer), kSeed1));
    uint64_t h2 = splitmix64(fnv1a_64(buffer, sizeof(buffer), kSeed2));

    if ((h2 & 1ULL) == 0) {
        h2 += 1ULL;
    }

    size_t step = 0;
    while (out_indices.size() < config_.num_probes) {
        uint64_t combined = h1 + static_cast<uint64_t>(step) * h2;
        size_t idx = static_cast<size_t>(combined % config_.num_cells);
        if (std::find(out_indices.begin(), out_indices.end(), idx) == out_indices.end()) {
            out_indices.push_back(idx);
        }
        ++step;
    }
}

bool InvertibleBloomFilter::is_pure_cell(const IBLTCell& cell, LandmarkKey& out_key) noexcept {
    if (std::abs(cell.count) != 1) {
        return false;
    }

    LandmarkKey candidate = bits_to_key(cell.key_sum_low, cell.key_sum_high);
    if (!candidate.is_valid()) {
        return false;
    }

    uint64_t expected_chk = compute_checksum(candidate);
    if (expected_chk != cell.checksum_sum) {
        return false;
    }

    out_key = candidate;
    return true;
}

bool InvertibleBloomFilter::insert(const LandmarkKey& key) {
    if (!key.is_valid()) {
        return false;
    }

    // Enforce set semantics
    if (local_set_.find(key) != local_set_.end()) {
        return false;
    }
    local_set_.insert(key);

    uint64_t low = 0;
    uint64_t high = 0;
    key_to_bits(key, low, high);
    uint64_t chk = compute_checksum(key);

    std::vector<size_t> indices;
    get_cell_indices(key, indices);

    for (size_t idx : indices) {
        if (cells_[idx].count == std::numeric_limits<int32_t>::max()) {
            throw std::overflow_error("IBLT cell count reached int32 max overflow");
        }
        cells_[idx].count += 1;
        cells_[idx].key_sum_low ^= low;
        cells_[idx].key_sum_high ^= high;
        cells_[idx].checksum_sum ^= chk;
    }

    return true;
}

bool InvertibleBloomFilter::remove(const LandmarkKey& key) {
    if (!key.is_valid()) {
        return false;
    }

    auto it = local_set_.find(key);
    if (it == local_set_.end()) {
        return false;
    }
    local_set_.erase(it);

    uint64_t low = 0;
    uint64_t high = 0;
    key_to_bits(key, low, high);
    uint64_t chk = compute_checksum(key);

    std::vector<size_t> indices;
    get_cell_indices(key, indices);

    for (size_t idx : indices) {
        if (cells_[idx].count == std::numeric_limits<int32_t>::min()) {
            throw std::underflow_error("IBLT cell count reached int32 min underflow");
        }
        cells_[idx].count -= 1;
        cells_[idx].key_sum_low ^= low;
        cells_[idx].key_sum_high ^= high;
        cells_[idx].checksum_sum ^= chk;
    }

    return true;
}

InvertibleBloomFilter InvertibleBloomFilter::subtract(const InvertibleBloomFilter& other) const {
    if (!is_compatible_with(other)) {
        throw std::invalid_argument("Cannot subtract IBLTs with incompatible configurations or protocol IDs");
    }

    InvertibleBloomFilter diff(config_);

    for (size_t i = 0; i < config_.num_cells; ++i) {
        int64_t net = static_cast<int64_t>(cells_[i].count) - static_cast<int64_t>(other.cells_[i].count);
        if (net < std::numeric_limits<int32_t>::min() || net > std::numeric_limits<int32_t>::max()) {
            throw std::overflow_error("Cell count overflow detected during IBLT subtraction");
        }

        diff.cells_[i].count = static_cast<int32_t>(net);
        diff.cells_[i].key_sum_low = cells_[i].key_sum_low ^ other.cells_[i].key_sum_low;
        diff.cells_[i].key_sum_high = cells_[i].key_sum_high ^ other.cells_[i].key_sum_high;
        diff.cells_[i].checksum_sum = cells_[i].checksum_sum ^ other.cells_[i].checksum_sum;
    }

    return diff;
}

IBLTDecodeResult InvertibleBloomFilter::decode() const {
    IBLTDecodeResult result;

    // Work on a private copy to leave the original IBLT completely untouched
    std::vector<IBLTCell> work_cells = cells_;
    std::vector<size_t> pure_queue;

    // Scan for all initial pure cells
    for (size_t i = 0; i < work_cells.size(); ++i) {
        LandmarkKey key;
        if (is_pure_cell(work_cells[i], key)) {
            pure_queue.push_back(i);
        }
    }

    const size_t max_peeling_ops = config_.num_cells * config_.num_probes * 2 + 1000;
    size_t peeling_ops = 0;

    while (!pure_queue.empty()) {
        if (++peeling_ops > max_peeling_ops) {
            result.status = IBLTDecodeStatus::ArithmeticOverflow;
            return result;
        }

        size_t idx = pure_queue.back();
        pure_queue.pop_back();

        LandmarkKey key;
        if (!is_pure_cell(work_cells[idx], key)) {
            // Cell contents changed as a result of peeling another key; no longer pure
            continue;
        }

        int32_t direction = work_cells[idx].count; // +1 (left-only) or -1 (right-only)

        if (direction == 1) {
            result.left_only.push_back(key);
        } else if (direction == -1) {
            result.right_only.push_back(key);
        }

        uint64_t low = 0;
        uint64_t high = 0;
        key_to_bits(key, low, high);
        uint64_t chk = compute_checksum(key);

        std::vector<size_t> indices;
        get_cell_indices(key, indices);

        for (size_t p : indices) {
            if (direction == 1) {
                work_cells[p].count -= 1;
            } else {
                work_cells[p].count += 1;
            }
            work_cells[p].key_sum_low ^= low;
            work_cells[p].key_sum_high ^= high;
            work_cells[p].checksum_sum ^= chk;

            LandmarkKey next_key;
            if (is_pure_cell(work_cells[p], next_key)) {
                pure_queue.push_back(p);
            }
        }
    }

    // Residual table check: count non-empty cells
    size_t unpeeled = 0;
    for (const auto& cell : work_cells) {
        if (!cell.is_empty()) {
            unpeeled++;
        }
    }

    result.unpeeled_cells = unpeeled;

    if (unpeeled == 0) {
        result.status = IBLTDecodeStatus::Success;
    } else {
        result.status = IBLTDecodeStatus::IncompleteResidual;
    }

    // Sort outputs for deterministic, reproducible caller access
    std::sort(result.left_only.begin(), result.left_only.end());
    std::sort(result.right_only.begin(), result.right_only.end());

    return result;
}

void InvertibleBloomFilter::clear() noexcept {
    for (auto& cell : cells_) {
        cell = IBLTCell{};
    }
    local_set_.clear();
}

const IBLTCell& InvertibleBloomFilter::cell_at(size_t index) const {
    if (index >= cells_.size()) {
        throw std::out_of_range("IBLT cell index out of range");
    }
    return cells_[index];
}

} // namespace astromesh::core
