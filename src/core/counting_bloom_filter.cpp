#include "astromesh/core/counting_bloom_filter.hpp"

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <cstring>

namespace astromesh::core {

namespace {

// SplitMix64 mixing step
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

CountingBloomFilter::CountingBloomFilter(size_t expected_items, double target_fpr, bool allow_duplicates)
    : allow_duplicates_(allow_duplicates) {
    if (expected_items == 0) {
        throw std::invalid_argument("expected_items must be strictly positive (greater than 0)");
    }
    if (std::isnan(target_fpr) || target_fpr <= 0.0 || target_fpr >= 1.0) {
        throw std::invalid_argument("target_fpr must be strictly in the range (0.0, 1.0)");
    }

    // Standard optimal Bloom filter dimension formulas:
    // m = - (n * ln(p)) / (ln(2)^2)
    // k = - ln(p) / ln(2)
    constexpr double kLn2 = 0.693147180559945309417;
    constexpr double kLn2Squared = 0.480453013918201424667;

    double m_calc = -(static_cast<double>(expected_items) * std::log(target_fpr)) / kLn2Squared;
    double k_calc = -std::log(target_fpr) / kLn2;

    if (m_calc > static_cast<double>(MAX_ALLOCATION_COUNTERS)) {
        throw std::invalid_argument("Calculated counter count exceeds maximum allocation threshold");
    }

    num_counters_ = static_cast<size_t>(std::ceil(m_calc));
    if (num_counters_ < 1) {
        num_counters_ = 1;
    }

    size_t k_rounded = static_cast<size_t>(std::round(k_calc));
    if (k_rounded < 1) k_rounded = 1;
    if (k_rounded > 30) k_rounded = 30; // Cap probe count for reasonable performance
    num_probes_ = k_rounded;

    counters_.assign(num_counters_, 0);
}

CountingBloomFilter::CountingBloomFilter(const CBFConfig& config)
    : num_counters_(config.num_counters),
      num_probes_(config.num_probes),
      allow_duplicates_(config.allow_duplicates) {
    if (num_counters_ == 0) {
        throw std::invalid_argument("num_counters must be strictly greater than 0");
    }
    if (num_counters_ > MAX_ALLOCATION_COUNTERS) {
        throw std::invalid_argument("num_counters exceeds maximum allocation threshold");
    }
    if (num_probes_ == 0) {
        throw std::invalid_argument("num_probes must be strictly greater than 0");
    }
    if (num_probes_ > 100) {
        throw std::invalid_argument("num_probes exceeds reasonable maximum limit (100)");
    }

    counters_.assign(num_counters_, 0);
}

CountingBloomFilter CountingBloomFilter::create_with_dimensions(
    size_t num_counters, size_t num_probes, bool allow_duplicates) {
    CBFConfig cfg{num_counters, num_probes, allow_duplicates};
    return CountingBloomFilter(cfg);
}

std::optional<CountingBloomFilter> CountingBloomFilter::try_create(
    size_t expected_items, double target_fpr, bool allow_duplicates) noexcept {
    try {
        return CountingBloomFilter(expected_items, target_fpr, allow_duplicates);
    } catch (...) {
        return std::nullopt;
    }
}

void CountingBloomFilter::compute_double_hashes(const LandmarkKey& key, uint64_t& out_h1, uint64_t& out_h2) noexcept {
    // Pack 16 bytes: schema(2), rover(2), local_idx(4), descriptor_hash(8) in deterministic little-endian
    uint8_t buffer[16];
    uint16_t s = key.schema_version();
    uint16_t r = key.rover_id();
    uint32_t idx = key.local_index();
    uint64_t h = key.descriptor_hash();

    buffer[0] = static_cast<uint8_t>(s & 0xFF);
    buffer[1] = static_cast<uint8_t>((s >> 8) & 0xFF);
    buffer[2] = static_cast<uint8_t>(r & 0xFF);
    buffer[3] = static_cast<uint8_t>((r >> 8) & 0xFF);

    buffer[4] = static_cast<uint8_t>(idx & 0xFF);
    buffer[5] = static_cast<uint8_t>((idx >> 8) & 0xFF);
    buffer[6] = static_cast<uint8_t>((idx >> 16) & 0xFF);
    buffer[7] = static_cast<uint8_t>((idx >> 24) & 0xFF);

    for (size_t i = 0; i < 8; ++i) {
        buffer[8 + i] = static_cast<uint8_t>((h >> (8 * i)) & 0xFF);
    }

    // Two distinct seed bases for independent hash derivations
    constexpr uint64_t kSeed1 = 0xcbf29ce484222325ULL;
    constexpr uint64_t kSeed2 = 0x84222325cbf29ce4ULL;

    uint64_t raw1 = fnv1a_64(buffer, sizeof(buffer), kSeed1);
    uint64_t raw2 = fnv1a_64(buffer, sizeof(buffer), kSeed2);

    out_h1 = splitmix64(raw1);
    out_h2 = splitmix64(raw2);

    // Kirsch-Mitzenmacher requirement: step h2 must be odd (coprime to powers of two)
    if ((out_h2 & 1ULL) == 0) {
        out_h2 += 1ULL;
    }
}

void CountingBloomFilter::get_probe_indices(const LandmarkKey& key, std::vector<size_t>& out_probes) const noexcept {
    out_probes.resize(num_probes_);
    uint64_t h1 = 0;
    uint64_t h2 = 0;
    compute_double_hashes(key, h1, h2);

    for (size_t i = 0; i < num_probes_; ++i) {
        uint64_t combined = h1 + static_cast<uint64_t>(i) * h2;
        out_probes[i] = static_cast<size_t>(combined % num_counters_);
    }
}

CBFInsertResult CountingBloomFilter::insert_detailed(const LandmarkKey& key) {
    if (!key.is_valid()) {
        return CBFInsertResult::InvalidKey;
    }

    auto it = local_ledger_.find(key);
    if (it != local_ledger_.end()) {
        if (!allow_duplicates_) {
            return CBFInsertResult::Duplicate;
        }
        it->second += 1;
    } else {
        local_ledger_.emplace(key, 1);
    }

    std::vector<size_t> probes;
    get_probe_indices(key, probes);

    bool has_saturated = false;
    for (size_t p : probes) {
        if (counters_[p] < MAX_COUNTER) {
            counters_[p]++;
            if (counters_[p] == MAX_COUNTER) {
                saturated_counters_++;
                has_saturated = true;
            }
        } else {
            has_saturated = true; // Already pinned
        }
    }

    total_insertions_++;
    return has_saturated ? CBFInsertResult::CounterSaturated : CBFInsertResult::Inserted;
}

bool CountingBloomFilter::insert(const LandmarkKey& key) {
    CBFInsertResult res = insert_detailed(key);
    return res == CBFInsertResult::Inserted || res == CBFInsertResult::CounterSaturated;
}

bool CountingBloomFilter::contains(const LandmarkKey& key) const noexcept {
    if (!key.is_valid()) {
        return false;
    }

    std::vector<size_t> probes;
    get_probe_indices(key, probes);

    for (size_t p : probes) {
        if (counters_[p] == 0) {
            return false;
        }
    }
    return true;
}

CBFRemoveResult CountingBloomFilter::remove_detailed(const LandmarkKey& key) {
    if (!key.is_valid()) {
        return CBFRemoveResult::InvalidKey;
    }

    auto it = local_ledger_.find(key);
    if (it == local_ledger_.end()) {
        // Enforce deletion safety contract: uninserted keys touch NO counters.
        return CBFRemoveResult::NotPresent;
    }

    it->second -= 1;
    if (it->second == 0) {
        local_ledger_.erase(it);
    }

    std::vector<size_t> probes;
    get_probe_indices(key, probes);

    bool pinned_skipped = false;
    for (size_t p : probes) {
        if (counters_[p] == MAX_COUNTER) {
            // Saturated counters remain pinned to guarantee zero false negatives.
            pinned_skipped = true;
        } else if (counters_[p] > 0) {
            counters_[p]--;
        }
    }

    total_removals_++;
    return pinned_skipped ? CBFRemoveResult::PinnedCounterSkipped : CBFRemoveResult::Removed;
}

bool CountingBloomFilter::remove(const LandmarkKey& key) {
    CBFRemoveResult res = remove_detailed(key);
    return res == CBFRemoveResult::Removed || res == CBFRemoveResult::PinnedCounterSkipped;
}

void CountingBloomFilter::clear() noexcept {
    std::fill(counters_.begin(), counters_.end(), static_cast<uint8_t>(0));
    local_ledger_.clear();
    total_insertions_ = 0;
    total_removals_ = 0;
    saturated_counters_ = 0;
}

uint8_t CountingBloomFilter::counter_at(size_t index) const {
    if (index >= num_counters_) {
        throw std::out_of_range("Counter index out of range");
    }
    return counters_[index];
}

double CountingBloomFilter::estimated_false_positive_rate() const noexcept {
    if (num_counters_ == 0 || num_probes_ == 0) {
        return 1.0;
    }
    size_t n = active_keys();
    if (n == 0) {
        return 0.0;
    }
    double exponent = -static_cast<double>(num_probes_ * n) / static_cast<double>(num_counters_);
    double prob_single = 1.0 - std::exp(exponent);
    return std::pow(prob_single, static_cast<double>(num_probes_));
}

} // namespace astromesh::core
