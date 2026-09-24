#include "sheetnest/spatial_index.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace sheetnest {
namespace {
constexpr std::size_t kMaxBucketCellsPerEntry = 256;
constexpr double kMinCellSize = 1e-6;
}

SpatialIndex::SpatialIndex(double cellSize)
    : cellSize_(std::max(kMinCellSize, cellSize)) {}

std::size_t SpatialIndex::CellHash::operator()(const CellKey& key) const noexcept {
    const auto mix = [](std::uint64_t x) {
        x += 0x9e3779b97f4a7c15ULL;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        return x ^ (x >> 31);
    };
    return static_cast<std::size_t>(mix(static_cast<std::uint64_t>(key.x)) ^
                                    (mix(static_cast<std::uint64_t>(key.y)) << 1));
}

std::int64_t SpatialIndex::cellCoordinate(double value, double cellSize) {
    return static_cast<std::int64_t>(std::floor(value / cellSize));
}

std::size_t SpatialIndex::coveredCells(const Bounds& bounds, double cellSize) {
    const auto minX = cellCoordinate(bounds.minX, cellSize);
    const auto maxX = cellCoordinate(bounds.maxX, cellSize);
    const auto minY = cellCoordinate(bounds.minY, cellSize);
    const auto maxY = cellCoordinate(bounds.maxY, cellSize);

    const auto width = static_cast<std::uint64_t>(
        maxX >= minX ? maxX - minX + 1 : 0);
    const auto height = static_cast<std::uint64_t>(
        maxY >= minY ? maxY - minY + 1 : 0);

    if (width == 0 || height == 0) return 0;
    if (width > std::numeric_limits<std::size_t>::max() / height) {
        return std::numeric_limits<std::size_t>::max();
    }
    return static_cast<std::size_t>(width * height);
}

void SpatialIndex::clear() {
    bounds_.clear();
    buckets_.clear();
    overflow_.clear();
}

void SpatialIndex::rebuild(const std::vector<Bounds>& boundsList) {
    clear();
    bounds_.reserve(boundsList.size());
    buckets_.reserve(boundsList.size() * 2 + 1);
    for (std::size_t i = 0; i < boundsList.size(); ++i) {
        insert(i, boundsList[i]);
    }
}

void SpatialIndex::insert(std::size_t id, const Bounds& bounds) {
    if (id != bounds_.size()) {
        // Append-only between rebuilds. Sparse IDs would make query results
        // ambiguous, so callers must rebuild after structural mutations.
        return;
    }

    bounds_.push_back(bounds);

    if (coveredCells(bounds, cellSize_) > kMaxBucketCellsPerEntry) {
        overflow_.push_back(id);
        return;
    }

    const auto minX = cellCoordinate(bounds.minX, cellSize_);
    const auto maxX = cellCoordinate(bounds.maxX, cellSize_);
    const auto minY = cellCoordinate(bounds.minY, cellSize_);
    const auto maxY = cellCoordinate(bounds.maxY, cellSize_);

    for (auto x = minX; x <= maxX; ++x) {
        for (auto y = minY; y <= maxY; ++y) {
            buckets_[CellKey{x, y}].push_back(id);
            if (y == maxY) break;
        }
        if (x == maxX) break;
    }
}

std::vector<std::size_t> SpatialIndex::query(
    const Bounds& bounds,
    double padding
) const {
    const double pad = std::max(0.0, padding);
    const Bounds expanded{
        bounds.minX - pad,
        bounds.minY - pad,
        bounds.maxX + pad,
        bounds.maxY + pad
    };

    std::vector<std::size_t> result;
    if (bounds_.empty()) return result;

    std::unordered_set<std::size_t> seen;
    seen.reserve(32);

    for (const auto id : overflow_) {
        if (id < bounds_.size() && seen.insert(id).second) {
            result.push_back(id);
        }
    }

    if (coveredCells(expanded, cellSize_) > kMaxBucketCellsPerEntry * 4) {
        result.reserve(bounds_.size());
        for (std::size_t id = 0; id < bounds_.size(); ++id) {
            if (seen.insert(id).second) result.push_back(id);
        }
        return result;
    }

    const auto minX = cellCoordinate(expanded.minX, cellSize_);
    const auto maxX = cellCoordinate(expanded.maxX, cellSize_);
    const auto minY = cellCoordinate(expanded.minY, cellSize_);
    const auto maxY = cellCoordinate(expanded.maxY, cellSize_);

    for (auto x = minX; x <= maxX; ++x) {
        for (auto y = minY; y <= maxY; ++y) {
            const auto it = buckets_.find(CellKey{x, y});
            if (it != buckets_.end()) {
                for (const auto id : it->second) {
                    if (id < bounds_.size() && seen.insert(id).second) {
                        result.push_back(id);
                    }
                }
            }
            if (y == maxY) break;
        }
        if (x == maxX) break;
    }

    return result;
}

} // namespace sheetnest
