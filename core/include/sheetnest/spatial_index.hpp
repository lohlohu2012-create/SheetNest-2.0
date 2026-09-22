#pragma once

#include "geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace sheetnest {

// Lightweight uniform-grid broad-phase index. It never decides whether two
// shapes actually conflict; it only returns entries whose AABBs may conflict.
class SpatialIndex {
public:
    explicit SpatialIndex(double cellSize = 100.0);

    void clear();
    void rebuild(const std::vector<Bounds>& boundsList);
    void insert(std::size_t id, const Bounds& bounds);

    std::vector<std::size_t> query(
        const Bounds& bounds,
        double padding = 0.0
    ) const;

    std::size_t size() const { return bounds_.size(); }
    double cellSize() const { return cellSize_; }

private:
    struct CellKey {
        std::int64_t x{};
        std::int64_t y{};
        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct CellHash {
        std::size_t operator()(const CellKey& key) const noexcept;
    };

    static std::int64_t cellCoordinate(double value, double cellSize);
    static std::size_t coveredCells(const Bounds& bounds, double cellSize);

    double cellSize_;
    std::vector<Bounds> bounds_;
    std::unordered_map<CellKey, std::vector<std::size_t>, CellHash> buckets_;
    std::vector<std::size_t> overflow_;
};

} // namespace sheetnest
