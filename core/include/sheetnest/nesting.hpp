#pragma once
#include "geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace sheetnest {

class Watchdog;

struct NestingProgress {
    std::size_t completedItems{};
    std::size_t totalItems{};
    std::size_t attempt{};
    std::size_t iterations{};
    std::string stage;
    double elapsedMs{};
    double remainingMs{};
    bool timedOut{};
    bool stopped{};
};

struct NestingTelemetry {
    std::size_t candidateChecks{};
    std::size_t boundsRejects{};
    std::size_t collisionRejects{};
    std::size_t feasibleCandidates{};
    std::size_t nfpSegmentsSampled{};
    std::size_t nfpBoundaryCandidates{};
    std::size_t nfpBudgetExceeded{};
    double nfpElapsedMs{};
};

struct Part {
    std::string id;
    Polygon outer;
    std::vector<Polygon> holes;
    std::string sourceId;
    std::string layer;
};

struct Instance {
    std::string id;
    Part part;
    std::string unitId;
};

struct Sheet {
    double width{};
    double height{};
    double edgeMarginMm{};
};

struct Placement {
    std::string id;
    double x{};
    double y{};
    int rotation{};
};

struct Result {
    std::vector<std::vector<Placement>> sheets;
    std::vector<std::string> unplaced;
    double utilization{};
    NestingTelemetry telemetry;
    bool timedOut{};
    bool stopped{};
};

struct Options {
    std::vector<int> rotations{0, 90, 180, 270};
    std::size_t iterations{24};
    double gapMm{2.0};
    std::uint32_t seed{0x534E4553u};
    double overallBudgetMs{120000.0};
    double nfpSearchBudgetMs{250.0};
    std::size_t candidateBudget{512};
    std::size_t segmentSamples{128};
    std::shared_ptr<Watchdog> watchdog;
    std::function<void(const NestingProgress&)> progress;
};

Result nest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options
);

} // namespace sheetnest
