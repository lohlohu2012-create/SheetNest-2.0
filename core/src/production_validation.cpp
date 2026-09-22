#include "sheetnest/production_validation.hpp"
#include "sheetnest/spatial_index.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <tuple>

namespace sheetnest {
namespace {

constexpr double kEps = 1e-7;

struct PlacedShape {
    Polygon outer;
    std::vector<Polygon> holes;
};

PlacedShape transform(
    const Instance& instance,
    const Placement& placement
) {
    PlacedShape shape;
    shape.outer = translate(
        rotate(instance.part.outer, placement.rotation),
        placement.x,
        placement.y
    );

    shape.holes.reserve(instance.part.holes.size());
    for (const auto& hole : instance.part.holes) {
        shape.holes.push_back(
            translate(
                rotate(hole, placement.rotation),
                placement.x,
                placement.y
            )
        );
    }

    return shape;
}

double orientation(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) -
           (b.y - a.y) * (c.x - a.x);
}

bool onSegment(Point p, Point a, Point b) {
    return std::abs(orientation(a, b, p)) <= 1e-9 &&
           p.x >= std::min(a.x, b.x) - 1e-9 &&
           p.x <= std::max(a.x, b.x) + 1e-9 &&
           p.y >= std::min(a.y, b.y) - 1e-9 &&
           p.y <= std::max(a.y, b.y) + 1e-9;
}

bool segmentsIntersect(
    Point a,
    Point b,
    Point c,
    Point d
) {
    const double o1 = orientation(a, b, c);
    const double o2 = orientation(a, b, d);
    const double o3 = orientation(c, d, a);
    const double o4 = orientation(c, d, b);

    if (((o1 > 0.0 && o2 < 0.0) ||
         (o1 < 0.0 && o2 > 0.0)) &&
        ((o3 > 0.0 && o4 < 0.0) ||
         (o3 < 0.0 && o4 > 0.0))) {
        return true;
    }

    return (std::abs(o1) <= 1e-9 && onSegment(c, a, b)) ||
           (std::abs(o2) <= 1e-9 && onSegment(d, a, b)) ||
           (std::abs(o3) <= 1e-9 && onSegment(a, c, d)) ||
           (std::abs(o4) <= 1e-9 && onSegment(b, c, d));
}

double pointSegmentDistance(
    Point p,
    Point a,
    Point b
) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double lengthSquared = vx * vx + vy * vy;

    if (lengthSquared <= kEps) {
        return std::hypot(p.x - a.x, p.y - a.y);
    }

    double t =
        ((p.x - a.x) * vx + (p.y - a.y) * vy) /
        lengthSquared;

    t = std::clamp(t, 0.0, 1.0);

    const Point q{
        a.x + t * vx,
        a.y + t * vy
    };

    return std::hypot(p.x - q.x, p.y - q.y);
}

double segmentDistance(
    Point a,
    Point b,
    Point c,
    Point d
) {
    if (segmentsIntersect(a, b, c, d)) {
        return 0.0;
    }

    return std::min({
        pointSegmentDistance(a, c, d),
        pointSegmentDistance(b, c, d),
        pointSegmentDistance(c, a, b),
        pointSegmentDistance(d, a, b)
    });
}

double boundaryDistance(
    const Polygon& a,
    const Polygon& b
) {
    if (a.empty() || b.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double best = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < a.size(); ++i) {
        const Point a0 = a[i];
        const Point a1 = a[(i + 1) % a.size()];

        for (std::size_t j = 0; j < b.size(); ++j) {
            const Point b0 = b[j];
            const Point b1 = b[(j + 1) % b.size()];

            best = std::min(
                best,
                segmentDistance(a0, a1, b0, b1)
            );
        }
    }

    return best;
}

double minBoundaryDistance(
    const PlacedShape& a,
    const PlacedShape& b
) {
    double best =
        boundaryDistance(a.outer, b.outer);

    for (const auto& ah : a.holes) {
        best = std::min(
            best,
            boundaryDistance(ah, b.outer)
        );

        for (const auto& bh : b.holes) {
            best = std::min(
                best,
                boundaryDistance(ah, bh)
            );
        }
    }

    for (const auto& bh : b.holes) {
        best = std::min(
            best,
            boundaryDistance(a.outer, bh)
        );
    }

    return best;
}

bool pointInMaterial(
    const PlacedShape& shape,
    const Point& point
) {
    if (!pointInPolygon(point, shape.outer)) {
        return false;
    }

    for (const auto& hole : shape.holes) {
        if (pointInPolygon(point, hole)) {
            return false;
        }
    }

    return true;
}

bool materialOverlap(
    const PlacedShape& a,
    const PlacedShape& b
) {
    if (polygonsIntersect(a.outer, b.outer)) {
        if (boundaryDistance(a.outer, b.outer) <= kEps) {
            return true;
        }

        for (const auto& hole : b.holes) {
            if (boundaryDistance(a.outer, hole) <= kEps) {
                return true;
            }
        }

        for (const auto& hole : a.holes) {
            if (boundaryDistance(hole, b.outer) <= kEps) {
                return true;
            }
        }
    }

    for (const auto& point : a.outer) {
        if (pointInMaterial(b, point)) {
            return true;
        }
    }

    for (const auto& point : b.outer) {
        if (pointInMaterial(a, point)) {
            return true;
        }
    }

    return false;
}

bool boundsCanConflict(
    const Bounds& a,
    const Bounds& b,
    double gap
) {
    const double g = std::max(0.0, gap);

    return !(
        a.maxX + g < b.minX - kEps ||
        b.maxX + g < a.minX - kEps ||
        a.maxY + g < b.minY - kEps ||
        b.maxY + g < a.minY - kEps
    );
}

const Instance* findInstance(
    const std::unordered_map<std::string, const Instance*>& index,
    const std::string& id
) {
    const auto it = index.find(id);
    return it == index.end() ? nullptr : it->second;
}

void addIssue(
    ProductionValidationReport& report,
    ProductionValidationIssue issue
) {
    report.valid = false;

    switch (issue.type) {
    case ProductionValidationIssueType::Collision:
        ++report.collisionCount;
        break;
    case ProductionValidationIssueType::Gap:
        ++report.gapViolationCount;
        break;
    case ProductionValidationIssueType::Margin:
        ++report.marginViolationCount;
        break;
    case ProductionValidationIssueType::DuplicateId:
        ++report.duplicateIdCount;
        break;
    case ProductionValidationIssueType::MissingId:
        ++report.missingIdCount;
        break;
    case ProductionValidationIssueType::UnknownId:
        ++report.unknownIdCount;
        break;
    }

    report.issues.push_back(std::move(issue));
}

} // namespace

const char* productionValidationIssueTypeName(
    ProductionValidationIssueType type
) {
    switch (type) {
    case ProductionValidationIssueType::Collision:
        return "collision";
    case ProductionValidationIssueType::Gap:
        return "gap";
    case ProductionValidationIssueType::Margin:
        return "margin";
    case ProductionValidationIssueType::DuplicateId:
        return "duplicate ID";
    case ProductionValidationIssueType::MissingId:
        return "missing ID";
    case ProductionValidationIssueType::UnknownId:
        return "unknown ID";
    }

    return "unknown";
}

ProductionValidationReport validateProductionResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    const Result& result
) {
    ProductionValidationReport report;

    std::unordered_map<std::string, const Instance*> instancesById;
    instancesById.reserve(instances.size());

    for (const auto& instance : instances) {
        if (instance.id.empty()) {
            addIssue(
                report,
                {
                    ProductionValidationIssueType::UnknownId,
                    0,
                    instance.id,
                    {},
                    instance.unitId,
                    0.0,
                    0.0,
                    "Пустой instanceId во входном списке."
                }
            );
            continue;
        }

        const auto [it, inserted] =
            instancesById.emplace(
                instance.id,
                &instance
            );

        if (!inserted) {
            addIssue(
                report,
                {
                    ProductionValidationIssueType::DuplicateId,
                    0,
                    instance.id,
                    {},
                    instance.unitId,
                    0.0,
                    0.0,
                    "Повторяющийся instanceId во входном списке."
                }
            );
        }
    }

    struct ValidatedPlacement {
        std::size_t sheetIndex{};
        std::string id;
        const Instance* instance{};
        PlacedShape shape;
        Bounds bounds{};
    };

    std::vector<ValidatedPlacement> placed;
    placed.reserve(result.sheets.size());

    std::unordered_map<std::string, std::size_t> placementCounts;
    placementCounts.reserve(instances.size());

    const double margin =
        std::max(0.0, sheet.edgeMarginMm);
    const double requiredGap =
        std::max(0.0, options.gapMm);

    for (std::size_t sheetIndex = 0;
         sheetIndex < result.sheets.size();
         ++sheetIndex) {
        const auto& placements = result.sheets[sheetIndex];

        for (const auto& placement : placements) {
            ++report.checkedPlacements;

            const auto* instance =
                findInstance(instancesById, placement.id);

            if (!instance) {
                addIssue(
                    report,
                    {
                        ProductionValidationIssueType::UnknownId,
                        sheetIndex,
                        placement.id,
                        {},
                        {},
                        0.0,
                        0.0,
                        "Раскладка содержит instanceId, которого нет во входных деталях."
                    }
                );
                continue;
            }

            const auto count =
                ++placementCounts[placement.id];

            if (count > 1) {
                addIssue(
                    report,
                    {
                        ProductionValidationIssueType::DuplicateId,
                        sheetIndex,
                        placement.id,
                        {},
                        instance->unitId,
                        static_cast<double>(count),
                        1.0,
                        "Один instanceId размещён более одного раза."
                    }
                );
            }

            const auto shape =
                transform(*instance, placement);

            const auto shapeBounds =
                bounds(shape.outer);

            if (shapeBounds.minX <
                    margin - kEps ||
                shapeBounds.minY <
                    margin - kEps ||
                shapeBounds.maxX >
                    sheet.width - margin + kEps ||
                shapeBounds.maxY >
                    sheet.height - margin + kEps) {

                addIssue(
                    report,
                    {
                        ProductionValidationIssueType::Margin,
                        sheetIndex,
                        placement.id,
                        {},
                        instance->unitId,
                        std::min({
                            shapeBounds.minX - margin,
                            shapeBounds.minY - margin,
                            sheet.width - margin - shapeBounds.maxX,
                            sheet.height - margin - shapeBounds.maxY
                        }),
                        0.0,
                        "Контур выходит за технологическое поле листа."
                    }
                );
            }

            placed.push_back({
                sheetIndex,
                placement.id,
                instance,
                std::move(shape),
                shapeBounds
            });
        }
    }

    for (const auto& instance : instances) {
        const auto it = placementCounts.find(instance.id);

        if (instance.id.empty() ||
            it == placementCounts.end() ||
            it->second == 0) {
            addIssue(
                report,
                {
                    ProductionValidationIssueType::MissingId,
                    0,
                    instance.id,
                    {},
                    instance.unitId,
                    0.0,
                    1.0,
                    "Ожидаемый instanceId отсутствует в итоговой раскладке."
                }
            );
        }
    }

    // Build one broad-phase index per sheet. The exact material-overlap
    // and boundary-distance predicates below remain authoritative.
    std::vector<SpatialIndex> sheetIndexes(result.sheets.size());
    std::vector<std::vector<std::size_t>> sheetPlacementIds(
        result.sheets.size()
    );

    for (std::size_t globalIndex = 0;
         globalIndex < placed.size();
         ++globalIndex) {
        const auto sheetIndex = placed[globalIndex].sheetIndex;
        if (sheetIndex >= sheetIndexes.size()) continue;

        auto& ids = sheetPlacementIds[sheetIndex];
        const auto localIndex = ids.size();
        ids.push_back(globalIndex);
        sheetIndexes[sheetIndex].insert(
            localIndex,
            placed[globalIndex].bounds
        );
    }

    for (std::size_t i = 0;
         i < placed.size();
         ++i) {
        const auto sheetIndex = placed[i].sheetIndex;
        const auto nearby = sheetIndexes[sheetIndex].query(
            placed[i].bounds,
            requiredGap
        );

        for (const auto localIndex : nearby) {
            if (localIndex >= sheetPlacementIds[sheetIndex].size()) continue;
            const auto j = sheetPlacementIds[sheetIndex][localIndex];
            if (j <= i || j >= placed.size()) continue;
            if (placed[i].sheetIndex != placed[j].sheetIndex) continue;

            if (materialOverlap(
                    placed[i].shape,
                    placed[j].shape
                )) {
                addIssue(
                    report,
                    {
                        ProductionValidationIssueType::Collision,
                        placed[i].sheetIndex,
                        placed[i].id,
                        placed[j].id,
                        placed[i].instance
                            ? placed[i].instance->unitId
                            : std::string{},
                        0.0,
                        0.0,
                        "Пересечение материальных областей деталей."
                    }
                );
                continue;
            }

            const double distance =
                minBoundaryDistance(
                    placed[i].shape,
                    placed[j].shape
                );

            if (distance + kEps < requiredGap) {
                addIssue(
                    report,
                    {
                        ProductionValidationIssueType::Gap,
                        placed[i].sheetIndex,
                        placed[i].id,
                        placed[j].id,
                        placed[i].instance
                            ? placed[i].instance->unitId
                            : std::string{},
                        distance,
                        requiredGap,
                        "Расстояние между контурами меньше заданного зазора."
                    }
                );
            }
        }
    }
    return report;
}


bool repairProductionResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    Result& result,
    ProductionValidationReport* reportOut
) {
    const auto repairStarted = std::chrono::steady_clock::now();

    ProductionValidationReport initial =
        validateProductionResult(
            instances,
            sheet,
            options,
            result
        );

    if (reportOut) {
        *reportOut = initial;
    }

    if (initial.valid) {
        return false;
    }

    Options repairOptions = options;
    repairOptions.enableOptimizer = true;
    repairOptions.enableProductionValidation = true;
    repairOptions.autoRepairTimeBudgetMs = 0;
    repairOptions.iterations = std::clamp<std::size_t>(
        std::max<std::size_t>(32, options.iterations),
        32,
        128
    );

    const std::uint32_t baseSeed = options.seed;
    const std::size_t maxAttempts =
        std::clamp<std::size_t>(
            std::max<std::size_t>(1, options.autoRepairAttempts),
            1,
            16
        );
    const auto timeBudget =
        std::chrono::milliseconds(options.autoRepairTimeBudgetMs);

    auto elapsedMs = [&]() -> std::uint64_t {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - repairStarted
            ).count()
        );
    };

    auto timeExpired = [&]() -> bool {
        if (options.autoRepairTimeBudgetMs == 0) {
            return false;
        }
        return elapsedMs() >=
               static_cast<std::uint64_t>(timeBudget.count());
    };

    auto failureScore = [](
        const ProductionValidationReport& validation,
        const Result& candidate
    ) {
        return std::tuple{
            validation.issues.size(),
            candidate.unplaced.size(),
            candidate.sheets.size(),
            -candidate.utilization
        };
    };

    auto validScore = [](
        const Result& candidate
    ) {
        return std::tuple{
            candidate.unplaced.size(),
            candidate.sheets.size(),
            -candidate.utilization
        };
    };

    Result bestValid;
    bool foundValid = false;
    std::size_t actualAttempts = 0;

    Result bestFailure = result;
    ProductionValidationReport bestFailureReport = initial;
    std::size_t actualAdaptiveRounds = 0;
    std::size_t largestAdaptiveGroup = 0;
    bool selectedAdaptiveCandidate = false;
    std::vector<std::string> adaptiveConflictIds;
    std::unordered_set<std::string> adaptiveExtractedSet;
    std::vector<AdaptiveRepairRound> adaptiveHistory;
    Result adaptiveBefore = result;

    auto seedIdsFromReport = [](
        const ProductionValidationReport& report,
        const Result& candidate,
        const std::vector<Instance>& allInstances,
        double gapMm,
        std::size_t maxNeighbors,
        std::size_t roundIndex
    ) {
        std::vector<std::string> ids;
        std::unordered_set<std::string> seen;

        for (const auto& issue : report.issues) {
            if (!issue.instanceId.empty() &&
                seen.insert(issue.instanceId).second) {
                ids.push_back(issue.instanceId);
            }

            if (!issue.relatedInstanceId.empty() &&
                seen.insert(issue.relatedInstanceId).second) {
                ids.push_back(issue.relatedInstanceId);
            }
        }

        if (ids.empty() || maxNeighbors == 0) {
            return ids;
        }

        std::unordered_map<std::string, const Instance*> instanceById;
        instanceById.reserve(allInstances.size());
        for (const auto& instance : allInstances) {
            instanceById.emplace(instance.id, &instance);
        }

        struct CandidateNeighbor {
            double distance{};
            std::string id;
        };

        std::vector<CandidateNeighbor> neighbors;
        const double roundScale =
            0.35 + 0.35 * static_cast<double>(
                std::min<std::size_t>(roundIndex, 3)
            );

        for (const auto& seedId : ids) {
            const auto seedInstanceIt = instanceById.find(seedId);
            if (seedInstanceIt == instanceById.end()) continue;

            for (std::size_t sheetIndex = 0;
                 sheetIndex < candidate.sheets.size();
                 ++sheetIndex) {
                const auto& placements = candidate.sheets[sheetIndex];

                const auto seedIt = std::find_if(
                    placements.begin(),
                    placements.end(),
                    [&](const Placement& placement) {
                        return placement.id == seedId;
                    }
                );
                if (seedIt == placements.end()) continue;

                const auto seedShape =
                    transform(*seedInstanceIt->second, *seedIt);
                const auto seedBounds =
                    bounds(seedShape.outer);

                const double seedSpan =
                    std::max(
                        seedBounds.width(),
                        seedBounds.height()
                    );
                const double searchRadius =
                    std::max(
                        gapMm * 2.0,
                        seedSpan * roundScale
                    );

                const double centerX =
                    (seedBounds.minX + seedBounds.maxX) * 0.5;
                const double centerY =
                    (seedBounds.minY + seedBounds.maxY) * 0.5;

                for (const auto& placement : placements) {
                    if (placement.id == seedId ||
                        seen.contains(placement.id)) {
                        continue;
                    }

                    const auto instanceIt =
                        instanceById.find(placement.id);
                    if (instanceIt == instanceById.end()) continue;

                    const auto shape =
                        transform(*instanceIt->second, placement);
                    const auto placementBounds =
                        bounds(shape.outer);

                    const double dx =
                        ((placementBounds.minX +
                          placementBounds.maxX) * 0.5) - centerX;
                    const double dy =
                        ((placementBounds.minY +
                          placementBounds.maxY) * 0.5) - centerY;
                    const double distance =
                        std::hypot(dx, dy);

                    const double reach =
                        searchRadius +
                        0.5 * std::max(
                            placementBounds.width(),
                            placementBounds.height()
                        );

                    if (distance <= reach + kEps) {
                        neighbors.push_back({
                            distance,
                            placement.id
                        });
                    }
                }
            }
        }

        std::sort(
            neighbors.begin(),
            neighbors.end(),
            [](const CandidateNeighbor& a,
               const CandidateNeighbor& b) {
                if (std::abs(a.distance - b.distance) > kEps) {
                    return a.distance < b.distance;
                }
                return a.id < b.id;
            }
        );

        std::size_t added = 0;
        for (const auto& neighbor : neighbors) {
            if (added >= maxNeighbors) break;
            if (seen.insert(neighbor.id).second) {
                ids.push_back(neighbor.id);
                ++added;
            }
        }

        return ids;
    };

    auto makeRoundSnapshot = [](
        std::size_t roundIndex,
        const std::vector<std::string>& conflictIds,
        const std::vector<std::string>& extractedIds,
        const Result& before,
        const Result& after
    ) {
        AdaptiveRepairRound roundSnapshot;
        roundSnapshot.roundIndex = roundIndex;
        roundSnapshot.beforeSheets = before.sheets;
        roundSnapshot.afterSheets = after.sheets;
        roundSnapshot.conflictIds = conflictIds;
        roundSnapshot.extractedIds = extractedIds;

        std::unordered_set<std::string> conflictSet(
            conflictIds.begin(),
            conflictIds.end()
        );
        std::unordered_set<std::string> extractedSet(
            extractedIds.begin(),
            extractedIds.end()
        );

        std::unordered_map<std::string, Placement> afterById;
        std::unordered_map<std::string, std::size_t> afterSheetById;

        for (std::size_t sheetIndex = 0;
             sheetIndex < after.sheets.size();
             ++sheetIndex) {
            for (const auto& placement : after.sheets[sheetIndex]) {
                afterById[placement.id] = placement;
                afterSheetById[placement.id] = sheetIndex;
            }
        }

        auto samePlacement = [](
            const Placement& a,
            const Placement& b
        ) {
            constexpr double eps = 1e-6;
            return a.id == b.id &&
                   std::abs(a.x - b.x) <= eps &&
                   std::abs(a.y - b.y) <= eps &&
                   a.rotation == b.rotation;
        };

        for (std::size_t sheetIndex = 0;
             sheetIndex < before.sheets.size();
             ++sheetIndex) {
            for (const auto& beforePlacement :
                 before.sheets[sheetIndex]) {
                const auto afterIt =
                    afterById.find(beforePlacement.id);
                const auto afterSheetIt =
                    afterSheetById.find(beforePlacement.id);

                if (afterIt == afterById.end() ||
                    afterSheetIt == afterSheetById.end()) {
                    continue;
                }

                AdaptiveRepairChange change;
                change.sheetIndex = sheetIndex;
                change.afterSheetIndex = afterSheetIt->second;
                change.before = beforePlacement;
                change.after = afterIt->second;
                change.conflictGroup =
                    conflictSet.contains(beforePlacement.id);
                change.extracted =
                    extractedSet.contains(beforePlacement.id);
                change.moved =
                    !samePlacement(
                        beforePlacement,
                        afterIt->second
                    ) ||
                    afterSheetIt->second != sheetIndex;
                change.stationary = !change.moved;

                roundSnapshot.changes.push_back(change);

                if (change.moved) {
                    roundSnapshot.movedIds.push_back(
                        beforePlacement.id
                    );
                } else {
                    roundSnapshot.stationaryIds.push_back(
                        beforePlacement.id
                    );
                }
            }
        }

        return roundSnapshot;
    };

    // Stage 0: adaptive destroy-and-repair. Only the conflict-driven local
    // group is extracted; unrelated placements remain fixed. Each round is
    // immediately checked by the Production Validator, and a new conflict
    // group is derived from any remaining issues.
    if (options.enableAdaptiveDestroyRepair &&
        !timeExpired()) {
        Result adaptiveCandidate = result;
        ProductionValidationReport adaptiveReport = initial;

        for (std::size_t round = 0;
             round < std::max<std::size_t>(
                 1,
                 options.adaptiveRepairRounds
             ) &&
             !timeExpired();
             ++round) {
            if (repairOptions.control &&
                repairOptions.control->shouldStop()) {
                break;
            }

            const auto seedIds =
                seedIdsFromReport(
                    adaptiveReport,
                    adaptiveCandidate,
                    instances,
                    repairOptions.gapMm,
                    repairOptions.adaptiveRepairMaxNeighbors,
                    round
                );

            if (seedIds.empty()) {
                break;
            }

            if (round == 0) {
                adaptiveConflictIds = seedIds;
            }

            for (const auto& id : seedIds) {
                adaptiveExtractedSet.insert(id);
            }

            largestAdaptiveGroup =
                std::max(
                    largestAdaptiveGroup,
                    seedIds.size()
                );

            const Result roundBefore =
                adaptiveCandidate;

            Result localCandidate = adaptiveCandidate;
            std::vector<std::string> roundExtractedIds;

            if (!adaptiveDestroyAndRepairResult(
                    instances,
                    sheet,
                    repairOptions,
                    seedIds,
                    localCandidate,
                    &roundExtractedIds
                )) {
                break;
            }

            ++actualAdaptiveRounds;

            for (const auto& id : roundExtractedIds) {
                adaptiveExtractedSet.insert(id);
            }

            adaptiveHistory.push_back(
                makeRoundSnapshot(
                    round + 1,
                    seedIds,
                    roundExtractedIds,
                    roundBefore,
                    localCandidate
                )
            );

            adaptiveReport =
                validateProductionResult(
                    instances,
                    sheet,
                    repairOptions,
                    localCandidate
                );

            if (adaptiveReport.valid) {
                if (!foundValid ||
                    validScore(localCandidate) <
                        validScore(bestValid)) {
                    bestValid =
                        localCandidate;
                    foundValid = true;
                    selectedAdaptiveCandidate = true;
                }
                break;
            }

            if (failureScore(
                    adaptiveReport,
                    localCandidate
                ) <
                failureScore(
                    bestFailureReport,
                    bestFailure
                )) {
                bestFailure =
                    localCandidate;
                bestFailureReport =
                    adaptiveReport;
            }

            adaptiveCandidate =
                std::move(localCandidate);
        }
    }

    // Stage 1: repair the incumbent in-place. This is cheap and mirrors the
    // "relax/repack" style used by industrial nesting systems before a full
    // restart.
    {
        Result incumbent = result;
        optimizeNestingResult(
            instances,
            sheet,
            repairOptions,
            incumbent
        );

        const auto validation =
            validateProductionResult(
                instances,
                sheet,
                repairOptions,
                incumbent
            );

        bestFailure = incumbent;
        bestFailureReport = validation;

        if (validation.valid) {
            if (!foundValid ||
                validScore(incumbent) < validScore(bestValid)) {
                bestValid = std::move(incumbent);
                foundValid = true;
                selectedAdaptiveCandidate = false;
            }
        }
    }

    // Stage 1: time-bounded multi-start. Do not stop at the first valid nest:
    // continue while budget remains and keep the best valid candidate.
    for (std::size_t attempt = 0;
         attempt < maxAttempts && !timeExpired();
         ++attempt) {
        ++actualAttempts;
        if (repairOptions.control &&
            repairOptions.control->shouldStop()) {
            break;
        }

        repairOptions.seed =
            baseSeed +
            static_cast<std::uint32_t>(
                0x9E3779B9u * static_cast<std::uint32_t>(attempt + 1)
            );

        repairOptions.iterations =
            std::clamp<std::size_t>(
                32 + attempt * 16,
                32,
                128
            );

        // Diversify rotation priority without changing the permitted
        // rotation set. Different search orders can expose different
        // interlocks in tight true-shape nests.
        repairOptions.rotations = options.rotations;
        if (!repairOptions.rotations.empty()) {
            const std::size_t shift =
                attempt % repairOptions.rotations.size();
            std::rotate(
                repairOptions.rotations.begin(),
                repairOptions.rotations.begin() +
                    static_cast<std::ptrdiff_t>(shift),
                repairOptions.rotations.end()
            );
            if (attempt % 2 == 1) {
                std::reverse(
                    repairOptions.rotations.begin(),
                    repairOptions.rotations.end()
                );
            }
        }

        Result candidate =
            nest(
                instances,
                sheet,
                repairOptions
            );

        // Always polish a restart before validating it. This makes repair a
        // real two-stage search rather than just repeated greedy restarts.
        if (!timeExpired() && !(
            repairOptions.control &&
            repairOptions.control->shouldStop()
        )) {
            optimizeNestingResult(
                instances,
                sheet,
                repairOptions,
                candidate
            );
        }

        const auto validation =
            validateProductionResult(
                instances,
                sheet,
                repairOptions,
                candidate
            );

        if (!validation.valid) {
            if (failureScore(validation, candidate) <
                failureScore(bestFailureReport, bestFailure)) {
                bestFailure = std::move(candidate);
                bestFailureReport = validation;
            }
            continue;
        }

        if (!foundValid ||
            validScore(candidate) < validScore(bestValid)) {
            bestValid = std::move(candidate);
            foundValid = true;
            selectedAdaptiveCandidate = false;
        }
    }

    const auto totalElapsedMs = elapsedMs();

    if (!foundValid) {
        bestFailureReport.repairAttempts = actualAttempts;
        bestFailureReport.adaptiveRepairRounds = actualAdaptiveRounds;
        bestFailureReport.adaptiveRepairGroupSize = largestAdaptiveGroup;
        bestFailureReport.repairElapsedMs = totalElapsedMs;
        bestFailureReport.repaired = false;

        if (reportOut) {
            *reportOut = bestFailureReport;
        }
        return false;
    }

    result = std::move(bestValid);
    result.productionValidated = true;
    result.productionValid = true;
    result.productionIssueCount = 0;

    const auto finalReport =
        validateProductionResult(
            instances,
            sheet,
            repairOptions,
            result
        );

    auto completedReport = finalReport;
    completedReport.repairAttempts = actualAttempts;
    completedReport.adaptiveRepairRounds = actualAdaptiveRounds;
    completedReport.adaptiveRepairGroupSize = largestAdaptiveGroup;
    completedReport.repairElapsedMs = totalElapsedMs;
    completedReport.repaired = true;

    if (selectedAdaptiveCandidate &&
        !adaptiveHistory.empty()) {
        completedReport.adaptiveHistory =
            adaptiveHistory;
    }

    if (selectedAdaptiveCandidate &&
        actualAdaptiveRounds > 0 &&
        !adaptiveConflictIds.empty()) {
        std::unordered_set<std::string> conflictSet(
            adaptiveConflictIds.begin(),
            adaptiveConflictIds.end()
        );

        std::unordered_map<std::string, Placement> afterById;
        std::unordered_map<std::string, std::size_t> afterSheetById;
        for (std::size_t sheetIndex = 0;
             sheetIndex < bestValid.sheets.size();
             ++sheetIndex) {
            for (const auto& placement :
                 bestValid.sheets[sheetIndex]) {
                afterById[placement.id] = placement;
                afterSheetById[placement.id] = sheetIndex;