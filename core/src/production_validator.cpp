#include "sheetnest/production_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace sheetnest {
namespace {

constexpr double kEps = 1e-7;

struct PlacedShape {
    Polygon outer;
    std::vector<Polygon> holes;
};

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

bool segmentsIntersect(Point a, Point b, Point c, Point d) {
    const double o1 = orientation(a, b, c);
    const double o2 = orientation(a, b, d);
    const double o3 = orientation(c, d, a);
    const double o4 = orientation(c, d, b);

    if (((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
        ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0))) {
        return true;
    }

    return (std::abs(o1) <= 1e-9 && onSegment(c, a, b)) ||
           (std::abs(o2) <= 1e-9 && onSegment(d, a, b)) ||
           (std::abs(o3) <= 1e-9 && onSegment(a, c, d)) ||
           (std::abs(o4) <= 1e-9 && onSegment(b, c, d));
}

double pointSegmentDistance(Point p, Point a, Point b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;

    if (len2 <= kEps) {
        return std::hypot(p.x - a.x, p.y - a.y);
    }

    double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2;
    t = std::clamp(t, 0.0, 1.0);

    const Point q{
        a.x + t * vx,
        a.y + t * vy
    };

    return std::hypot(p.x - q.x, p.y - q.y);
}

double segmentDistance(Point a, Point b, Point c, Point d) {
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
    double best = boundaryDistance(a.outer, b.outer);

    for (const auto& hole : a.holes) {
        best = std::min(
            best,
            boundaryDistance(hole, b.outer)
        );

        for (const auto& otherHole : b.holes) {
            best = std::min(
                best,
                boundaryDistance(hole, otherHole)
            );
        }
    }

    for (const auto& hole : b.holes) {
        best = std::min(
            best,
            boundaryDistance(a.outer, hole)
        );
    }

    return best;
}

bool pointInMaterial(
    const PlacedShape& shape,
    const Point& p
) {
    if (!pointInPolygon(p, shape.outer)) {
        return false;
    }

    for (const auto& hole : shape.holes) {
        if (pointInPolygon(p, hole)) {
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
        bool boundaryCrossesMaterial = false;

        if (boundaryDistance(a.outer, b.outer) <= kEps) {
            boundaryCrossesMaterial = true;
        }

        for (const auto& hole : b.holes) {
            if (boundaryDistance(a.outer, hole) <= kEps) {
                boundaryCrossesMaterial = true;
                break;
            }
        }

        for (const auto& hole : a.holes) {
            if (boundaryDistance(hole, b.outer) <= kEps) {
                boundaryCrossesMaterial = true;
                break;
            }
        }

        if (boundaryCrossesMaterial) {
            return true;
        }
    }

    for (const auto& p : a.outer) {
        if (pointInMaterial(b, p)) {
            return true;
        }
    }

    for (const auto& p : b.outer) {
        if (pointInMaterial(a, p)) {
            return true;
        }
    }

    return false;
}

PlacedShape transformed(
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

const Instance* findInstance(
    const std::unordered_map<std::string, const Instance*>& byId,
    const std::string& id
) {
    const auto it = byId.find(id);
    return it == byId.end() ? nullptr : it->second;
}

void appendIssue(
    ProductionValidationReport& report,
    ProductionValidationIssue issue
) {
    switch (issue.type) {
    case ProductionValidationIssueType::Collision:
        ++report.collisionCount;
        break;
    case ProductionValidationIssueType::Gap:
        ++report.gapCount;
        break;
    case ProductionValidationIssueType::Margin:
        ++report.marginCount;
        break;
    case ProductionValidationIssueType::DuplicateId:
        ++report.duplicateIdCount;
        break;
    case ProductionValidationIssueType::MissingId:
        ++report.missingIdCount;
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
        return "duplicate-id";
    case ProductionValidationIssueType::MissingId:
        return "missing-id";
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

    std::unordered_map<std::string, const Instance*> instanceById;
    std::unordered_set<std::string> duplicateSourceIds;

    instanceById.reserve(instances.size() * 2 + 1);

    for (const auto& instance : instances) {
        const auto [it, inserted] =
            instanceById.emplace(instance.id, &instance);

        if (!inserted) {
            duplicateSourceIds.insert(instance.id);
        }
    }

    for (const auto& id : duplicateSourceIds) {
        appendIssue(
            report,
            {
                ProductionValidationIssueType::DuplicateId,
                std::numeric_limits<std::size_t>::max(),
                id,
                {},
                0.0,
                1.0,
                "Дублирующийся ID во входных instances: " + id
            }
        );
    }

    struct ValidatedPlacement {
        std::size_t sheetIndex{};
        Placement placement;
        const Instance* instance{};
        PlacedShape shape;
    };

    std::vector<ValidatedPlacement> placed;
    placed.reserve(instances.size());

    std::unordered_set<std::string> outputIds;
    outputIds.reserve(instances.size() * 2 + 1);

    const double requiredMargin = std::max(0.0, sheet.edgeMarginMm);
    const double requiredGap = std::max(0.0, options.gapMm);

    for (std::size_t sheetIndex = 0;
         sheetIndex < result.sheets.size();
         ++sheetIndex) {

        const auto& placements = result.sheets[sheetIndex];

        for (const auto& placement : placements) {
            if (!outputIds.insert(placement.id).second) {
                appendIssue(
                    report,
                    {
                        ProductionValidationIssueType::DuplicateId,
                        sheetIndex,
                        placement.id,
                        {},
                        2.0,
                        1.0,
                        "ID размещён более одного раза: " +
                            placement.id
                    }
                );
            }

            const auto* instance =
                findInstance(instanceById, placement.id);

            if (!instance) {
                appendIssue(
                    report,
                    {
                        ProductionValidationIssueType::MissingId,
                        sheetIndex,
                        placement.id,
                        {},
                        0.0,
                        1.0,
                        "Результат содержит ID, которого нет во входных instances: " +
                            placement.id
                    }
                );
                continue;
            }

            const auto shape = transformed(*instance, placement);
            const auto b = bounds(shape.outer);

            const bool inside =
                b.minX >= requiredMargin - kEps &&
                b.minY >= requiredMargin - kEps &&
                b.maxX <= sheet.width - requiredMargin + kEps &&
                b.maxY <= sheet.height - requiredMargin + kEps;

            if (!inside) {
                const double leftViolation =
                    std::max(0.0, requiredMargin - b.minX);
                const double bottomViolation =
                    std::max(0.0, requiredMargin - b.minY);
                const double rightViolation =
                    std::max(0.0, b.maxX -
                        (sheet.width - requiredMargin));
                const double topViolation =
                    std::max(0.0, b.maxY -
                        (sheet.height - requiredMargin));

                const double violation =
                    std::max({
                        leftViolation,
                        bottomViolation,
                        rightViolation,
                        topViolation
                    });

                appendIssue(
                    report,
                    {
                        ProductionValidationIssueType::Margin,
                        sheetIndex,
                        placement.id,
                        {},
                        violation,
                        requiredMargin,
                        "Деталь нарушает поле от края листа: " +
                            placement.id
                    }
                );
            }

            placed.push_back({
                sheetIndex,
                placement,
                instance,
                shape
            });
        }
    }

    for (const auto& instance : instances) {
        if (outputIds.find(instance.id) == outputIds.end()) {
            appendIssue(
                report,
                {
                    ProductionValidationIssueType::MissingId,
                    std::numeric_limits<std::size_t>::max(),
                    instance.id,
                    {},
                    0.0,
                    1.0,
                    "Instance не найден в результате раскладки: " +
                        instance.id
                }
            );
        }
    }

    for (std::size_t i = 0; i < placed.size(); ++i) {
        for (std::size_t j = i + 1; j < placed.size(); ++j) {
            const auto& a = placed[i];
            const auto& b = placed[j];

            if (a.sheetIndex != b.sheetIndex) {
                continue;
            }

            const bool overlaps =
                materialOverlap(a.shape, b.shape);

            if (overlaps) {
                appendIssue(
                    report,
                    {
                        ProductionValidationIssueType::Collision,
                        a.sheetIndex,
                        a.placement.id,
                        b.placement.id,
                        0.0,
                        requiredGap,
                        "Коллизия деталей на листе: " +
                            a.placement.id +
                            " ↔ " +
                            b.placement.id
                    }
                );
                continue;
            }

            if (requiredGap > kEps) {
                const double actualGap =
                    minBoundaryDistance(a.shape, b.shape);

                if (actualGap + kEps < requiredGap) {
                    appendIssue(
                        report,
                        {
                            ProductionValidationIssueType::Gap,
                            a.sheetIndex,
                            a.placement.id,
                            b.placement.id,
                            actualGap,
                            requiredGap,
                            "Недостаточный зазор между деталями: " +
                                a.placement.id +
                                " ↔ " +
                                b.placement.id
                        }
                    );
                }
            }
        }
    }

    report.valid = report.issues.empty();
    return report;
}

} // namespace sheetnest
