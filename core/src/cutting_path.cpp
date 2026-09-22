#include "sheetnest/cutting_path.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
namespace sheetnest {

static double d(Point a, Point b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

CuttingPath planCuttingPath(
    const std::vector<Polygon>& contours,
    const CuttingParameters& parameters,
    const PathOptions& options
) {
    CuttingPath result;
    Point head{};
    std::vector<bool> used(contours.size());
    std::vector<std::size_t> order;
    order.reserve(contours.size());
    for (std::size_t i = 0; i < contours.size(); ++i) {
        if (!contours[i].empty()) order.push_back(i);
    }

    // For sheet cutting, internal contours must be cut before their enclosing
    // outer contour so the material is still mechanically supported.
    // Area ordering is a conservative geometry-only approximation that also
    // works when nesting metadata is unavailable. The nearest-neighbour pass
    // below still minimizes rapid travel within the same priority level.
    std::vector<bool> inner(contours.size(), false);
    if (options.innerContoursFirst) {
        // A contour is internal when its first point lies inside another
        // closed contour. This is more reliable than area sorting because
        // nearest-neighbour selection must never pull an outer contour ahead
        // of a hole simply because the outer contour is closer to the head.
        for (std::size_t i = 0; i < contours.size(); ++i) {
            if (contours[i].size() < 3) continue;
            const Point probe = contours[i].front();
            for (std::size_t j = 0; j < contours.size(); ++j) {
                if (i == j || contours[j].size() < 3) continue;
                if (pointInPolygon(probe, contours[j])) {
                    inner[i] = true;
                    break;
                }
            }
        }
    }

    for (std::size_t k = 0; k < order.size(); ++k) {
        std::size_t bestPos = k;
        double bestDistance = std::numeric_limits<double>::infinity();
        const std::size_t candidateCount = order.size();

        bool hasInnerRemaining = false;
        if (options.innerContoursFirst) {
            for (std::size_t pos = k; pos < candidateCount; ++pos) {
                const auto index = order[pos];
                if (inner[index] && !contours[index].empty()) {
                    hasInnerRemaining = true;
                    break;
                }
            }
        }

        for (std::size_t pos = k; pos < candidateCount; ++pos) {
            const auto index = order[pos];
            const auto& contour = contours[index];
            if (contour.empty()) continue;
            if (hasInnerRemaining && !inner[index]) continue;

            const double distance = d(head, contour.front());
            if (distance < bestDistance - 1e-9 ||
                (std::abs(distance - bestDistance) <= 1e-9 &&
                 index < order[bestPos])) {
                bestDistance = distance;
                bestPos = pos;
            }
        }
        std::swap(order[k], order[bestPos]);

        const auto& contour = contours[order[k]];
        const Point start = contour.front();
        result.moves.push_back({
            CutType::Rapid, head, start, bestDistance, options.rapidSpeedMMin
        });
        result.totalRapidLengthMm += bestDistance;
        result.moves.push_back({
            CutType::Pierce, start, start, 0.0, parameters.speedMMin
        });
        ++result.pierces;

        for (std::size_t i = 0; i < contour.size(); ++i) {
            const Point from = contour[i];
            const Point to = contour[(i + 1) % contour.size()];
            const double length = d(from, to);
            result.moves.push_back({
                CutType::Cut, from, to, length, parameters.speedMMin
            });
            result.totalCutLengthMm += length;
        }
        head = start;
    }

    return result;
}

CuttingEstimate estimateCuttingPath(
    const CuttingPath& path,
    const CuttingParameters& parameters,
    const PathOptions& options
) {
    CuttingEstimate result;
    result.parameters = parameters;
    result.contourLengthMm = path.totalCutLengthMm;
    result.pierces = path.pierces;
    result.cuttingMinutes = parameters.speedMMin > 0.0
        ? path.totalCutLengthMm / (parameters.speedMMin * 1000.0)
        : 0.0;
    result.rapidMinutes = options.rapidSpeedMMin > 0.0
        ? path.totalRapidLengthMm / (options.rapidSpeedMMin * 1000.0)
        : 0.0;
    result.piercingMinutes =
        path.pierces * std::max(0.0, options.pierceSeconds) / 60.0;
    result.totalMinutes =
        result.cuttingMinutes + result.rapidMinutes + result.piercingMinutes;
    return result;
}

} // namespace sheetnest
