#include "sheetnest/cutting_path.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace sheetnest {

static double distance(Point a, Point b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

static void appendOperation(
    CuttingPath& result,
    const CuttingContour& source,
    std::size_t operationIndex,
    Point& head,
    const CuttingParameters& parameters,
    const PathOptions& options
) {
    const auto& contour = source.polygon;
    if (contour.empty()) return;

    const Point start = contour.front();
    const double rapidLength = distance(head, start);
    const double rapidSeconds =
        options.rapidSpeedMMin > 0.0
            ? rapidLength / (options.rapidSpeedMMin * 1000.0 / 60.0)
            : 0.0;

    double cutLength = 0.0;
    for (std::size_t i = 0; i < contour.size(); ++i) {
        cutLength += distance(contour[i], contour[(i + 1) % contour.size()]);
    }

    const double cuttingSeconds =
        parameters.speedMMin > 0.0
            ? cutLength / (parameters.speedMMin * 1000.0 / 60.0)
            : 0.0;

    const double pierceSeconds = std::max(0.0, options.pierceSeconds);
    const double totalSeconds =
        rapidSeconds + pierceSeconds + cuttingSeconds;

    result.operations.push_back({
        operationIndex,
        source.sheetIndex,
        source.instanceId,
        source.contourIndex,
        source.inner,
        contour,
        start,
        contour.back(),
        cutLength,
        rapidLength,
        pierceSeconds,
        cuttingSeconds,
        rapidSeconds,
        totalSeconds
    });

    if (rapidLength > 1e-9) {
        result.moves.push_back({
            CutType::Rapid,
            head,
            start,
            rapidLength,
            options.rapidSpeedMMin
        });
    }

    result.moves.push_back({
        CutType::Pierce,
        start,
        start,
        0.0,
        parameters.speedMMin
    });

    for (std::size_t i = 0; i < contour.size(); ++i) {
        const Point from = contour[i];
        const Point to = contour[(i + 1) % contour.size()];
        result.moves.push_back({
            CutType::Cut,
            from,
            to,
            distance(from, to),
            parameters.speedMMin
        });
    }

    result.totalCutLengthMm += cutLength;
    result.totalRapidLengthMm += rapidLength;
    result.totalPiercingSeconds += pierceSeconds;
    result.totalCuttingSeconds += cuttingSeconds;
    result.totalRapidSeconds += rapidSeconds;
    result.totalSeconds += totalSeconds;
    ++result.pierces;
    head = start;
}

CuttingPath planCuttingRoute(
    const std::vector<CuttingContour>& contours,
    const CuttingParameters& parameters,
    const PathOptions& options
) {
    CuttingPath result;
    std::vector<std::size_t> order;
    order.reserve(contours.size());

    for (std::size_t i = 0; i < contours.size(); ++i) {
        if (!contours[i].polygon.empty() && contours[i].polygon.size() >= 2) {
            order.push_back(i);
        }
    }

    // Stable priority: inner contours first, then nearest-neighbour within
    // that priority class. The core owns this order; GUI never recomputes it.
    Point head{};
    for (std::size_t k = 0; k < order.size(); ++k) {
        std::size_t bestPos = k;
        double bestDistance = std::numeric_limits<double>::infinity();

        bool hasInnerRemaining = false;
        if (options.innerContoursFirst) {
            for (std::size_t pos = k; pos < order.size(); ++pos) {
                if (contours[order[pos]].inner) {
                    hasInnerRemaining = true;
                    break;
                }
            }
        }

        for (std::size_t pos = k; pos < order.size(); ++pos) {
            const auto index = order[pos];
            if (hasInnerRemaining && !contours[index].inner) continue;
            const double candidateDistance =
                distance(head, contours[index].polygon.front());
            if (candidateDistance < bestDistance - 1e-9 ||
                (std::abs(candidateDistance - bestDistance) <= 1e-9 &&
                 index < order[bestPos])) {
                bestDistance = candidateDistance;
                bestPos = pos;
            }
        }

        std::swap(order[k], order[bestPos]);
        appendOperation(
            result,
            contours[order[k]],
            k,
            head,
            parameters,
            options
        );
    }

    return result;
}

CuttingPath planCuttingPath(
    const std::vector<Polygon>& contours,
    const CuttingParameters& parameters,
    const PathOptions& options
) {
    std::vector<CuttingContour> inputs;
    inputs.reserve(contours.size());
    for (std::size_t i = 0; i < contours.size(); ++i) {
        inputs.push_back({
            0,
            {},
            i,
            false,
            contours[i]
        });
    }
    return planCuttingRoute(inputs, parameters, options);
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
        path.totalPiercingSeconds / 60.0;
    result.totalMinutes =
        path.totalSeconds > 0.0
            ? path.totalSeconds / 60.0
            : result.cuttingMinutes +
              result.rapidMinutes +
              result.piercingMinutes;
    return result;
}

} // namespace sheetnest
