#include "sheetnest/dxf_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace sheetnest {


DxfPreflightReport preflightDxf(const DxfDocument& document) {
    DxfPreflightReport report;
    report.minArea = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < document.contours.size(); ++i) {
        const auto& contour = document.contours[i];
        if (contour.outer.size() < 3) {
            ++report.emptyParts;
            report.issues.push_back(
                "Контур " + std::to_string(i + 1) +
                " содержит менее 3 вершин."
            );
            continue;
        }

        double area = 0.0;
        for (std::size_t j = 0; j < contour.outer.size(); ++j) {
            const auto& a = contour.outer[j];
            const auto& b = contour.outer[(j + 1) % contour.outer.size()];
            if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
                !std::isfinite(b.x) || !std::isfinite(b.y)) {
                ++report.invalidParts;
                report.issues.push_back(
                    "Контур " + std::to_string(i + 1) +
                    " содержит нечисловые координаты."
                );
                area = 0.0;
                break;
            }
            area += a.x * b.y - b.x * a.y;
        }
        area = std::abs(area) * 0.5;

        if (area <= 1e-7) {
            ++report.invalidParts;
            report.issues.push_back(
                "Контур " + std::to_string(i + 1) +
                " имеет нулевую площадь."
            );
            continue;
        }

        ++report.validParts;
        report.minArea = std::min(report.minArea, area);
        report.maxArea = std::max(report.maxArea, area);
    }

    if (report.validParts == 0) {
        report.minArea = 0.0;
        report.maxArea = 0.0;
    }

    report.valid =
        report.validParts == document.contours.size() &&
        !document.contours.empty() &&
        !document.contours.empty() &&
        report.invalidParts == 0;

    return report;
}

std::vector<Part> partsFromDxf(const DxfDocument& document) {
    std::vector<Part> parts;
    parts.reserve(document.contours.size());

    for (std::size_t i = 0; i < document.contours.size(); ++i) {
        const auto& contour = document.contours[i];

        Part part;
        part.id = contour.sourceId.empty()
            ? "DXF_PART_" + std::to_string(i + 1)
            : contour.sourceId;
        part.sourceId = contour.sourceId;
        part.layer = contour.layer;
        part.outer = contour.outer;
        part.holes = contour.holes;

        parts.push_back(std::move(part));
    }

    return parts;
}

std::vector<Instance> instancesFromDxf(
    const DxfDocument& document,
    std::size_t quantityPerPart
) {
    const auto parts = partsFromDxf(document);
    std::vector<std::size_t> quantities(parts.size(), quantityPerPart);
    return instancesFromDxf(document, quantities);
}

std::vector<Instance> instancesFromDxf(
    const DxfDocument& document,
    const std::vector<std::size_t>& quantities
) {
    const auto parts = partsFromDxf(document);

    std::vector<Instance> instances;
    const std::size_t count = std::min(parts.size(), quantities.size());

    for (std::size_t partIndex = 0; partIndex < count; ++partIndex) {
        const auto& part = parts[partIndex];
        const std::size_t quantity = quantities[partIndex];

        for (std::size_t copy = 0; copy < quantity; ++copy) {
            Instance instance;
            instance.id =
                part.id +
                "#" +
                std::to_string(copy + 1);
            instance.part = part;
            instance.unitId =
                part.id +
                ":unit-" +
                std::to_string(copy + 1);
            instances.push_back(std::move(instance));
        }
    }

    return instances;
}

} // namespace sheetnest
