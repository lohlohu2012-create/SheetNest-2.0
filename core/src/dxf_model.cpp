#include "sheetnest/dxf_model.hpp"

#include <algorithm>
#include <string>

namespace sheetnest {

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

    std::vector<Instance> instances;
    if (quantityPerPart == 0) return instances;

    instances.reserve(parts.size() * quantityPerPart);

    for (const auto& part : parts) {
        for (std::size_t copy = 0; copy < quantityPerPart; ++copy) {
            Instance instance;
            instance.id =
                part.id +
                "#" +
                std::to_string(copy + 1);
            instance.part = part;
            instances.push_back(std::move(instance));
        }
    }

    return instances;
}

} // namespace sheetnest
