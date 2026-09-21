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
