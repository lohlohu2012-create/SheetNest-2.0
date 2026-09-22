#include "sheetnest/cutting.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sheetnest {

struct TechnologyRow {
    Material material;
    double thicknessMm;
    double speedMMin;
    const char* gas;
};

static constexpr TechnologyRow kBodor3kW[] = {
    {Material::CarbonSteel,    1,  9.0, "O2"},
    {Material::CarbonSteel,    2,  6.5, "O2"},
    {Material::CarbonSteel,    3,  3.5, "O2"},
    {Material::CarbonSteel,    4,  3.2, "O2"},
    {Material::CarbonSteel,    5,  2.75, "O2"},
    {Material::CarbonSteel,    6,  2.25, "O2"},
    {Material::CarbonSteel,    8,  1.9, "O2"},
    {Material::CarbonSteel,   10,  1.3, "O2"},
    {Material::CarbonSteel,   12,  1.2, "O2"},
    {Material::CarbonSteel,   16,  0.8, "O2"},
    {Material::CarbonSteel,   20,  0.7, "O2"},

    {Material::StainlessSteel, 1, 30.0, "N2"},
    {Material::StainlessSteel, 2, 20.0, "N2"},
    {Material::StainlessSteel, 3, 11.0, "N2"},
    {Material::StainlessSteel, 4,  8.0, "N2"},
    {Material::StainlessSteel, 5,  7.5, "N2"},
    {Material::StainlessSteel, 6,  5.0, "N2"},
    {Material::StainlessSteel, 8,  3.5, "N2"},
    {Material::StainlessSteel,10,  2.0, "N2"},
    {Material::StainlessSteel,12,  1.5, "N2"},

    {Material::Aluminum, 1, 24.0, "N2"},
    {Material::Aluminum, 2, 20.0, "N2"},
    {Material::Aluminum, 3,  7.5, "N2"},
    {Material::Aluminum, 4,  5.0, "N2"},
    {Material::Aluminum, 5,  4.5, "N2"},
    {Material::Aluminum, 6,  3.5, "N2"},
    {Material::Aluminum, 8,  2.5, "N2"},

    {Material::Brass, 1, 25.0, "N2"},
    {Material::Brass, 2, 15.0, "N2"},
    {Material::Brass, 3,  6.5, "N2"},
    {Material::Brass, 4,  5.0, "N2"},
    {Material::Brass, 5,  4.0, "N2"},
    {Material::Brass, 6,  3.0, "N2"},
    {Material::Brass, 8,  2.5, "N2"}
};

CuttingParameters bodor3kWParameters(Material material, double thicknessMm) {
    CuttingParameters result{material, thicknessMm, 0.0, {}};
    const TechnologyRow* lower = nullptr;
    const TechnologyRow* upper = nullptr;

    for (const auto& row : kBodor3kW) {
        if (row.material != material) continue;

        if (row.thicknessMm <= thicknessMm &&
            (!lower || row.thicknessMm > lower->thicknessMm)) {
            lower = &row;
        }
        if (row.thicknessMm >= thicknessMm &&
            (!upper || row.thicknessMm < upper->thicknessMm)) {
            upper = &row;
        }
    }

    // Interpolate inside the technology table instead of snapping to the
    // nearest thickness. This keeps cutting-time estimates continuous when
    // the operator enters a thickness between reference rows.
    if (lower && upper && lower != upper &&
        upper->thicknessMm > lower->thicknessMm) {
        const double t =
            (thicknessMm - lower->thicknessMm) /
            (upper->thicknessMm - lower->thicknessMm);
        result.speedMMin =
            lower->speedMMin +
            (upper->speedMMin - lower->speedMMin) * t;
        result.assistGas = lower->gas;
    } else if (lower) {
        result.speedMMin = lower->speedMMin;
        result.assistGas = lower->gas;
    } else if (upper) {
        result.speedMMin = upper->speedMMin;
        result.assistGas = upper->gas;
    }

    return result;
}

CuttingEstimate estimateCutting(
    const std::vector<CutContour>& contours,
    const CuttingParameters& parameters,
    int pierces,
    double rapidLengthMm,
    double pierceSeconds
) {
    CuttingEstimate result;
    result.parameters = parameters;
    result.pierces = std::max(0, pierces);

    for (const auto& contour : contours) {
        result.contourLengthMm += std::max(0.0, contour.lengthMm);
    }

    if (parameters.speedMMin > 0.0) {
        result.cuttingMinutes =
            result.contourLengthMm / (parameters.speedMMin * 1000.0);
        result.rapidMinutes =
            std::max(0.0, rapidLengthMm) / (120.0 * 1000.0);
    }

    result.piercingMinutes =
        static_cast<double>(result.pierces) * std::max(0.0, pierceSeconds) / 60.0;

    result.totalMinutes =
        result.cuttingMinutes +
        result.piercingMinutes +
        result.rapidMinutes;

    return result;
}

} // namespace sheetnest
