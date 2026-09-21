#pragma once

#include "nesting.hpp"

#include <string>
#include <vector>

namespace sheetnest {

struct DxfExportOptions {
    bool includeSheetOutlines{true};
    bool includePartIds{true};
    double sheetSpacingMm{100.0};
};

std::string exportNestDxf(
    const Result& result,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const DxfExportOptions& options = {}
);

} // namespace sheetnest
