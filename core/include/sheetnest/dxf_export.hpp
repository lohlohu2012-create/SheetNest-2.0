#pragma once
#include "nesting.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace sheetnest {

struct DxfExportDiagnostic {
  std::string instanceId;
  std::string message;
};

struct DxfExportResult {
  std::string text;
  std::size_t exportedPlacements{};
  std::vector<DxfExportDiagnostic> diagnostics;
};

DxfExportResult exportNestDxf(const std::vector<Instance>& parts,const Result& result);
bool exportNestDxfFile(const std::string& path,const std::vector<Instance>& parts,const Result& result);

}
