#pragma once

#include "dxf.hpp"

namespace sheetnest {

// Imports SVG 1.1/2D geometry into the same contour model used by DXF.
// Supported geometry: path, polygon, polyline, rect, circle, ellipse, line.
// Supported path commands: M/m L/l H/h V/v C/c S/s Q/q T/t A/a Z/z.
// Supported transforms: translate, scale, rotate, matrix.
DxfDocument importSvg(const std::string& text, double curveToleranceMm = 0.25);

} // namespace sheetnest
