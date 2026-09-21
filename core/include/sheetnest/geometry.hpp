#pragma once
#include <cstddef>
#include <vector>

namespace sheetnest {
struct Point { double x{}, y{}; };
using Polygon = std::vector<Point>;

struct Bounds {
  double minX{}, minY{}, maxX{}, maxY{};
  double width() const { return maxX - minX; }
  double height() const { return maxY - minY; }
};

struct Shape {
  Polygon outer;
  std::vector<Polygon> holes;
};

Bounds bounds(const Polygon&);
double signedPolygonArea(const Polygon&);
double polygonArea(const Polygon&);
bool pointOnSegment(const Point&, const Point&, const Point&, double eps = 1e-9);
bool pointInPolygon(const Point&, const Polygon&);
bool pointInShape(const Point&, const Shape&);
bool segmentsIntersect(const Point&, const Point&, const Point&, const Point&, double eps = 1e-9);
double segmentDistance(const Point&, const Point&, const Point&, const Point&);
double polygonBoundaryDistance(const Polygon&, const Polygon&);
bool polygonsIntersect(const Polygon&, const Polygon&);
bool shapesIntersect(const Shape&, const Shape&, double gap = 0.0);

Polygon rotate(const Polygon&, int degrees);
Polygon rotate(const Polygon&, double radians);
Polygon translate(const Polygon&, double x, double y);
Shape rotate(const Shape&, int degrees);
Shape translate(const Shape&, double x, double y);
Polygon normalized(const Polygon&);
Shape normalized(const Shape&);
}
