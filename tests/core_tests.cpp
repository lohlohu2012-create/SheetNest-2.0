#include "sheetnest/dxf.hpp"
#include "sheetnest/geometry.hpp"
#include "sheetnest/nesting.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace sheetnest;

int main() {
  Polygon outer{{0,0},{100,0},{100,100},{0,100}};
  Polygon hole{{25,25},{75,25},{75,75},{25,75}};
  Shape plate{outer,{hole}};
  assert(std::abs(polygonArea(outer)-10000.0)<1e-9);
  assert(pointInShape({10,10},plate));
  assert(!pointInShape({50,50},plate));

  const std::string dxf =
    "0\nSECTION\n2\nENTITIES\n"
    "0\nLWPOLYLINE\n70\n1\n"
    "10\n0\n20\n0\n10\n100\n20\n0\n10\n100\n20\n100\n10\n0\n20\n100\n"
    "0\nCIRCLE\n10\n50\n20\n50\n40\n10\n"
    "0\nENDSEC\n0\nEOF\n";

  const auto doc=importDxf(dxf,0.25);
  assert(!doc.contours.empty());
  assert(doc.entityCount>=2);
  bool hasHole=false;
  for(const auto& c:doc.contours) if(!c.holes.empty()) hasHole=true;
  assert(hasHole);

  Part part{"P",plate};
  std::vector<Instance> parts{{"P1",part},{"P2",part},{"P3",part}};
  auto result=nest(parts,Sheet{210,100,0},Options{});
  assert(result.unplaced.empty());
  assert(result.sheets.size()>=1 && result.sheets.size()<=3);
  std::cout<<"SheetNest core tests: OK\n";
}
