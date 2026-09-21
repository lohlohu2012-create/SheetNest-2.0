#include "sheetnest/dxf.hpp"
#include "sheetnest/geometry.hpp"
#include "sheetnest/nesting.hpp"
#include "sheetnest/nfp.hpp"
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

  const Polygon fixed{{0,0},{100,0},{100,100},{0,100}};
  const Polygon moving{{0,0},{20,0},{20,10},{0,10}};
  const nfp=buildNfp(fixed,moving);
  assert(nfp.valid);
  assert(nfp.quality==NfpQuality::ExactConvex);
  const nb=bounds(nfp.boundary);
  assert(std::abs(nb.minX+20.0)<1e-9);
  assert(std::abs(nb.maxX-100.0)<1e-9);
  assert(std::abs(nb.minY+10.0)<1e-9);
  assert(std::abs(nb.maxY-100.0)<1e-9);
  assert(pointInPolygon({100,10},nfp.boundary));

  Shape insertInsideHole{
    Polygon{{40,40},{60,40},{60,60},{40,60}},{}
  };
  assert(!shapesIntersect(plate,insertInsideHole,0.0));
  assert(shapesIntersect(plate,insertInsideHole,6.0));

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

  Options opt;
  opt.iterations=8;
  opt.parallelism=2;
  opt.seed=12345;
  auto resultA=nest(parts,Sheet{210,100,0},opt);
  auto resultB=nest(parts,Sheet{210,100,0},opt);

  assert(resultA.unplaced.empty());
  assert(resultA.sheets.size()>=1 && resultA.sheets.size()<=3);
  assert(resultA.sheets.size()==resultB.sheets.size());
  assert(resultA.unplaced.size()==resultB.unplaced.size());
  assert(std::abs(resultA.utilization-resultB.utilization)<1e-12);

  std::cout<<"SheetNest core tests: OK\n";
}
