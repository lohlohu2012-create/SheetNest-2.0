#include "sheetnest/nesting.hpp"
#include <algorithm>
namespace sheetnest {
Result nest(const std::vector<Instance>&parts,const Sheet&s,const Options&o){
 Result r; std::vector<std::vector<Placement>> sheets; double area=0;
 for(const auto&i:parts){bool placed=false;auto b=bounds(i.part.outer);
  for(size_t si=0;si<sheets.size()&&!placed;++si) for(int rot:o.rotations){auto g=rotate(i.part.outer,rot);auto q=bounds(g);for(double y=0;y+q.height()<=s.height&&!placed;y+=std::max(1.0,q.height()/2))for(double x=0;x+q.width()<=s.width&&!placed;x+=std::max(1.0,q.width()/2)){bool ok=true;for(auto&p:sheets[si]){(void)p;}if(ok){sheets[si].push_back({i.id,x,y,rot});placed=true;}}}
  if(!placed)sheets.push_back({{i.id,0,0,0}}); area+=polygonArea(i.part.outer);
 }
 r.sheets=sheets;r.utilization=sheets.empty()?0:area/(sheets.size()*s.width*s.height);return r;
}
}
