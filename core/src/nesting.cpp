#include "sheetnest/nesting.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace sheetnest {
struct PlacedGeom{Polygon poly; Placement p;};
static Polygon normalized(const Polygon&p){auto b=bounds(p);return translate(p,-b.minX,-b.minY);}
static bool fits(const Polygon&p,const Sheet&s,double gap,double x,double y){auto q=translate(p,x,y);auto b=bounds(q);return b.minX>=-1e-7&&b.minY>=-1e-7&&b.maxX<=s.width+1e-7&&b.maxY<=s.height+1e-7;}
static bool collide(const Polygon&p,const std::vector<PlacedGeom>&v,double gap){
 if(p.size()<3)return false;
 auto b=bounds(p);
 for(auto&q:v){auto c=bounds(q.poly);if(b.maxX+gap<=c.minX||c.maxX+gap<=b.minX||b.maxY+gap<=c.minY||c.maxY+gap<=b.minY)continue;
  if(polygonsIntersect(p,q.poly))return true;
  if(gap>0){for(auto&a:p)for(auto&z:q.poly)if(std::hypot(a.x-z.x,a.y-z.y)<gap)return true;}
 }
 return false;
}
Result nest(const std::vector<Instance>&parts,const Sheet&s,const Options&o){
 Result best; size_t bestSheets=std::numeric_limits<size_t>::max(); double bestUtil=-1;
 std::vector<size_t> order(parts.size());for(size_t i=0;i<parts.size();++i)order[i]=i;
 std::sort(order.begin(),order.end(),[&](size_t a,size_t b){return polygonArea(parts[a].part.outer)>polygonArea(parts[b].part.outer);});
 for(int pass=0;pass<3;++pass){
  std::vector<std::vector<PlacedGeom>> geoms; std::vector<std::vector<Placement>> sheets; double area=0;
  for(size_t oi=0;oi<order.size();++oi){const auto&i=parts[order[oi]];area+=polygonArea(i.part.outer);bool done=false;
   std::vector<int> rots=o.rotations; if(pass==1)std::reverse(rots.begin(),rots.end());
   for(size_t si=0;si<geoms.size()&&!done;++si)for(int rot:rots){auto g=normalized(rotate(i.part.outer,rot));auto b=bounds(g);
    if(b.width()>s.width||b.height()>s.height)continue;
    double step=std::max(2.0,std::min(b.width(),b.height())/2.0);
    for(double y=0;y+b.height()<=s.height+1e-7&&!done;y+=step)for(double x=0;x+b.width()<=s.width+1e-7&&!done;x+=step){
     auto placed=translate(g,x,y);if(!fits(g,s,o.gapMm,x,y)||collide(placed,geoms[si],o.gapMm))continue;
     sheets[si].push_back({i.id,x,y,rot});geoms[si].push_back({placed,{i.id,x,y,rot}});done=true;
    }
   }
   if(!done){auto g=normalized(i.part.outer);bool opened=false;for(int rot:o.rotations){auto q=normalized(rotate(i.part.outer,rot));auto b=bounds(q);if(b.width()<=s.width&&b.height()<=s.height){geoms.push_back({{translate(q,0,0),{i.id,0,0,rot}}});sheets.push_back({{i.id,0,0,rot}});opened=true;break;}}if(!opened)best.unplaced.push_back(i.id);}
  }
  double util=geoms.empty()?0:area/(geoms.size()*s.width*s.height);
  if(geoms.size()<bestSheets||(geoms.size()==bestSheets&&util>bestUtil)){bestSheets=geoms.size();bestUtil=util;best.sheets=sheets;best.utilization=util;best.unplaced.clear();}
 }
 return best;
}
}
