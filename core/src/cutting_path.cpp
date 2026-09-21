#include "sheetnest/cutting_path.hpp"
#include <cmath>
#include <limits>
namespace sheetnest {
namespace { double d(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);} double signedA(const Polygon&p){return signedPolygonArea(p);} }
CuttingPath planCuttingPath(const std::vector<Polygon>& cs,const CuttingParameters& p,const PathOptions& o){
  CuttingPath r; std::vector<bool> used(cs.size(),false); Point head{};
  for(int phase=0;phase<2;++phase){
    while(true){
      size_t bi=cs.size(); double bd=std::numeric_limits<double>::infinity();
      for(size_t k=0;k<cs.size();++k){
        if(used[k]||cs[k].empty())continue;
        const bool inner=signedA(cs[k])<0;
        if((phase==0)!=inner)continue;
        const double z=d(head,cs[k][0]);
        if(z<bd){bd=z;bi=k;}
      }
      if(bi==cs.size())break;
      used[bi]=true; const auto& g=cs[bi]; const Point start=g.front();
      r.moves.push_back(ToolMove{CutType::Rapid,head,start,bd,o.rapidSpeedMMin});
      r.totalRapidLengthMm+=bd;
      r.moves.push_back(ToolMove{CutType::Pierce,start,start,0,p.speedMMin}); ++r.pierces;
      for(size_t i=0;i<g.size();++i){const Point a=g[i],b=g[(i+1)%g.size()];const double z=d(a,b);r.moves.push_back(ToolMove{CutType::Cut,a,b,z,p.speedMMin});r.totalCutLengthMm+=z;}
      head=start;
    }
  }
  return r;
}
CuttingEstimate estimateCuttingPath(const CuttingPath& p,const CuttingParameters& c,const PathOptions& o){
  CuttingEstimate e; e.parameters=c; e.contourLengthMm=p.totalCutLengthMm; e.pierces=p.pierces;
  e.cuttingMinutes=c.speedMMin>0?p.totalCutLengthMm/(c.speedMMin*1000.0):0;
  e.rapidMinutes=o.rapidSpeedMMin>0?p.totalRapidLengthMm/(o.rapidSpeedMMin*1000.0):0;
  e.piercingMinutes=p.pierces*o.pierceSeconds/60.0;
  e.totalMinutes=e.cuttingMinutes+e.piercingMinutes+e.rapidMinutes;
  return e;
}
}
