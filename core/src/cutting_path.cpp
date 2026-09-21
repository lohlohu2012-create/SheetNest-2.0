#include "sheetnest/cutting_path.hpp"
#include <algorithm>
#include <cmath>
namespace sheetnest {
static double dist(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
CuttingPath planCuttingPath(const std::vector<Polygon>&cs,const CuttingParameters&p,const PathOptions&o){CuttingPath r;Point head{0,0};std::vector<bool> used(cs.size());for(size_t k=0;k<cs.size();++k){size_t bi=cs.size(),bv=0;double bd=1e100;for(size_t i=0;i<cs.size();++i)if(!used[i]&&!cs[i].empty()){double z=dist(head,cs[i][0]);if(z<bd){bd=z;bi=i;}}if(bi==cs.size())break;used[bi]=true;auto&g=cs[bi];Point start=g.front();r.moves.push_back({CutType::Rapid,head,start,bd,o.rapidSpeedMMin});r.totalRapidLengthMm+=bd;r.moves.push_back({CutType::Pierce,start,start,0,p.speedMMin});r.pierces++;for(size_t i=0;i<g.size();++i){Point a=g[i],b=g[(i+1)%g.size()];double len=dist(a,b);if(len<=0)continue;r.moves.push_back({CutType::Cut,a,b,len,p.speedMMin});r.totalCutLengthMm+=len;}head=start;}return r;}
CuttingEstimate estimateCuttingPath(const CuttingPath&path,const CuttingParameters&p,const PathOptions&o){CuttingEstimate e;e.parameters=p;e.contourLengthMm=path.totalCutLengthMm;e.pierces=path.pierces;e.cuttingMinutes=p.speedMMin>0?path.totalCutLengthMm/(p.speedMMin*1000):0;e.rapidMinutes=o.rapidSpeedMMin>0?path.totalRapidLengthMm/(o.rapidSpeedMMin*1000):0;e.piercingMinutes=path.pierces*o.pierceSeconds/60.;e.totalMinutes=e.cuttingMinutes+e.rapidMinutes+e.piercingMinutes;return e;}
}
