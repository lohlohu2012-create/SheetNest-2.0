#include "sheetnest/nesting.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace sheetnest {
namespace {
struct PlacedGeom { Shape shape; Placement placement; };
struct SheetState { std::vector<PlacedGeom> geoms; std::vector<Placement> placements; };
struct Candidate { double x{}, y{}; int rotation{}; };

static double area(const Shape& s) {
  double a=polygonArea(s.outer);
  for (const auto& h:s.holes) a-=polygonArea(h);
  return std::max(0.0,a);
}
static bool fitsSheet(const Shape& shape,const Sheet& s,double x,double y) {
  const auto b=bounds(shape.outer);
  return b.minX>=s.marginMm-1e-9 && b.minY>=s.marginMm-1e-9 &&
         b.maxX<=s.width-s.marginMm+1e-9 && b.maxY<=s.height-s.marginMm+1e-9;
}
static bool collides(const Shape& shape,const SheetState& sheet,double gap) {
  for (const auto& g:sheet.geoms) if (shapesIntersect(shape,g.shape,gap)) return true;
  return false;
}
static void addCandidates(std::vector<Candidate>& c,const Shape& moving,const Sheet& s,
                          const SheetState& state,int rot,double grid) {
  const auto b=bounds(moving.outer);
  const double minX=s.marginMm,minY=s.marginMm;
  const double maxX=s.width-s.marginMm-b.width(),maxY=s.height-s.marginMm-b.height();
  auto add=[&](double x,double y){if(x>=minX-1e-7&&x<=maxX+1e-7&&y>=minY-1e-7&&y<=maxY+1e-7)c.push_back({x,y,rot});};
  add(minX,minY); add(maxX,minY); add(minX,maxY); add(maxX,maxY);
  for (const auto& pg:state.geoms) {
    const auto pb=bounds(pg.shape.outer);
    add(pb.maxX,pb.minY); add(pb.minX-b.width(),pb.minY);
    add(pb.minX,pb.maxY); add(pb.minX,pb.minY-b.height());
    add(pb.maxX,pb.maxY); add(pb.maxX-b.width(),pb.maxY);
    add(pb.minX,pb.maxY-b.height()); add(pb.maxX-b.width(),pb.minY);
  }
  if (grid>0) {
    const double xmax=std::min(maxX,minX+grid*40);
    const double ymax=std::min(maxY,minY+grid*40);
    for (double x=minX;x<=xmax+1e-7;x+=grid){add(x,minY);add(x,maxY);}
    for (double y=minY;y<=ymax+1e-7;y+=grid){add(minX,y);add(maxX,y);}
  }
}
static std::vector<Candidate> uniqueCandidates(std::vector<Candidate> c) {
  std::sort(c.begin(),c.end(),[](const auto&a,const auto&b){
    if(a.y!=b.y)return a.y<b.y; if(a.x!=b.x)return a.x<b.x; return a.rotation<b.rotation;
  });
  c.erase(std::unique(c.begin(),c.end(),[](const auto&a,const auto&b){
    return a.rotation==b.rotation&&std::abs(a.x-b.x)<1e-7&&std::abs(a.y-b.y)<1e-7;
  }),c.end());
  return c;
}
static bool tryPlace(const Instance& i,const Sheet& s,const Options& o,std::vector<SheetState>& sheets) {
  std::vector<Candidate> candidates;
  for (size_t si=0;si<sheets.size();++si)
    for (int rot:o.rotations) {
      Shape g=normalized(rotate(i.part.shape,rot));
      addCandidates(candidates,g,s,sheets[si],rot,o.candidateGridMm);
    }
  candidates=uniqueCandidates(std::move(candidates));

  double bestScore=std::numeric_limits<double>::infinity();
  Candidate best{}; size_t bestSheet=sheets.size(); Shape bestShape;
  for (const auto& c:candidates) {
    Shape g=normalized(rotate(i.part.shape,c.rotation));
    for (size_t si=0;si<sheets.size();++si) {
      if (!fitsSheet(g,s,c.x,c.y)) continue;
      Shape placed=translate(g,c.x,c.y);
      if (collides(placed,sheets[si],o.gapMm)) continue;
      const auto b=bounds(placed.outer);
      const double score=b.minY*1e6+b.minX*1e3+polygonArea(placed.outer)*1e-6+si*1e-3;
      if (score<bestScore) {bestScore=score;best={c.x,c.y,c.rotation};bestSheet=si;bestShape=std::move(placed);}
    }
  }
  if (bestSheet==sheets.size()) return false;
  sheets[bestSheet].placements.push_back({i.id,best.x,best.y,best.rotation});
  sheets[bestSheet].geoms.push_back({std::move(bestShape),sheets[bestSheet].placements.back()});
  return true;
}
static bool better(const Result&a,const Result&b) {
  if (a.sheets.size()!=b.sheets.size()) return a.sheets.size()<b.sheets.size();
  if (a.unplaced.size()!=b.unplaced.size()) return a.unplaced.size()<b.unplaced.size();
  if (std::abs(a.utilization-b.utilization)>1e-12) return a.utilization>b.utilization;
  return a.usedAreaMm2>b.usedAreaMm2;
}
}
Result nest(const std::vector<Instance>& parts,const Sheet& s,const Options& o) {
  Result best{}; bool have=false;
  if (parts.empty()) return best;
  std::mt19937_64 rng(o.seed?o.seed:std::random_device{}());
  const size_t restarts=std::max<size_t>(1,std::min<size_t>(o.iterations,256));

  for (size_t pass=0;pass<restarts;++pass) {
    std::vector<size_t> order(parts.size());
    for(size_t i=0;i<parts.size();++i) order[i]=i;
    std::sort(order.begin(),order.end(),[&](size_t a,size_t b){
      const double aa=area(parts[a].part.shape),ab=area(parts[b].part.shape);
      if(std::abs(aa-ab)>1e-9)return aa>ab;
      return parts[a].id<parts[b].id;
    });
    if(pass>0 && order.size()>1) {
      const size_t swaps=std::max<size_t>(1,order.size()/5);
      for(size_t k=0;k<swaps;++k){size_t a=rng()%order.size(),b=rng()%order.size();std::swap(order[a],order[b]);}
    }

    std::vector<SheetState> sheets; std::vector<std::string> unplaced; double used=0;
    for(size_t idx:order) {
      if(tryPlace(parts[idx],s,o,sheets)) {used+=area(parts[idx].part.shape); continue;}
      sheets.emplace_back();
      if(tryPlace(parts[idx],s,o,sheets)) used+=area(parts[idx].part.shape);
      else {sheets.pop_back();unplaced.push_back(parts[idx].id);}
    }

    Result r; r.iterations=pass+1; r.unplaced=std::move(unplaced); r.usedAreaMm2=used;
    r.sheets.reserve(sheets.size());
    for(auto& st:sheets) r.sheets.push_back(std::move(st.placements));
    const double usableW=std::max(0.0,s.width-2*s.marginMm), usableH=std::max(0.0,s.height-2*s.marginMm);
    r.utilization=sheets.empty()?0.0:used/(sheets.size()*usableW*usableH);
    if(!have||better(r,best)){best=std::move(r);have=true;}
    if(best.unplaced.empty()&&best.sheets.size()==1&&best.utilization>0.99) break;
  }
  best.iterations=restarts;
  return best;
}
}
