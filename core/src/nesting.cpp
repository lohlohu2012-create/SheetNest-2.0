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

static bool fitsSheet(const Shape& shape,const Sheet& s) {
  const auto b=bounds(shape.outer);
  return b.minX>=s.marginMm-1e-9 && b.minY>=s.marginMm-1e-9 &&
         b.maxX<=s.width-s.marginMm+1e-9 && b.maxY<=s.height-s.marginMm+1e-9;
}

static bool collides(const Shape& shape,const SheetState& sheet,double gap) {
  for (const auto& g:sheet.geoms) if (shapesIntersect(shape,g.shape,gap)) return true;
  return false;
}

static void addCandidate(std::vector<Candidate>& c,double x,double y,int rot,const Sheet& s,const Shape& moving) {
  const auto b=bounds(moving.outer);
  const double minX=s.marginMm,minY=s.marginMm;
  const double maxX=s.width-s.marginMm-b.width(),maxY=s.height-s.marginMm-b.height();
  if(x>=minX-1e-7&&x<=maxX+1e-7&&y>=minY-1e-7&&y<=maxY+1e-7)c.push_back({x,y,rot});
}

static Point edgeProjection(Point p,Point a,Point b) {
  const double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
  if(den<1e-12)return a;
  double t=((p.x-a.x)*dx+(p.y-a.y)*dy)/den;
  t=std::clamp(t,0.0,1.0);
  return {a.x+t*dx,a.y+t*dy};
}

static void addContactCandidates(std::vector<Candidate>& c,const Shape& moving,const Sheet& s,
                                 const SheetState& state,int rot,double gap) {
  const auto mb=bounds(moving.outer);
  addCandidate(c,s.marginMm,s.marginMm,rot,s,moving);
  addCandidate(c,s.width-s.marginMm-mb.width(),s.marginMm,rot,s,moving);
  addCandidate(c,s.marginMm,s.height-s.marginMm-mb.height(),rot,s,moving);
  addCandidate(c,s.width-s.marginMm-mb.width(),s.height-s.marginMm-mb.height(),rot,s,moving);

  // Contact candidates approximate a No-Fit-Polygon boundary by aligning
  // moving vertices to stationary vertices/edges. Every candidate is
  // subsequently validated against exact segment geometry and the gap.
  for(const auto& pg:state.geoms) {
    const auto& fixed=pg.shape.outer;
    if(fixed.empty()||moving.outer.empty())continue;

    for(const auto& mp:moving.outer) {
      for(const auto& fp:fixed) {
        addCandidate(c,fp.x-mp.x,fp.y-mp.y,rot,s,moving);
        const double eps=std::max(0.05,gap);
        for(double sx:{-1.0,1.0}) addCandidate(c,fp.x-mp.x+sx*eps,fp.y-mp.y,rot,s,moving);
        for(double sy:{-1.0,1.0}) addCandidate(c,fp.x-mp.x,fp.y-mp.y+sy*eps,rot,s,moving);
      }
      for(size_t j=0;j<fixed.size();++j) {
        const Point a=fixed[j],b=fixed[(j+1)%fixed.size()];
        const Point q=edgeProjection(mp,a,b);
        const double len=std::hypot(b.x-a.x,b.y-a.y);
        if(len<1e-12)continue;
        const Point n{-(b.y-a.y)/len,(b.x-a.x)/len};
        const double off=std::max(0.0,gap);
        for(double sign:{-1.0,1.0}) {
          addCandidate(c,q.x-mp.x+sign*n.x*off,q.y-mp.y+sign*n.y*off,rot,s,moving);
        }
      }
    }
  }

  // Sparse frontier fallback keeps the search bounded for large DXF contours.
  const double grid=5.0;
  const double xmax=s.width-s.marginMm-mb.width(), ymax=s.height-s.marginMm-mb.height();
  const double xend=std::min(xmax,s.marginMm+grid*32.0);
  const double yend=std::min(ymax,s.marginMm+grid*32.0);
  for(double x=s.marginMm;x<=xend+1e-7;x+=grid){addCandidate(c,x,s.marginMm,rot,s,moving);addCandidate(c,x,yend,rot,s,moving);}
  for(double y=s.marginMm;y<=yend+1e-7;y+=grid){addCandidate(c,s.marginMm,y,rot,s,moving);addCandidate(c,xmax,y,rot,s,moving);}
}

static std::vector<Candidate> uniqueCandidates(std::vector<Candidate> c) {
  std::sort(c.begin(),c.end(),[](const auto&a,const auto&b){
    if(a.y!=b.y)return a.y<b.y;
    if(a.x!=b.x)return a.x<b.x;
    return a.rotation<b.rotation;
  });
  c.erase(std::unique(c.begin(),c.end(),[](const auto&a,const auto&b){
    return a.rotation==b.rotation&&std::abs(a.x-b.x)<1e-7&&std::abs(a.y-b.y)<1e-7;
  }),c.end());
  return c;
}

static double contactScore(const Shape& moving,const SheetState& sheet,double gap) {
  if(sheet.geoms.empty())return 0.0;
  double nearest=std::numeric_limits<double>::infinity();
  const auto& mp=moving.outer;
  for(const auto& pg:sheet.geoms) nearest=std::min(nearest,polygonBoundaryDistance(mp,pg.shape.outer));
  return std::isfinite(nearest)?nearest:0.0;
}

static bool tryPlace(const Instance& i,const Sheet& s,const Options& o,std::vector<SheetState>& sheets) {
  double bestScore=std::numeric_limits<double>::infinity();
  Candidate best{}; size_t bestSheet=sheets.size(); Shape bestShape;

  for(size_t si=0;si<sheets.size();++si) {
    for(int rot:o.rotations) {
      Shape g=normalized(rotate(i.part.shape,rot));
      std::vector<Candidate> candidates;
      addContactCandidates(candidates,g,s,sheets[si],rot,o.gapMm);
      candidates=uniqueCandidates(std::move(candidates));

      for(const auto& c:candidates) {
        Shape placed=translate(g,c.x,c.y);
        if(!fitsSheet(placed,s)||collides(placed,sheets[si],o.gapMm))continue;

        const auto b=bounds(placed.outer);
        double usedMaxX=b.maxX,usedMaxY=b.maxY;
        for(const auto& q:sheets[si].geoms){
          const auto qb=bounds(q.shape.outer);
          usedMaxX=std::max(usedMaxX,qb.maxX); usedMaxY=std::max(usedMaxY,qb.maxY);
        }
        const double footprint=usedMaxX*usedMaxY;
        const double touch=contactScore(placed,sheets[si],o.gapMm);

        // Primary score: compact occupied footprint. Secondary: bottom/left.
        const double score=footprint*1e3 + usedMaxY*10.0 + b.minY*0.1 + b.minX*0.01 + touch;
        if(score<bestScore) {
          bestScore=score;
          best={c.x,c.y,c.rotation};
          bestSheet=si;
          bestShape=std::move(placed);
        }
      }
    }
  }

  if(bestSheet==sheets.size())return false;
  sheets[bestSheet].placements.push_back({i.id,best.x,best.y,best.rotation});
  sheets[bestSheet].geoms.push_back({std::move(bestShape),sheets[bestSheet].placements.back()});
  return true;
}

static bool better(const Result&a,const Result&b) {
  if(a.sheets.size()!=b.sheets.size())return a.sheets.size()<b.sheets.size();
  if(a.unplaced.size()!=b.unplaced.size())return a.unplaced.size()<b.unplaced.size();
  if(std::abs(a.utilization-b.utilization)>1e-12)return a.utilization>b.utilization;
  return a.usedAreaMm2>b.usedAreaMm2;
}
}

Result nest(const std::vector<Instance>& parts,const Sheet& s,const Options& o) {
  Result best{}; bool have=false;
  if(parts.empty())return best;

  std::mt19937_64 rng(o.seed?o.seed:std::random_device{}());
  const size_t restarts=std::max<size_t>(1,std::min<size_t>(o.iterations,256));

  for(size_t pass=0;pass<restarts;++pass) {
    std::vector<size_t> order(parts.size());
    for(size_t i=0;i<parts.size();++i)order[i]=i;

    std::sort(order.begin(),order.end(),[&](size_t a,size_t b){
      const double aa=area(parts[a].part.shape),ab=area(parts[b].part.shape);
      if(std::abs(aa-ab)>1e-9)return aa>ab;
      const auto ba=bounds(parts[a].part.shape.outer),bb=bounds(parts[b].part.shape.outer);
      const double pa=ba.width()*ba.height(),pb=bb.width()*bb.height();
      if(std::abs(pa-pb)>1e-9)return pa>pb;
      return parts[a].id<parts[b].id;
    });

    if(pass>0&&order.size()>1) {
      const size_t swaps=std::max<size_t>(1,order.size()/5);
      for(size_t k=0;k<swaps;++k){size_t a=rng()%order.size(),b=rng()%order.size();std::swap(order[a],order[b]);}
    }

    std::vector<SheetState> sheets;
    std::vector<std::string> unplaced;
    double used=0;

    for(size_t idx:order) {
      if(tryPlace(parts[idx],s,o,sheets)) {used+=area(parts[idx].part.shape);continue;}
      sheets.emplace_back();
      if(tryPlace(parts[idx],s,o,sheets))used+=area(parts[idx].part.shape);
      else {sheets.pop_back();unplaced.push_back(parts[idx].id);}
    }

    Result r;
    r.iterations=pass+1;
    r.unplaced=std::move(unplaced);
    r.usedAreaMm2=used;
    r.sheets.reserve(sheets.size());
    for(auto& st:sheets)r.sheets.push_back(std::move(st.placements));

    const double usableW=std::max(0.0,s.width-2*s.marginMm);
    const double usableH=std::max(0.0,s.height-2*s.marginMm);
    r.utilization=sheets.empty()?0.0:used/(sheets.size()*usableW*usableH);

    if(!have||better(r,best)){best=std::move(r);have=true;}
    if(best.unplaced.empty()&&best.sheets.size()==1&&best.utilization>0.99)break;
  }

  best.iterations=restarts;
  return best;
}
}
