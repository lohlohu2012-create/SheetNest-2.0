#include "sheetnest/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace sheetnest {
namespace {
constexpr double EPS = 1e-9;
double cross(Point a, Point b, Point c) {
  return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
}
}
Bounds bounds(const Polygon& p) {
  Bounds b{};
  if (p.empty()) return b;
  b.minX=b.maxX=p[0].x; b.minY=b.maxY=p[0].y;
  for (auto q:p) {
    b.minX=std::min(b.minX,q.x); b.maxX=std::max(b.maxX,q.x);
    b.minY=std::min(b.minY,q.y); b.maxY=std::max(b.maxY,q.y);
  }
  return b;
}
double signedPolygonArea(const Polygon& p) {
  double a=0;
  for (size_t i=0;i<p.size();++i) {
    const auto& a0=p[i]; const auto& b=p[(i+1)%p.size()];
    a += a0.x*b.y - b.x*a0.y;
  }
  return 0.5*a;
}
double polygonArea(const Polygon& p) { return std::abs(signedPolygonArea(p)); }
bool pointOnSegment(const Point& p,const Point& a,const Point& b,double eps) {
  if (std::abs(cross(a,b,p))>eps) return false;
  return p.x>=std::min(a.x,b.x)-eps && p.x<=std::max(a.x,b.x)+eps &&
         p.y>=std::min(a.y,b.y)-eps && p.y<=std::max(a.y,b.y)+eps;
}
bool pointInPolygon(const Point& p,const Polygon& g) {
  if (g.size()<3) return false;
  bool in=false;
  for (size_t i=0,j=g.size()-1;i<g.size();j=i++) {
    const auto a=g[i], b=g[j];
    if (pointOnSegment(p,a,b,EPS)) return true;
    if (((a.y>p.y)!=(b.y>p.y)) &&
        p.x < (b.x-a.x)*(p.y-a.y)/(b.y-a.y) + a.x) in=!in;
  }
  return in;
}
bool pointInShape(const Point& p,const Shape& s) {
  if (!pointInPolygon(p,s.outer)) return false;
  for (const auto& h:s.holes) if (pointInPolygon(p,h)) return false;
  return true;
}
bool segmentsIntersect(const Point& a,const Point& b,const Point& c,const Point& d,double eps) {
  const double c1=cross(a,b,c), c2=cross(a,b,d), c3=cross(c,d,a), c4=cross(c,d,b);
  if (((c1>eps&&c2<-eps)||(c1<-eps&&c2>eps)) &&
      ((c3>eps&&c4<-eps)||(c3<-eps&&c4>eps))) return true;
  return pointOnSegment(c,a,b,eps)||pointOnSegment(d,a,b,eps)||
         pointOnSegment(a,c,d,eps)||pointOnSegment(b,c,d,eps);
}
static double pointSegmentDistance(Point p,Point a,Point b) {
  const double vx=b.x-a.x, vy=b.y-a.y, wx=p.x-a.x, wy=p.y-a.y, vv=vx*vx+vy*vy;
  double t = vv>EPS ? (wx*vx+wy*vy)/vv : 0.0;
  t=std::clamp(t,0.0,1.0);
  return std::hypot(p.x-(a.x+t*vx),p.y-(a.y+t*vy));
}
double segmentDistance(const Point& a,const Point& b,const Point& c,const Point& d) {
  if (segmentsIntersect(a,b,c,d)) return 0.0;
  return std::min({pointSegmentDistance(a,c,d),pointSegmentDistance(b,c,d),
                   pointSegmentDistance(c,a,b),pointSegmentDistance(d,a,b)});
}
double polygonBoundaryDistance(const Polygon& a,const Polygon& b) {
  if (a.empty()||b.empty()) return std::numeric_limits<double>::infinity();
  double best=std::numeric_limits<double>::infinity();
  for (size_t i=0;i<a.size();++i) for (size_t j=0;j<b.size();++j) {
    best=std::min(best,segmentDistance(a[i],a[(i+1)%a.size()],b[j],b[(j+1)%b.size()]));
    if (best==0) return 0;
  }
  return best;
}
bool polygonsIntersect(const Polygon& a,const Polygon& b) {
  if (a.size()<3||b.size()<3) return false;
  const auto ba=bounds(a), bb=bounds(b);
  if (ba.maxX<bb.minX-EPS||bb.maxX<ba.minX-EPS||
      ba.maxY<bb.minY-EPS||bb.maxY<ba.minY-EPS) return false;
  for (size_t i=0;i<a.size();++i) for (size_t j=0;j<b.size();++j)
    if (segmentsIntersect(a[i],a[(i+1)%a.size()],b[j],b[(j+1)%b.size()])) return true;
  return pointInPolygon(a[0],b)||pointInPolygon(b[0],a);
}
bool shapesIntersect(const Shape& a,const Shape& b,double gap) {
  if (a.outer.size()<3||b.outer.size()<3) return false;
  const std::vector<const Polygon*> ar = [&]{std::vector<const Polygon*> r{&a.outer}; for (auto& h:a.holes) r.push_back(&h); return r;}();
  const std::vector<const Polygon*> br = [&]{std::vector<const Polygon*> r{&b.outer}; for (auto& h:b.holes) r.push_back(&h); return r;}();
  for (auto pa:ar) for (auto pb:br) {
    if (polygonsIntersect(*pa,*pb)) {
      if (pa==&a.outer && pb!=&b.outer && pointInPolygon(a.outer[0],*pb)) continue;
      if (pb==&b.outer && pa!=&a.outer && pointInPolygon(b.outer[0],*pa)) continue;
      return true;
    }
    if (gap>0 && polygonBoundaryDistance(*pa,*pb)<gap) return true;
  }
  return pointInShape(a.outer[0],b)||pointInShape(b.outer[0],a);
}
Polygon rotate(const Polygon& p,double r) {
  const double c=std::cos(r), s=std::sin(r); Polygon q; q.reserve(p.size());
  for (auto a:p) q.push_back({a.x*c-a.y*s,a.x*s+a.y*c});
  return q;
}
Polygon rotate(const Polygon& p,int d) { return rotate(p,d*3.14159265358979323846/180.0); }
Polygon translate(const Polygon& p,double x,double y) { auto q=p; for (auto& a:q){a.x+=x;a.y+=y;} return q; }
Shape rotate(const Shape& s,int d) {
  Shape q{rotate(s.outer,d),{}}; for (const auto& h:s.holes) q.holes.push_back(rotate(h,d)); return q;
}
Shape translate(const Shape& s,double x,double y) {
  Shape q{translate(s.outer,x,y),{}}; for (const auto& h:s.holes) q.holes.push_back(translate(h,x,y)); return q;
}
Polygon normalized(const Polygon& p) { auto b=bounds(p); return translate(p,-b.minX,-b.minY); }
Shape normalized(const Shape& s) {
  Shape q{s.outer,{}}; if (q.outer.empty()) return q;
  auto b=bounds(q.outer); q.outer=translate(q.outer,-b.minX,-b.minY);
  for (const auto& h:s.holes) q.holes.push_back(translate(h,-b.minX,-b.minY));
  return q;
}
}
