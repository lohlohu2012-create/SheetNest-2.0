#include "sheetnest/nfp.hpp"
#include <algorithm>
#include <cmath>

namespace sheetnest {
namespace {
constexpr double EPS=1e-9;

static double cross(Point a,Point b,Point c){
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
static Polygon negate(const Polygon& p){
  Polygon r; r.reserve(p.size());
  for(const auto& q:p)r.push_back({-q.x,-q.y});
  return r;
}
static bool samePoint(Point a,Point b){
  return std::abs(a.x-b.x)<=EPS&&std::abs(a.y-b.y)<=EPS;
}
static Polygon minkowskiByVertexHull(const Polygon& a,const Polygon& b){
  Polygon sums; sums.reserve(a.size()*b.size());
  for(const auto& pa:a)for(const auto& pb:b)sums.push_back({pa.x+pb.x,pa.y+pb.y});
  return convexHull(sums);
}
}
bool isConvex(const Polygon& p){
  if(p.size()<3)return false;
  int sign=0;
  for(size_t i=0;i<p.size();++i){
    const double z=cross(p[i],p[(i+1)%p.size()],p[(i+2)%p.size()]);
    if(std::abs(z)<=EPS)continue;
    const int s=z>0?1:-1;
    if(sign==0)sign=s;
    else if(sign!=s)return false;
  }
  return sign!=0;
}
Polygon convexHull(const Polygon& input){
  if(input.size()<3)return input;
  Polygon p=input;
  std::sort(p.begin(),p.end(),[](const Point&a,const Point&b){
    if(std::abs(a.x-b.x)>EPS)return a.x<b.x;
    return a.y<b.y;
  });
  p.erase(std::unique(p.begin(),p.end(),[](const Point&a,const Point&b){
    return samePoint(a,b);
  }),p.end());
  if(p.size()<3)return p;
  Polygon lower,upper;
  for(const auto& q:p){
    while(lower.size()>=2&&cross(lower[lower.size()-2],lower.back(),q)<=EPS)lower.pop_back();
    lower.push_back(q);
  }
  for(auto it=p.rbegin();it!=p.rend();++it){
    while(upper.size()>=2&&cross(upper[upper.size()-2],upper.back(),*it)<=EPS)upper.pop_back();
    upper.push_back(*it);
  }
  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(),upper.begin(),upper.end());
  return lower;
}
NfpResult buildNfp(const Polygon& fixed,const Polygon& moving){
  if(fixed.size()<3||moving.size()<3)return {};
  const bool exact=isConvex(fixed)&&isConvex(moving);
  const Polygon fixedConv=exact?fixed:convexHull(fixed);
  const Polygon movingConv=exact?moving:convexHull(moving);
  const Polygon boundary=minkowskiByVertexHull(fixedConv,negate(movingConv));
  return {boundary,exact?NfpQuality::ExactConvex:NfpQuality::ConservativeConvexHull,
          boundary.size()>=3};
}
}
