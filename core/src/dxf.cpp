#include "sheetnest/dxf.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace sheetnest {
namespace {
struct Pair{int code{};std::string value;};
struct Path{Polygon points;std::string type,id;bool closed{};};
constexpr double PI=3.14159265358979323846;
static double num(const std::string&s){try{return std::stod(s);}catch(...){return 0;}}
static int integer(const std::string&s){try{return std::stoi(s);}catch(...){return 0;}}
static std::vector<Pair> readPairs(const std::string& text){
  std::istringstream in(text); std::string a,b; std::vector<Pair> out;
  while(std::getline(in,a)&&std::getline(in,b)){
    if(!a.empty()&&a.back()=='\r')a.pop_back(); if(!b.empty()&&b.back()=='\r')b.pop_back();
    try{out.push_back({std::stoi(a),b});}catch(...){}
  }
  return out;
}
static bool near(Point a,Point b,double e=1e-7){return std::hypot(a.x-b.x,a.y-b.y)<=e;}
static Polygon arcPoints(Point a,Point b,double bulge,double tol){
  if(std::abs(bulge)<1e-12)return {a,b};
  const double chord=std::hypot(b.x-a.x,b.y-a.y); if(chord<1e-12)return {a};
  const double theta=4*std::atan(bulge), half=theta/2.0;
  const double radius=chord/(2*std::sin(std::abs(half)));
  const Point mid{(a.x+b.x)/2,(a.y+b.y)/2}, u{(b.x-a.x)/chord,(b.y-a.y)/chord}, n{-u.y,u.x};
  const double offset=chord/(2*std::tan(half));
  const Point center{mid.x+n.x*offset,mid.y+n.y*offset};
  const double start=std::atan2(a.y-center.y,a.x-center.x), r=std::abs(radius), span=std::abs(theta);
  double maxStep=span;
  if(tol>0&&r>tol){
    maxStep=2*std::acos(std::clamp(1.0-tol/r,-1.0,1.0));
    if(maxStep<1e-3)maxStep=1e-3;
  }
  int count=std::max(1,(int)std::ceil(span/maxStep)); Polygon p; p.reserve(count+1);
  for(int i=0;i<=count;++i){double t=(double)i/count,ang=start+theta*t;p.push_back({center.x+r*std::cos(ang),center.y+r*std::sin(ang)});}
  return p;
}
static Polygon circlePoints(double cx,double cy,double r,double tol){
  int n=64;
  if(tol>0&&r>tol){double step=2*std::acos(std::clamp(1.0-tol/r,-1.0,1.0));if(step>1e-6)n=std::clamp((int)std::ceil(2*PI/step),32,720);}
  Polygon p; p.reserve(n); for(int i=0;i<n;++i){double a=2*PI*i/n;p.push_back({cx+r*std::cos(a),cy+r*std::sin(a)});} return p;
}
static void appendArc(Polygon& out,Point a,Point b,double bulge,double tol){
  auto p=arcPoints(a,b,bulge,tol); if(out.empty())out=p; else out.insert(out.end(),p.begin()+1,p.end());
}
static void connectPath(std::vector<Path>& paths,Path p){
  if(p.points.size()<2)return;
  if(p.closed){paths.push_back(std::move(p));return;}
  for(auto& cur:paths){
    if(cur.closed)continue;
    if(near(cur.points.back(),p.points.front())){cur.points.insert(cur.points.end(),p.points.begin()+1,p.points.end());return;}
    if(near(cur.points.back(),p.points.back())){std::reverse(p.points.begin(),p.points.end());cur.points.insert(cur.points.end(),p.points.begin()+1,p.points.end());return;}
    if(near(cur.points.front(),p.points.back())){cur.points.insert(cur.points.begin(),p.points.begin(),p.points.end()-1);return;}
    if(near(cur.points.front(),p.points.front())){std::reverse(p.points.begin(),p.points.end());cur.points.insert(cur.points.begin(),p.points.begin(),p.points.end()-1);return;}
  }
  paths.push_back(std::move(p));
}
}
DxfDocument importDxf(const std::string& text,double tol){
  DxfDocument doc; auto pairs=readPairs(text); std::vector<Path> paths;
  for(size_t i=0;i<pairs.size();){
    if(pairs[i].code!=0){++i;continue;}
    const std::string type=pairs[i].value; size_t j=i+1;
    while(j<pairs.size()&&pairs[j].code!=0)++j;
    const size_t begin=i,end=j; std::string id;
    for(size_t k=begin;k<end;++k)if(pairs[k].code==5){id=pairs[k].value;break;}
    ++doc.entityCount;

    if(type=="LINE"){
      Point a{},b{};
      for(size_t k=begin+1;k<end;++k){if(pairs[k].code==10)a.x=num(pairs[k].value);else if(pairs[k].code==20)a.y=num(pairs[k].value);else if(pairs[k].code==11)b.x=num(pairs[k].value);else if(pairs[k].code==21)b.y=num(pairs[k].value);}
      connectPath(paths,{Polygon{a,b},type,id,false});
    } else if(type=="ARC"){
      Point c{};double r=0,a0=0,a1=0;
      for(size_t k=begin+1;k<end;++k){switch(pairs[k].code){case 10:c.x=num(pairs[k].value);break;case 20:c.y=num(pairs[k].value);break;case 40:r=num(pairs[k].value);break;case 50:a0=num(pairs[k].value);break;case 51:a1=num(pairs[k].value);break;}}
      if(r>0){if(a1<=a0)a1+=360;double step=2*std::acos(std::clamp(1.0-tol/r,-1.0,1.0));int n=std::max(2,(int)std::ceil((a1-a0)*PI/180.0/std::max(step,1e-3)));Polygon p;p.reserve(n+1);for(int q=0;q<=n;++q){double a=(a0+(a1-a0)*q/n)*PI/180.0;p.push_back({c.x+r*std::cos(a),c.y+r*std::sin(a)});}connectPath(paths,{p,type,id,false});}
    } else if(type=="CIRCLE"){
      Point c{};double r=0;for(size_t k=begin+1;k<end;++k){if(pairs[k].code==10)c.x=num(pairs[k].value);else if(pairs[k].code==20)c.y=num(pairs[k].value);else if(pairs[k].code==40)r=num(pairs[k].value);}if(r>0)paths.push_back({circlePoints(c.x,c.y,r,tol),type,id,true});
    } else if(type=="LWPOLYLINE"){
      struct V{Point p;double bulge{};}; std::vector<V> v; bool closed=false;
      for(size_t k=begin+1;k<end;){if(pairs[k].code==10){V x;x.p.x=num(pairs[k].value);if(k+1<end&&pairs[k+1].code==20){x.p.y=num(pairs[k+1].value);k+=2;}else ++k;if(k<end&&pairs[k].code==42){x.bulge=num(pairs[k].value);++k;}v.push_back(x);}else if(pairs[k].code==70){closed=(integer(pairs[k].value)&1)!=0;++k;}else ++k;}
      if(v.size()>=2){Polygon p;for(size_t q=0;q+1<v.size();++q)appendArc(p,v[q].p,v[q+1].p,v[q].bulge,tol);if(closed)appendArc(p,v.back().p,v.front().p,v.back().bulge,tol);connectPath(paths,{p,type,id,closed});}
    } else if(type=="POLYLINE"){
      struct V{Point p;double bulge{};}; std::vector<V> v; bool closed=false; size_t k=i+1;
      while(k<pairs.size()){if(pairs[k].code==0&&pairs[k].value=="SEQEND"){++k;break;}if(pairs[k].code==0&&pairs[k].value=="VERTEX"){++k;V x{};bool have=false;while(k<pairs.size()&&pairs[k].code!=0){if(pairs[k].code==10){x.p.x=num(pairs[k].value);have=true;}else if(pairs[k].code==20)x.p.y=num(pairs[k].value);else if(pairs[k].code==42)x.bulge=num(pairs[k].value);++k;}if(have)v.push_back(x);continue;}if(pairs[k].code==70&&v.empty())closed=(integer(pairs[k].value)&1)!=0;++k;}
      j=k;if(v.size()>=2){Polygon p;for(size_t q=0;q+1<v.size();++q)appendArc(p,v[q].p,v[q+1].p,v[q].bulge,tol);if(closed)appendArc(p,v.back().p,v.front().p,v.back().bulge,tol);connectPath(paths,{p,type,id,closed});}
    }
    i=j;
  }

  std::vector<Polygon> closed;
  for(auto& path:paths){
    if(path.points.size()<3){doc.diagnostics.push_back({path.type,path.id,"Открытый или вырожденный контур: менее 3 точек.",false});continue;}
    if(near(path.points.front(),path.points.back()))path.points.pop_back();
    if(path.points.size()<3||std::abs(signedPolygonArea(path.points))<1e-9){doc.diagnostics.push_back({path.type,path.id,"Контур имеет нулевую/недостаточную площадь.",false});continue;}
    closed.push_back(std::move(path.points)); ++doc.closedPathCount;
  }
  for(size_t i=0;i<closed.size();++i){
    const auto& p=closed[i]; int depth=0;
    for(size_t j=0;j<closed.size();++j)if(i!=j&&pointInPolygon(p[0],closed[j]))++depth;
    if(depth%2)continue;
    DxfContour c; c.outer=p; c.sourceId=std::to_string(i);
    for(size_t j=0;j<closed.size();++j){
      if(i==j)continue;
      int d=0;for(size_t k=0;k<closed.size();++k)if(k!=j&&pointInPolygon(closed[j][0],closed[k]))++d;
      if(d==depth+1&&pointInPolygon(closed[j][0],p))c.holes.push_back(closed[j]);
    }
    doc.contours.push_back(std::move(c));
  }
  if(doc.contours.empty()&&!paths.empty())doc.diagnostics.push_back({"DXF","","Не найдено ни одного замкнутого валидного контура.",false});
  return doc;
}
DxfDocument importDxfFile(const std::string& path,double tol){
  std::ifstream f(path,std::ios::binary); if(!f){DxfDocument d;d.diagnostics.push_back({"FILE",path,"Не удалось открыть DXF-файл.",false});return d;}
  std::ostringstream ss;ss<<f.rdbuf();return importDxf(ss.str(),tol);
}
}
