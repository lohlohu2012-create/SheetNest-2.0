#include "sheetnest/dxf.hpp"
#include <sstream>
#include <cmath>
#include <unordered_map>
namespace sheetnest {
struct Pair{std::string c,v;};
static std::vector<Pair> pairs(const std::string&s){std::istringstream in(s);std::string c,v;std::vector<Pair>o;while(std::getline(in,c)&&std::getline(in,v))o.push_back({c,v});return o;}
static double d(const std::string&s){try{return std::stod(s);}catch(...){return 0;}}
static Polygon circle(double cx,double cy,double r){Polygon p;for(int i=0;i<64;i++){double a=6.283185307179586*i/64.;p.push_back({cx+r*std::cos(a),cy+r*std::sin(a)});}return p;}
static Polygon arc(double cx,double cy,double r,double a0,double a1,double tol){double span=a1-a0;if(span<0)span+=360;int n=std::max(8,(int)std::ceil(span/(std::max(1.0,tol)*180/(3.141592653589793*r))));Polygon p;for(int i=0;i<=n;i++){double a=(a0+span*i/n)*3.141592653589793/180.;p.push_back({cx+r*std::cos(a),cy+r*std::sin(a)});}return p;}
DxfDocument importDxf(const std::string&text,double tol){DxfDocument out;auto ps=pairs(text);for(size_t i=0;i<ps.size();){if(ps[i].c!="0"){++i;continue;}auto type=ps[i].v;
 if(type=="CIRCLE"){double x=0,y=0,r=0;for(size_t j=i+1;j<ps.size()&&ps[j].c!="0";++j){if(ps[j].c=="10")x=d(ps[j].v);else if(ps[j].c=="20")y=d(ps[j].v);else if(ps[j].c=="40")r=d(ps[j].v);}if(r>0)out.contours.push_back({circle(x,y,r),{},"CIRCLE"});}
 else if(type=="ARC"){double x=0,y=0,r=0,a=0,b=0;for(size_t j=i+1;j<ps.size()&&ps[j].c!="0";++j){if(ps[j].c=="10")x=d(ps[j].v);else if(ps[j].c=="20")y=d(ps[j].v);else if(ps[j].c=="40")r=d(ps[j].v);else if(ps[j].c=="50")a=d(ps[j].v);else if(ps[j].c=="51")b=d(ps[j].v);}if(r>0)out.contours.push_back({arc(x,y,r,a,b,tol),{},"ARC"});}
 else if(type=="LWPOLYLINE"){Polygon p;bool closed=false;double x=0;for(size_t j=i+1;j<ps.size()&&ps[j].c!="0";++j){if(ps[j].c=="10"){x=d(ps[j].v);if(j+1<ps.size()&&ps[j+1].c=="20")p.push_back({x,d(ps[j+1].v)});}else if(ps[j].c=="70")closed=(static_cast<int>(d(ps[j].v))&1)!=0;}if(closed&&p.size()>=3)out.contours.push_back({p,{},"LWPOLYLINE"});}
 else if(type=="LINE"){double x1=0,y1=0,x2=0,y2=0;for(size_t j=i+1;j<ps.size()&&ps[j].c!="0";++j){if(ps[j].c=="10")x1=d(ps[j].v);else if(ps[j].c=="20")y1=d(ps[j].v);else if(ps[j].c=="11")x2=d(ps[j].v);else if(ps[j].c=="21")y2=d(ps[j].v);}Polygon p{{x1,y1},{x2,y2}};out.contours.push_back({p,{},"LINE"});}
 ++i;}return out;}
}
