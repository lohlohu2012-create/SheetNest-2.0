#include "sheetnest/geometry.hpp"
#include <algorithm>
#include <cmath>
namespace sheetnest {
Bounds bounds(const Polygon&p){Bounds b{};if(p.empty())return b;b.minX=b.maxX=p[0].x;b.minY=b.maxY=p[0].y;for(auto&q:p){b.minX=std::min(b.minX,q.x);b.maxX=std::max(b.maxX,q.x);b.minY=std::min(b.minY,q.y);b.maxY=std::max(b.maxY,q.y);}return b;}
double polygonArea(const Polygon&p){double a=0;for(size_t i=0;i<p.size();++i){auto&a=p[i];auto&b=p[(i+1)%p.size()];a+=a.x*b.y-b.x*a.y;}return std::abs(a)*.5;}
bool pointInPolygon(const Point&p,const Polygon&g){if(g.size()<3)return false;bool in=false;for(size_t i=0,j=g.size()-1;i<g.size();j=i++){auto a=g[i],b=g[j];if(((a.y>p.y)!=(b.y>p.y))&&(p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y+1e-15)+a.x))in=!in;}return in;}
static bool onSeg(Point p,Point a,Point b){double c=(b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);return std::abs(c)<1e-9&&p.x>=std::min(a.x,b.x)-1e-9&&p.x<=std::max(a.x,b.x)+1e-9&&p.y>=std::min(a.y,b.y)-1e-9&&p.y<=std::max(a.y,b.y)+1e-9;}
static int ori(Point a,Point b,Point c){double v=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);return(v>1e-9)-(v<-1e-9);}
static bool hit(Point a,Point b,Point c,Point d){int a1=ori(a,b,c),a2=ori(a,b,d),b1=ori(c,d,a),b2=ori(c,d,b);if(a1!=a2&&b1!=b2)return true;return(a1==0&&onSeg(c,a,b))||(a2==0&&onSeg(d,a,b))||(b1==0&&onSeg(a,c,d))||(b2==0&&onSeg(b,c,d));}
bool polygonsIntersect(const Polygon&a,const Polygon&b){if(a.empty()||b.empty())return false;for(size_t i=0;i<a.size();++i)for(size_t j=0;j<b.size();++j)if(hit(a[i],a[(i+1)%a.size()],b[j],b[(j+1)%b.size()]))return true;return pointInPolygon(a[0],b)||pointInPolygon(b[0],a);}
Polygon rotate(const Polygon&p,int deg){double r=deg*3.141592653589793/180.,c=std::cos(r),s=std::sin(r);Polygon o;for(auto q:p)o.push_back({q.x*c-q.y*s,q.x*s+q.y*c});return o;}
Polygon translate(const Polygon&p,double dx,double dy){auto o=p;for(auto&q:o){q.x+=dx;q.y+=dy;}return o;}
}
