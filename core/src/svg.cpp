#include "sheetnest/svg.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <optional>
#include <regex>
#include <string>
#include <vector>

namespace sheetnest {
namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kEps = 1e-9;
struct Matrix { double a{1},b{0},c{0},d{1},e{0},f{0}; };
struct SvgLoop { Polygon polygon; std::string source; };

Matrix multiply(const Matrix& x,const Matrix& y){
    return {x.a*y.a+x.c*y.b,x.b*y.a+x.d*y.b,
            x.a*y.c+x.c*y.d,x.b*y.c+x.d*y.d,
            x.a*y.e+x.c*y.f+x.e,x.b*y.e+x.d*y.f+x.f};
}
Point apply(const Matrix&m,Point p){return {m.a*p.x+m.c*p.y+m.e,m.b*p.x+m.d*p.y+m.f};}
double signedArea(const Polygon&p){double s=0;for(size_t i=0;i<p.size();++i){auto&a=p[i];auto&b=p[(i+1)%p.size()];s+=a.x*b.y-b.x*a.y;}return s*.5;}
bool finite(Point p){return std::isfinite(p.x)&&std::isfinite(p.y);}
void appendUnique(Polygon&p,Point q,double e=1e-8){if(!finite(q))return;if(p.empty()||std::hypot(p.back().x-q.x,p.back().y-q.y)>e)p.push_back(q);}

std::string attr(const std::string&s,const std::string&name){
    std::smatch m;
    const std::regex q(name+R"(\s*=\s*["']([^"']*)["'])",std::regex::icase);
    if(std::regex_search(s,m,q))return m[1].str();
    const std::regex b(name+R"(\s*=\s*([^\s"'<>]+))",std::regex::icase);
    if(std::regex_search(s,m,b))return m[1].str();
    return {};
}
std::optional<double> number(const std::string&s){
    try{size_t n=0;double v=std::stod(s,&n);if(!n||!std::isfinite(v))return std::nullopt;return v;}catch(...){return std::nullopt;}
}
std::vector<double> numbers(const std::string&s){
    static const std::regex r(R"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)");
    std::vector<double>o;for(auto i=std::sregex_iterator(s.begin(),s.end(),r);i!=std::sregex_iterator();++i){auto v=number(i->str());if(v)o.push_back(*v);}return o;
}
std::vector<std::string> pathTokens(const std::string&s){
    static const std::regex r(R"([A-Za-z]|[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)");
    std::vector<std::string>o;for(auto i=std::sregex_iterator(s.begin(),s.end(),r);i!=std::sregex_iterator();++i)o.push_back(i->str());return o;
}
Matrix parseTransform(const std::string&s,std::vector<DxfDiagnostic>&d){
    Matrix out;static const std::regex r(R"(([A-Za-z]+)\s*\(([^)]*)\))");
    for(auto i=std::sregex_iterator(s.begin(),s.end(),r);i!=std::sregex_iterator();++i){
        std::string n=(*i)[1].str(),u=n;for(char&c:u)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        auto v=numbers((*i)[2].str());Matrix m;
        if(u=="translate"){if(v.empty()||v.size()>2){d.push_back({DxfSeverity::Warning,"SVG/TRANSFORM","Некорректный translate()."});continue;}m.e=v[0];m.f=v.size()==2?v[1]:0;}
        else if(u=="scale"){if(v.empty()||v.size()>2){d.push_back({DxfSeverity::Warning,"SVG/TRANSFORM","Некорректный scale()."});continue;}m.a=v[0];m.d=v.size()==2?v[1]:v[0];}
        else if(u=="rotate"){if(v.size()!=1&&v.size()!=3){d.push_back({DxfSeverity::Warning,"SVG/TRANSFORM","Некорректный rotate()."});continue;}double a=v[0]*kPi/180,cs=std::cos(a),sn=std::sin(a);m={cs,sn,-sn,cs,0,0};if(v.size()==3){m.e=v[1]-cs*v[1]+sn*v[2];m.f=v[2]-sn*v[1]-cs*v[2];}}
        else if(u=="matrix"){if(v.size()!=6){d.push_back({DxfSeverity::Warning,"SVG/TRANSFORM","matrix() требует 6 чисел."});continue;}m={v[0],v[1],v[2],v[3],v[4],v[5]};}
        else {d.push_back({DxfSeverity::Warning,"SVG/TRANSFORM","Неподдерживаемая SVG transform: "+n});continue;}
        out=multiply(out,m);
    }
    return out;
}
Point cubic(Point a,Point b,Point c,Point d,double t){double u=1-t;return {u*u*u*a.x+3*u*u*t*b.x+3*u*t*t*c.x+t*t*t*d.x,u*u*u*a.y+3*u*u*t*b.y+3*u*t*t*c.y+t*t*t*d.y};}
Point quadratic(Point a,Point b,Point c,double t){double u=1-t;return {u*u*a.x+2*u*t*b.x+t*t*c.x,u*u*a.y+2*u*t*b.y+t*t*c.y};}
void appendCubic(Polygon&o,Point a,Point b,Point c,Point d,double tol){double chord=std::hypot(d.x-a.x,d.y-a.y),ctrl=std::max(std::hypot(b.x-a.x,b.y-a.y),std::hypot(c.x-d.x,c.y-d.y));int n=std::clamp((int)std::ceil(std::max(chord,std::max(0.0,ctrl-chord))/std::max(.01,tol)),4,512);for(int i=1;i<=n;++i)appendUnique(o,cubic(a,b,c,d,(double)i/n),tol*.05);}
void appendQuadratic(Polygon&o,Point a,Point b,Point c,double tol){double l=std::max(std::hypot(c.x-a.x,c.y-a.y),std::hypot(b.x-a.x,b.y-a.y));int n=std::clamp((int)std::ceil(l/std::max(.01,tol)),3,384);for(int i=1;i<=n;++i)appendUnique(o,quadratic(a,b,c,(double)i/n),tol*.05);}
void appendArc(Polygon&o,Point s,double rx,double ry,double rot,bool large,bool sweep,Point e,double tol){
    rx=std::abs(rx);ry=std::abs(ry);if(rx<=kEps||ry<=kEps){appendUnique(o,e);return;}
    double ph=rot*kPi/180,cp=std::cos(ph),sp=std::sin(ph),dx=(s.x-e.x)/2,dy=(s.y-e.y)/2,x1=cp*dx+sp*dy,y1=-sp*dx+cp*dy;
    double lam=x1*x1/(rx*rx)+y1*y1/(ry*ry);if(lam>1){double z=std::sqrt(lam);rx*=z;ry*=z;}
    double rx2=rx*rx,ry2=ry*ry,den=rx2*y1*y1+ry2*x1*x1;if(den<=kEps){appendUnique(o,e);return;}
    double coef=(large==sweep?-1:1)*std::sqrt(std::max(0.0,(rx2*ry2-rx2*y1*y1-ry2*x1*x1)/den));
    double cx1=coef*rx*y1/ry,cy1=coef*-ry*x1/rx;
    Point center{cp*cx1-sp*cy1+(s.x+e.x)/2,sp*cx1+cp*cy1+(s.y+e.y)/2};
    auto angle=[](double ux,double uy,double vx,double vy){return std::atan2(ux*vy-uy*vx,std::clamp(ux*vx+uy*vy,-1.0,1.0));};
    double t1=std::atan2((y1-cy1)/ry,(x1-cx1)/rx),delta=angle((x1-cx1)/rx,(y1-cy1)/ry,(-x1-cx1)/rx,(-y1-cy1)/ry);
    if(!sweep&&delta>0)delta-=2*kPi;if(sweep&&delta<0)delta+=2*kPi;
    int n=std::clamp((int)std::ceil(std::abs(delta)*std::max(rx,ry)/std::max(.01,tol)),4,1024);
    for(int i=1;i<=n;++i){double a=t1+delta*i/n;appendUnique(o,{center.x+rx*std::cos(a)*cp-ry*std::sin(a)*sp,center.y+rx*std::cos(a)*sp+ry*std::sin(a)*cp},tol*.05);}
}
bool pair(const std::vector<std::string>&t,size_t&i,double&x,double&y){if(i+2>=t.size())return false;auto a=number(t[i+1]),b=number(t[i+2]);if(!a||!b)return false;x=*a;y=*b;i+=3;return true;}

Polygon parsePath(const std::string&data,double tol,bool&closed,std::vector<DxfDiagnostic>&d){
    auto t=pathTokens(data);if(t.empty())return{};size_t i=0;char cmd=0;Point cur{},start{},lastC{},lastQ{};bool hc=false,hq=false;
    auto iscmd=[](const std::string&s){return s.size()==1&&std::isalpha((unsigned char)s[0]);};
    auto next=[&](double&v){if(i>=t.size()||iscmd(t[i]))return false;auto n=number(t[i++]);if(!n)return false;v=*n;return true;};
    while(i<t.size()){
        if(iscmd(t[i]))cmd=t[i++][0];else if(!cmd){d.push_back({DxfSeverity::Error,"SVG/PATH","Path data начинается без команды."});return{};}
        bool rel=std::islower((unsigned char)cmd),consumed=true;char op=(char)std::toupper((unsigned char)cmd);
        if(op=='Z'){appendUnique(out,start);cur=start;closed=true;hc=hq=false;continue;}
        Polygon dummy;
        switch(op){
        case'M':{double x,y;if(!pair(t,i,x,y))return{};if(rel){x+=cur.x;y+=cur.y;}cur=start={x,y};appendUnique(dummy,cur);/* handled below */}break;
        default:break;
        }
        // Re-run with a compact state machine below; the M branch above only validates the pair.
        break;
    }
    // Full state machine (kept separate so repeated parameter groups remain supported).
    i=0;cmd=0;cur={};start={};lastC={};lastQ={};hc=hq=false;closed=false;Polygon out;
    while(i<t.size()){
        if(iscmd(t[i]))cmd=t[i++][0];else if(!cmd)return{};
        bool rel=std::islower((unsigned char)cmd),ok=true;char op=(char)std::toupper((unsigned char)cmd);
        if(op=='Z'){appendUnique(out,start);cur=start;closed=true;hc=hq=false;continue;}
        auto xy=[&](double&x,double&y){return pair(t,i,x,y);};
        switch(op){
        case'M':{double x,y;if(!xy(x,y))ok=false;else{if(rel){x+=cur.x;y+=cur.y;}cur=start={x,y};appendUnique(out,cur);cmd=rel?'l':'L';hc=hq=false;}break;}
        case'L':{double x,y;if(!xy(x,y))ok=false;else{if(rel){x+=cur.x;y+=cur.y;}cur={x,y};appendUnique(out,cur);hc=hq=false;}break;}
        case'H':{double x;if(!next(x))ok=false;else{if(rel)x+=cur.x;cur.x=x;appendUnique(out,cur);hc=hq=false;}break;}
        case'V':{double y;if(!next(y))ok=false;else{if(rel)y+=cur.y;cur.y=y;appendUnique(out,cur);hc=hq=false;}break;}
        case'C':{if(i+6>t.size())ok=false;else{auto n=[&](size_t k){return number(t[k]);};if(!n(i)||!n(i+1)||!n(i+2)||!n(i+3)||!n(i+4)||!n(i+5))ok=false;else{double x1=*n(i),y1=*n(i+1),x2=*n(i+2),y2=*n(i+3),x=*n(i+4),y=*n(i+5);i+=6;if(rel){x1+=cur.x;y1+=cur.y;x2+=cur.x;y2+=cur.y;x+=cur.x;y+=cur.y;}appendCubic(out,cur,{x1,y1},{x2,y2},{x,y},tol);cur={x,y};lastC={x2,y2};hc=true;hq=false;}}break;}
        case'S':{if(i+4>t.size())ok=false;else{auto n=[&](size_t k){return number(t[k]);};if(!n(i)||!n(i+1)||!n(i+2)||!n(i+3))ok=false;else{double x2=*n(i),y2=*n(i+1),x=*n(i+2),y=*n(i+3);i+=4;if(rel){x2+=cur.x;y2+=cur.y;x+=cur.x;y+=cur.y;}Point c=hc?Point{2*cur.x-lastC.x,2*cur.y-lastC.y}:cur;appendCubic(out,cur,c,{x2,y2},{x,y},tol);cur={x,y};lastC={x2,y2};hc=true;hq=false;}}break;}
        case'Q':{if(i+4>t.size())ok=false;else{auto n=[&](size_t k){return number(t[k]);};if(!n(i)||!n(i+1)||!n(i+2)||!n(i+3))ok=false;else{double x1=*n(i),y1=*n(i+1),x=*n(i+2),y=*n(i+3);i+=4;if(rel){x1+=cur.x;y1+=cur.y;x+=cur.x;y+=cur.y;}appendQuadratic(out,cur,{x1,y1},{x,y},tol);cur={x,y};lastQ={x1,y1};hq=true;hc=false;}}break;}
        case'T':{if(i+2>t.size())ok=false;else{auto n=[&](size_t k){return number(t[k]);};if(!n(i)||!n(i+1))ok=false;else{double x=*n(i),y=*n(i+1);i+=2;if(rel){x+=cur.x;y+=cur.y;}Point c=hq?Point{2*cur.x-lastQ.x,2*cur.y-lastQ.y}:cur;appendQuadratic(out,cur,c,{x,y},tol);cur={x,y};lastQ=c;hq=true;hc=false;}}break;}
        case'A':{if(i+7>t.size())ok=false;else{auto n=[&](size_t k){return number(t[k]);};for(size_t k=i;k<i+7;++k)if(!n(k))ok=false;if(ok){double rx=*n(i),ry=*n(i+1),rot=*n(i+2),la=*n(i+3),sw=*n(i+4),x=*n(i+5),y=*n(i+6);i+=7;if(rel){x+=cur.x;y+=cur.y;}appendArc(out,cur,rx,ry,rot,std::abs(la)>.5,std::abs(sw)>.5,{x,y},tol);cur={x,y};hc=hq=false;}}break;}
        default:d.push_back({DxfSeverity::Error,"SVG/PATH","Неподдерживаемая SVG path-команда: "+std::string(1,op)});return{};
        }
        if(!ok){d.push_back({DxfSeverity::Error,"SVG/PATH","Некорректные параметры path-команды."});return{};}
    }
    if(!closed&&out.size()>=3&&std::hypot(out.front().x-out.back().x,out.front().y-out.back().y)<=tol){out.pop_back();closed=true;}
    return out;
}
Polygon transformPolygon(const Polygon&p,const Matrix&m){Polygon o;o.reserve(p.size());for(auto q:p)appendUnique(o,apply(m,q));if(o.size()>1&&std::hypot(o.front().x-o.back().x,o.front().y-o.back().y)<1e-8)o.pop_back();return o;}
Polygon ellipse(double cx,double cy,double rx,double ry,double tol){int n=std::clamp((int)std::ceil(2*kPi*std::max(rx,ry)/std::max(.01,tol)),16,2048);Polygon p;p.reserve(n);for(int i=0;i<n;++i){double a=2*kPi*i/n;p.push_back({cx+rx*std::cos(a),cy+ry*std::sin(a)});}return p;}
Polygon rect(double x,double y,double w,double h){return{{x,y},{x+w,y},{x+w,y+h},{x,y+h}};}
bool pointIn(Point p,const Polygon&v){if(v.size()<3)return false;bool in=false;for(size_t i=0,j=v.size()-1;i<v.size();j=i++){auto&a=v[i];auto&b=v[j];if((a.y>p.y)!=(b.y>p.y)&&p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y+std::numeric_limits<double>::epsilon())+a.x)in=!in;}return in;}
void classify(std::vector<SvgLoop>l,DxfDocument&doc){
    size_t n=l.size();std::vector<int>depth(n);for(size_t i=0;i<n;++i)for(size_t j=0;j<n;++j)if(i!=j&&std::abs(signedArea(l[j].polygon))>std::abs(signedArea(l[i].polygon))&&pointIn(l[i].polygon.front(),l[j].polygon))++depth[i];
    std::vector<int>oi(n,-1);for(size_t i=0;i<n;++i)if(!(depth[i]&1)){auto p=l[i].polygon;if(signedArea(p)<0)std::reverse(p.begin(),p.end());oi[i]=(int)doc.contours.size();doc.contours.push_back({std::move(p),{},l[i].source,"0"});}
    for(size_t i=0;i<n;++i)if(depth[i]&1){size_t par=n;double best=std::numeric_limits<double>::infinity();for(size_t j=0;j<n;++j)if(!(depth[j]&1)&&pointIn(l[i].polygon.front(),l[j].polygon)){double a=std::abs(signedArea(l[j].polygon));if(a<best){best=a;par=j;}}if(par<n&&oi[par]>=0){auto h=l[i].polygon;if(signedArea(h)>0)std::reverse(h.begin(),h.end());doc.contours[oi[par]].holes.push_back(std::move(h));}}
}

} // namespace

DxfDocument importSvg(const std::string&text,double curveToleranceMm){
    DxfDocument doc;if(text.empty()){doc.diagnostics.push_back({DxfSeverity::Error,"SVG/INPUT","SVG-текст пуст."});return doc;}
    if(text.find("<svg")==std::string::npos&&text.find("<SVG")==std::string::npos){doc.diagnostics.push_back({DxfSeverity::Error,"SVG/ROOT","Корневой элемент <svg> не найден."});return doc;}
    const double tol=std::max(.001,std::abs(curveToleranceMm));
    static const std::regex tag(R"(<\s*(/?)\s*([A-Za-z_][\w:.-]*)([^>]*)>)");
    std::vector<Matrix> ts{Matrix{}};std::vector<bool>vs{true};std::vector<SvgLoop>loops;size_t entity=0;
    for(auto it=std::sregex_iterator(text.begin(),text.end(),tag);it!=std::sregex_iterator();++it){
        bool close=!(*it)[1].str().empty();std::string name=(*it)[2].str(),attrs=(*it)[3].str();std::string lower=name;for(char&c:lower)c=(char)std::tolower((unsigned char)c);
        if(close){if(lower=="g"||lower=="svg"){if(ts.size()>1)ts.pop_back();if(vs.size()>1)vs.pop_back();}continue;}
        bool self=!attrs.empty()&&attrs.back()=='/';Matrix local=parseTransform(attr(attrs,"transform"),doc.diagnostics),world=multiply(ts.back(),local);
        std::string display=attr(attrs,"display"),visibility=attr(attrs,"visibility");bool visible=vs.back()&&display!="none"&&visibility!="hidden";
        if(lower=="g"||lower=="svg"){ts.push_back(world);vs.push_back(visible);if(self){ts.pop_back();vs.pop_back();}continue;}if(!visible)continue;
        ++entity;Polygon p;bool closed=true;std::string id=attr(attrs,"id");
        if(lower=="path")p=parsePath(attr(attrs,"d"),tol,closed,doc.diagnostics);
        else if(lower=="polygon"||lower=="polyline"){auto v=numbers(attr(attrs,"points"));if(v.size()<4||v.size()%2){doc.diagnostics.push_back({DxfSeverity::Error,"SVG/"+lower,"Некорректный points."});continue;}for(size_t i=0;i<v.size();i+=2)p.push_back({v[i],v[i+1]});closed=lower=="polygon";}
        else if(lower=="rect"){auto x=number(attr(attrs,"x")).value_or(0),y=number(attr(attrs,"y")).value_or(0),w=number(attr(attrs,"width")),h=number(attr(attrs,"height"));if(!w||!h||*w<=0||*h<=0){doc.diagnostics.push_back({DxfSeverity::Error,"SVG/rect","Некорректный width/height."});continue;}auto rx=number(attr(attrs,"rx")).value_or(0),ry=number(attr(attrs,"ry")).value_or(rx);if(rx<=kEps&&ry<=kEps)p=rect(x,y,*w,*h);else{double ax=std::min(rx,*w/2),ay=std::min(ry,*h/2);int n=std::clamp((int)std::ceil(kPi*std::max(ax,ay)/(2*std::max(.01,tol))),4,128);for(int i=0;i<=n;++i){double a=-kPi/2+i*kPi/(2*n);appendUnique(p,{x+ax+ax*std::cos(a),y+ay+ay*std::sin(a)});}for(int i=0;i<=n;++i){double a=i*kPi/(2*n);appendUnique(p,{x+*w-ax+ax*std::cos(a),y+ay+ay*std::sin(a)});}for(int i=0;i<=n;++i){double a=kPi/2+i*kPi/(2*n);appendUnique(p,{x+*w-ax+ax*std::cos(a),y+*h-ay+ay*std::sin(a)});}for(int i=0;i<=n;++i){double a=kPi+i*kPi/(2*n);appendUnique(p,{x+ax+ax*std::cos(a),y+*h-ay+ay*std::sin(a)});}}
        }else if(lower=="circle"){auto cx=number(attr(attrs,"cx")),cy=number(attr(attrs,"cy")),r=number(attr(attrs,"r"));if(!cx||!cy||!r||*r<=0)continue;p=ellipse(*cx,*cy,*r,*r,tol);}
        else if(lower=="ellipse"){auto cx=number(attr(attrs,"cx")),cy=number(attr(attrs,"cy")),rx=number(attr(attrs,"rx")),ry=number(attr(attrs,"ry"));if(!cx||!cy||!rx||!ry||*rx<=0||*ry<=0)continue;p=ellipse(*cx,*cy,*rx,*ry,tol);}
        else if(lower=="line"){auto x1=number(attr(attrs,"x1")),y1=number(attr(attrs,"y1")),x2=number(attr(attrs,"x2")),y2=number(attr(attrs,"y2"));if(!x1||!y1||!x2||!y2)continue;p={{*x1,*y1},{*x2,*y2}};closed=false;}
        else continue;
        if(p.size()<2)continue;p=transformPolygon(p,world);++doc.entitiesRead;
        if(closed&&p.size()>=3&&std::abs(signedArea(p))>1e-7){loops.push_back({std::move(p),"SVG#"+std::to_string(entity)+(id.empty()?"":":"+id)});++doc.supportedEntities;}
        else {doc.diagnostics.push_back({DxfSeverity::Warning,"SVG/OPEN","Открытая или вырожденная SVG-геометрия пропущена."});++doc.malformedEntities;}
    }
    classify(std::move(loops),doc);doc.closedLoopsFound=doc.contours.size();
    if(doc.contours.empty())doc.diagnostics.push_back({DxfSeverity::Error,"SVG/GEOMETRY","Не найдено ни одного валидного замкнутого SVG-контура."});
    else doc.diagnostics.push_back({DxfSeverity::Info,"SVG/GEOMETRY","SVG успешно преобразован в замкнутые контуры."});
    return doc;
}
} // namespace sheetnest
