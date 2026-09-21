#include "sheetnest/cutting.hpp"
#include <algorithm>
namespace sheetnest {
struct Row{Material m;double t;double speed;const char* gas;};
static const Row rows[]={
 {Material::CarbonSteel,1,9,"O2"},{Material::CarbonSteel,2,6.5,"O2"},{Material::CarbonSteel,3,3.5,"O2"},{Material::CarbonSteel,4,3.2,"O2"},{Material::CarbonSteel,5,2.75,"O2"},{Material::CarbonSteel,6,2.25,"O2"},{Material::CarbonSteel,8,1.9,"O2"},{Material::CarbonSteel,10,1.3,"O2"},{Material::CarbonSteel,12,1.2,"O2"},{Material::CarbonSteel,16,.8,"O2"},{Material::CarbonSteel,20,.7,"O2"},
 {Material::StainlessSteel,1,30,"N2"},{Material::StainlessSteel,2,20,"N2"},{Material::StainlessSteel,3,11,"N2"},{Material::StainlessSteel,4,8,"N2"},{Material::StainlessSteel,5,7.5,"N2"},{Material::StainlessSteel,6,5,"N2"},{Material::StainlessSteel,8,3.5,"N2"},{Material::StainlessSteel,10,2,"N2"},{Material::StainlessSteel,12,1.5,"N2"},
 {Material::Aluminum,1,24,"N2"},{Material::Aluminum,2,20,"N2"},{Material::Aluminum,3,7.5,"N2"},{Material::Aluminum,4,5,"N2"},{Material::Aluminum,5,4.5,"N2"},{Material::Aluminum,6,3.5,"N2"},{Material::Aluminum,8,2.5,"N2"},
 {Material::Brass,1,25,"N2"},{Material::Brass,2,15,"N2"},{Material::Brass,3,6.5,"N2"},{Material::Brass,4,5,"N2"},{Material::Brass,5,4,"N2"},{Material::Brass,6,3,"N2"},{Material::Brass,8,2.5,"N2"}
};
CuttingParameters bodor3kWParameters(Material m,double t){CuttingParameters p{m,t,0,""};const Row*best=nullptr;double d=1e9;for(auto&r:rows)if(r.m==m&&std::abs(r.t-t)<d){best=&r;d=std::abs(r.t-t);}if(best){p.speedMMin=best->speed;p.assistGas=best->gas;}return p;}
CuttingEstimate estimateCutting(const std::vector<CutContour>& c,const CuttingParameters&p,int pierces,double rapid,double pierceSeconds){CuttingEstimate e;e.parameters=p;e.pierces=pierces;e.rapidMinutes=p.speedMMin>0?rapid/(p.speedMMin*1000.0):0;for(auto&x:c)e.contourLengthMm+=x.lengthMm;e.cuttingMinutes=p.speedMMin>0?e.contourLengthMm/(p.speedMMin*1000.0):0;e.piercingMinutes=pierces*pierceSeconds/60.0;e.totalMinutes=e.cuttingMinutes+e.piercingMinutes+e.rapidMinutes;return e;}
}
