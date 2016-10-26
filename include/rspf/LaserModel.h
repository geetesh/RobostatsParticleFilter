#include "rspf/SensorModel.h"
#include "rspf/Distributions.h"

namespace rspf {
class LaserModel: public SensorModel
{
private:
    double w_hit;
    double w_short;
    double w_max;
    double w_rand;
    double g_var;
    double e_lambda;
    unsigned int laserSubsample;
    double max_range;
    normalPDF hitPdf;
    exponentialPDF shortPdf;
    uniformPDF maxPdf;
    uniformPDF randPdf;

    double rayStepSize;
    double rayThreshold;

public:
    LaserModel( const Map& _map, const PropertyTree& ptree );
    virtual void weightParticle( Particle& particle, const SensorData& data );
    virtual std::vector<double> rayTrace(Particle& particle, const SensorData& data);

};

}
