#ifndef _LASER_MODEL_H_
#define _LASER_MODEL_H_

#include "rspf/SensorModel.h"
#include "rspf/Distributions.h"
#include "rspf/MyMap.h"

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
    double w_lambda;
    unsigned int laserSubsample;
    double rayThreshold;
    double rayStepSize;
    int rayTotalSteps;
    double max_range;
    normalPDF hitPdf;
    exponentialPDF shortPdf;
    uniformPDF maxPdf;
    uniformPDF randPdf;

    MyMap* wean_map;

    std::vector<double> rayTrace(Particle& particle, const SensorData& data);

public:
    LaserModel(const PropertyTree& ptree );
    virtual void weightParticle( Particle& particle, const SensorData& data );


};

}
#endif
