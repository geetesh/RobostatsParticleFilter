#include "rspf/LaserModel.h"

using namespace arma;

namespace rspf {
LaserModel::LaserModel(const rspf::Map &_map, const rspf::PropertyTree &ptree)
{
    w_hit   = ptree.get<double>("gaussian_weight");
    w_short = ptree.get<double>("exponential_weight");
    w_rand  = ptree.get<double>("uniform_weight");
    w_max   = ptree.get<double>("max_range_weight");
    g_var   = ptree.get<double>("gaussian_var");
    e_lambda= ptree.get<double>("exp_lambda");
    laserSubsample = ptree.get<unsigned int>("laser_subsample");
    max_range= ptree.get_child("uniform_component").get<double>("upper_bound");

    hitPdf.SetVariance(g_var);
    shortPdf.SetLambda(e_lambda);
    randPdf.SetBounds(0.0,max_range);
    maxPdf.SetBounds(max_range-1e2,max_range);

    std::printf("\n w_hit \t: %f",w_hit);
    std::printf("\n w_short \t: %f",w_short);
    std::printf("\n w_rand \t: %f",w_rand);
    std::printf("\n w_max \t: %f",w_max);
    std::printf("\n g_var \t: %f",g_var);
    std::printf("\n e_lambda \t: %f",e_lambda);
    std::printf("\n max_range \t: %f",max_range);
}

void LaserModel::weightParticle(Particle &particle, const SensorData &data)
{
    if(!data.hasScan)
    {
//        particle.setW(1.0);
        return;
    }

    std::vector<double> zhat;// = RayTrace( particle, data );
    zhat.resize(data.ScanSize/laserSubsample);

    int laserSubsampledBeams = data.ScanSize/laserSubsample;
    for(size_t i=0;i<laserSubsampledBeams;i++ )
    {

        double trueMeasurement = data.points[i*laserSubsample];
        double estMeasurement = zhat[i];
        hitPdf.SetMean(trueMeasurement);
        shortPdf.SetTrueVal(trueMeasurement);

        rowvec w;
        w << w_hit << w_short << w_rand << w_max;
        w.print("Weight");

        colvec p;
        p << hitPdf.GetProb(estMeasurement) << shortPdf.GetProb(estMeasurement) << randPdf.GetProb(estMeasurement) << maxPdf.GetProb(estMeasurement);
        p.print("Prob:");
        p = log(p);


//        double gp = 1.0/sqrt(2*M_PI*g_var)*exp(-(zhat[i]-data.Scan))
    }

}

}
