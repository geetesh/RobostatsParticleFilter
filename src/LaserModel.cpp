#include "rspf/LaserModel.h"
#include "opencv2/highgui/highgui.hpp"

using namespace arma;

namespace rspf {

std::vector<double> LaserModel::rayTrace3(Particle& particle, const SensorData& data)
{
    PoseSE2 laserPos = particle.Pose * data.laserOffset;
    double scanAngle = laserPos.getTheta();
    int points = std::floor(data.ScanSize/(double)laserSubsample);

    std::vector<double> zhat(points);

    for(int i=0;i<points;i++)
    {
        double angle = scanAngle + i*data.ScanResolution*laserSubsample - M_PI/2;
        double startX = laserPos.getX(); // in meters
        double startY = laserPos.getY(); // in meters
        double endX = startX + data.MaxRange*std::cos(angle);
        double endY = startY + data.MaxRange*std::sin(angle);


        double prob = 1.0;
        double r = -1.0;

        if(startX<0 || startY<0 || startX>=(wean_map->Size.x/wean_map->Resolution) || startY>=(wean_map->Size.y/wean_map->Resolution) )
        {
            return std::vector<double>();
        }


        for(int j =0;j<rayTotalSteps;j++)
        {
            double alpha = (double)j/(double)rayTotalSteps;
            double xval = alpha*endX + (1-alpha)*startX;
            double yval = alpha*endY + (1-alpha)*startY;
            if(xval<0 || yval<0 || xval>=(wean_map->Size.x/wean_map->Resolution) || yval>=(wean_map->Size.y/wean_map->Resolution))
            {
                r = data.MaxRange;
                break;
            }
            prob *= wean_map->GetMapValue(xval,yval);
            if(prob<=rayThreshold)
            {
                r = j*rayStepSize;
                break;
            }
        }
        if(r < 0.0)
        {
            r = data.MaxRange;
        }
        zhat[i] = r;
    }
    return zhat;
}

LaserModel::LaserModel(const rspf::Map &_map, const rspf::PropertyTree &ptree) :
    map( _map )
{
    w_hit   = ptree.get<double>("gaussian_weight");
    w_short = ptree.get<double>("exponential_weight");
    w_rand  = ptree.get<double>("uniform_weight");
    w_max   = ptree.get<double>("max_range_weight");
    g_var   = ptree.get<double>("gaussian_var");
    e_lambda= ptree.get<double>("exp_lambda");
    laserSubsample = ptree.get<unsigned int>("laser_subsample");
    rayThreshold = ptree.get<double>("raytrace_threshold");
    rayStepSize = ptree.get<double>("raytrace_stepsize");
    max_range= ptree.get_child("uniform_component").get<double>("upper_bound");
    rayTotalSteps = std::floor(SensorData::MaxRange/rayStepSize);

    hitPdf.SetVariance(g_var);
    shortPdf.SetLambda(e_lambda);
    randPdf.SetBounds(0.0,max_range);
    maxPdf.SetBounds(max_range-1e2,max_range);

    wean_map = new MyMap("wean.dat");

    std::printf("\n w_hit \t: %f",w_hit);
    std::printf("\n w_short \t: %f",w_short);
    std::printf("\n w_rand \t: %f",w_rand);
    std::printf("\n w_max \t: %f",w_max);
    std::printf("\n g_var \t: %f",g_var);
    std::printf("\n e_lambda \t: %f",e_lambda);
    std::printf("\n max_range \t: %f",max_range);
    std::printf("\n ray thresh \t: %f",rayThreshold);
    std::printf("\n ray Step \t: %f",rayStepSize);
    std::printf("\n ray Total Steps \t: %f",rayTotalSteps);
    std::printf("\n Wean Map \t: %f",wean_map->Resolution);
}

void LaserModel::weightParticle(Particle &particle, const SensorData &data)
{

    if(!data.hasScan)
    {
        return;
    }

    unsigned int size = data.ScanSize/laserSubsample;
    std::vector<double> zhat = rayTrace3(particle, data);

    if( zhat.empty() ) {
        particle.Weight = 0;
        return;
    }

    rowvec w;
    w << w_hit << w_short << w_rand << w_max;
    w = normalise(w);
    colvec log_prob_scan;
    log_prob_scan.zeros(size);

    for(size_t i=0;i<size;i++ )
    {

        double trueMeasurement = data.points[i*laserSubsample];
        double estMeasurement = zhat[i];
        hitPdf.SetMean(trueMeasurement);
        shortPdf.SetTrueVal(trueMeasurement);

        colvec p;
        p << hitPdf.GetProb(estMeasurement) << shortPdf.GetProb(estMeasurement) << randPdf.GetProb(estMeasurement) << maxPdf.GetProb(estMeasurement);
        double log_prob = log(as_scalar(w*p));
        log_prob_scan(i) = log_prob;
    }

    double joint_prob = sum(log_prob_scan);
    particle.Weight = exp(joint_prob);
}

}
