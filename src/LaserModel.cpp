#include "rspf/LaserModel.h"

using namespace arma;

namespace rspf {
std::vector<double> LaserModel::RayTrace(Particle particle, SensorData data, unsigned int size)
{
    double raytraceStepsize  = 0.01;
    double raytraceThreshold = 0.1;
    std::vector<double> zhat(size); // initialize vector of probabilities-per-laser-beam

    PoseSE2 laserH = particle.getPose()*data.laserOffset; // find laser position in world

    double laserAngle = laserH.getTheta(); // get the angle of the laser in the world
    double scanAngle = data.StartAngle + laserAngle;  // get the starting scan angle (relative to the laser angle)

    for( unsigned int s = 0; s < size; s++ ) { //data.ScanSize

        double prob = 1; // this is the starting probability that the laser beam passes through a cell

        double xL = laserH.getX(); // this is the x-starting position of the laser beam
        double yL = laserH.getY(); // this is the y-starting position of the laser beam

        double dX = raytraceStepsize * std::cos( scanAngle );
        double dY = raytraceStepsize * std::sin( scanAngle );
        scanAngle += laserSubsample * SensorData::ScanResolution;
        double r = 0;

        if( xL < 0 || xL >= map.GetXSize() - 1E-3 || yL < 0 || yL >= map.GetYSize() - 1E-3 ) {
            return std::vector<double>() ;
        }

        while( prob > raytraceThreshold ){

            xL += dX; // step along the laser ray
            yL += dY;
            r += raytraceStepsize;

            if( xL < 0 || xL >= map.GetXSize() - 1E-3 || yL < 0 || yL >= map.GetYSize() - 1E-3 ) {
                r = data.MaxRange;
                break;
            }

//                if( xL < 0 ) {
//                    xL = 0;
//                }
//                else if( xL >= map.GetXSize() - 1E-3 ) {
//                    xL = map.GetXSize() - 1;
//                }

//                if( yL < 0 ) {
//                    yL = 0;
//                }
//                else if( yL >= map.GetYSize() - 1E-3 ) {
//                    yL = map.GetYSize() - 1;
//                }

            double mapVal = map.GetValue( xL, yL );
//            std::cout<<"\ngetVal:"<<xL<<","<<yL;

            prob = prob*mapVal; // check the map, & update the probability that the laser hit something

        } // end while-probability-of-ray-hasn't-hit-a-wall

// 			xL = xL-laserH.getX();
// 			yL = yL-laserH.getY();
// 			zhat[s] = std::sqrt( xL*xL + yL*yL ); // store the distance that the laser went on this scan before it hit something
        std::cout<<"\n angle:"<< scanAngle;
        zhat[s] = r;
    } // end for-every-scan

    return zhat;
}

std::vector<double> LaserModel::rayTrace3(Particle& particle, const SensorData& data)
{
    cv::Mat map_decimeter = map.GetMap();
   PoseSE2 laserPos = particle.getPose()*data.laserOffset;
   double scanAngle = laserPos.getTheta();
   int points = std::floor(data.ScanSize/(double)laserSubsample);

   std::vector<double> zhat(points);

   for(int i=0;i<points;i++)
   {
       double angle = scanAngle + i*data.ScanResolution*laserSubsample - M_PI/2;
       double startX = laserPos.getX();
       double startY = laserPos.getY();
       double endX = startX + data.MaxRange*std::cos(angle);
       double endY = startY + data.MaxRange*std::sin(angle);


       double prob = 1.0;
       double r = -1.0;

       if(startX<0 || startY<0 || startX>=map.GetMap().rows || startY>=map.GetMap().cols )
       {
           return std::vector<double>();
       }


       for(int j =0;j<rayTotalSteps;j++)
       {
           double alpha = (double)j/(double)rayTotalSteps;
           double xval = alpha*endX + (1-alpha)*startX;
           double yval = alpha*endY + (1-alpha)*startY;
//           if(startX<0 || startY<0 || startX>map.size.x || startY>map.size.y)
           if(xval<0 || yval<0 || xval>=map.GetXSize()/*.GetMap().rows*/ || yval>=map.GetYSize()/*.GetMap().cols*/)
           {
               r = data.MaxRange;
               break;
           }
//           prob *= map.GetValue((double)xval/**map.GetScale()*/,(double)yval/**map.GetScale()*/);
           xval *=10;
           yval *=10;
           prob *= map_decimeter.at<double>((int)xval,(int)yval);
//           std::cout<<"\ngetVal2:"<<xval<<","<<yval;
//           std::cout<<"\nitpos:"<<it.pos() <<" ,Start: "<<startPoint << "start/end"<<endPoint;

           if(prob<=rayThreshold)
           {

               r = j*rayStepSize;
//               std::cout<<"\n r:"<< prob;
               break;
           }

       }
       if(r < 0.0)
       {
           r = data.MaxRange;
       }
//       std::cout<<"\n angle2:"<< angle;
   zhat[i] = r;
   }

   return zhat;
}

std::vector<double> LaserModel::rayTrace2(Particle& particle, const SensorData& data)
{
   PoseSE2 laserPos = particle.getPose()*data.laserOffset;
   double scanAngle = laserPos.getTheta();
   int points = std::floor(data.ScanSize/(double)laserSubsample);
   cv::Point startPoint(laserPos.getX(),laserPos.getY());
   std::vector<double> zhat(points);

   for(int i=0;i<points;i++)
   {
       double angle = scanAngle + i*data.ScanResolution*laserSubsample - M_PI/2;
       double startX = laserPos.getX();
       double startY = laserPos.getY();
       double endX = startX + data.MaxRange*map.GetScale()*std::cos(angle);
       double endY = startY + data.MaxRange*map.GetScale()*std::sin(angle);
       cv::Point endPoint(endX,endY);


//       cv::LineIterator it(map.GetMap(),startPoint,endPoint,8);

       cv::Mat I;
       I.zeros(30,30,CV_8U);
       cv::LineIterator it(I,cv::Point(0,0),cv::Point(10,10),8);
       std::cout<<"\nITCOUNT:"<<it.count;
       for(int k=0;k<it.count;k++)
       {
           std::cout<<"\nitpos:"<<it.pos();
           it++;
       }


       double prob = 1;
       double r = -1;
//     if(startX<0 || startY<0 || startX>map.size.x || startY>map.size.y)
       if(startX<0 || startY<0 || startX>=map.GetMap().rows || startY>=map.GetMap().cols )
       {
           return std::vector<double>();
       }


       for(int j = 0; j<it.count;j++,++it)
       {
           int xval = it.pos().x;
           int yval = it.pos().y;
//           if(startX<0 || startY<0 || startX>map.size.x || startY>map.size.y)
           if(xval<0 || yval<0 || xval>=map.GetMap().rows || yval>=map.GetMap().cols)
           {
               r = data.MaxRange;
               break;
           }
           xval *= map.GetScale();
           yval *= map.GetScale();
//           prob *= img.at(it.pos());
           prob = map.GetValue((double)xval,(double)yval);
           std::cout<<"\nitpos:"<<it.pos() <<" ,Start: "<<startPoint << "start/end"<<endPoint;
           if(prob<rayThreshold)
           {

               r = cv::norm(it.pos()-startPoint) * map.GetScale();
               std::cout<<"\n r:"<< r;
//               return std::vector<double>();
               break;
           }
       }
       if(r == -1)
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
//        particle.setW(1.0);
        return;
    }

    unsigned int size = data.ScanSize/laserSubsample;
//    std::vector<double> zhat = RayTrace( particle, data , size);
//    std::cout<<"\nZhat1:\n";
//    for(int i=0;i<zhat.size();i++)
//        std::cout<<" "<<zhat[i];
    std::vector<double> zhat = rayTrace3(particle, data);
//    std::cout<<"\nZhat2:\n";
//    for(int i=0;i<zhat2.size();i++)
//        std::cout<<" "<<zhat2[i];
//    zhat.resize(size);
//    zhat.assign(size,10.0);

    if( zhat.empty() ) {
        particle.setW( 0 );
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


//        w.print("Weight");

        colvec p;
        p << hitPdf.GetProb(estMeasurement) << shortPdf.GetProb(estMeasurement) << randPdf.GetProb(estMeasurement) << maxPdf.GetProb(estMeasurement);
//        p.print("Prob:");
        double log_prob = log(as_scalar(w*p));
        log_prob_scan(i) = log_prob;
    }
//    log_prob_scan.print("LogProbScan:");

    double joint_prob = sum(log_prob_scan);
    particle.setW(exp(joint_prob));
//    particle.setW(1.0);
}

}
