#include "rspf/MyTransitionModel.h"

namespace rspf
{

MyTransitionModel::MyTransitionModel(const rspf::PropertyTree &ptree):
    x_pdf(ptree.get_child("x_noise")),
    y_pdf(ptree.get_child("y_noise")),
    th_pdf(ptree.get_child("th_noise"))
{
    std::printf("\n x_noise \t: %f",x_pdf.GetVariance());
    std::printf("\n y_noise \t: %f",y_pdf.GetVariance());
    std::printf("\n th_noise \t: %f",th_pdf.GetVariance());
}

void MyTransitionModel::transitionParticle(Particle& p, const SensorData &data)
{
//    PoseSE2 pose = p.Pose;
    PoseSE2 noise(x_pdf.Sample(),y_pdf.Sample(),th_pdf.Sample());
    PoseSE2 corruptedPose = p.Pose * noise * data.displacement;
//    pose = pose * noise;
    p.Pose = corruptedPose;
}
}
