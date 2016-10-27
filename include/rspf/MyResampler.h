#ifndef MY_RESAMPLER_
#define MY_RESAMPLER_

#include "rspf/Distributions.h"
#include "rspf/Particle.h"

namespace rspf {

class MyResampler
{
public:
    MyResampler(const rspf::PropertyTree &ptree);
    std::vector<Particle> resampleParticles( const std::vector<Particle>& particles);
private:
    double r;
    double delta;
    normalPDF x_pdf;
    normalPDF y_pdf;
    normalPDF th_pdf;
};

}

#endif
