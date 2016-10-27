#ifndef MY_TRANSITION_MODEL_
#define MY_TRANSITION_MODEL_

#include "rspf/Distributions.h"
#include "rspf/Particle.h"

namespace rspf {

class MyTransitionModel
{
public:
    MyTransitionModel(const rspf::PropertyTree &ptree);
    void transitionParticle( Particle& p);

    typedef std::shared_ptr<MyTransitionModel> Ptr;
private:
    normalPDF x_pdf;
    normalPDF y_pdf;
    normalPDF th_pdf;
};

}

#endif
