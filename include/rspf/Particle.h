#ifndef PARTICLE_
#define PARTICLE_

#include "rspf/PoseSE2.h"

namespace rspf 
{
    class Particle 
    {
    public:
        Particle();
        PoseSE2 Pose;
        double Weight;
    };
}

#endif 
