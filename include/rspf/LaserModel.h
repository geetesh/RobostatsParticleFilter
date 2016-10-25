#include "rspf/SensorModel.h"

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

public:
    LaserModel( const Map& _map, const PropertyTree& ptree );
    virtual void weightParticle( Particle& particle, const SensorData& data );


};

}
