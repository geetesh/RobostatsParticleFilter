#include "rspf/LaserModel.h"

namespace rspf {
LaserModel::LaserModel(const rspf::Map &_map, const rspf::PropertyTree &ptree)
{
    w_hit   = ptree.get<double>("gaussian_weight");
    w_short = ptree.get<double>("exponential_weight");
    w_rand  = ptree.get<double>("uniform_weight");
    w_max   = ptree.get<double>("max_range_weight");
    g_var   = ptree.get<double>("gaussian_var");
    e_lambda= ptree.get<double>("exp_lambda");
}

void LaserModel::weightParticle(Particle &particle, const SensorData &data)
{

}

}
