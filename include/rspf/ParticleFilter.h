#ifndef PARTICLE_FILTER_
#define PARTICLE_FILTER_

#include "rspf/Particle.h"
#include "rspf/RobotLogReader.h"
#include "rspf/Parameterized.h"

#include "rspf/SensorModel.h"

#include "rspf/WorkerPool.h"
#include "rspf/SynchronizationPrimitives.h"

#include <vector>

#include "rspf/MyResampler.h"

#include "rspf/MyTransitionModel.h"
#include "rspf/MyMap.h"


namespace rspf 
{
    class ParticleFilter 
    {
    public:

        ParticleFilter(const MyMap& _map, const PropertyTree& ptree );
        ~ParticleFilter();

        std::vector<Particle> Particles;
        void makeParticleSet();
        void weightParticleSet( SensorData x );
        void make_a_transition(); 
        void handleData( const SensorData& data );
        
    private:

        MyTransitionModel::Ptr myTransitionModel;
        std::vector< std::vector<SensorModel::Ptr> > sensorModels;

        MyResampler my_resampler;

        WorkerPool workers;
        Semaphore jobsPending;

        const MyMap& map;

        void Initialize( unsigned int numParticles );
        void handleDataSubset( const SensorData& data, unsigned int startIndex,
                                unsigned int endIndex, unsigned int instanceNum );
    }; 
} 

#endif
