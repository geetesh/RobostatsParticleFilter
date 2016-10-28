#ifndef PARTICLE_FILTER_
#define PARTICLE_FILTER_

#include "rspf/Particle.h"
#include "rspf/RobotLogReader.h"
#include "rspf/MyMap.h"
#include "rspf/Parameterized.h"
#include "rspf/SensorModel.h"


#include <vector>

#include "rspf/MyResampler.h"

#include "rspf/MyTransitionModel.h"


namespace rspf {

    class ParticleFilter {
    public:

        ParticleFilter(const PropertyTree& ptree );
		~ParticleFilter();
		
		void makeParticleSet();
		void weightParticleSet( SensorData x );
		std::vector<Particle> GetParticles();
		void make_a_transition(); // all the arguments this "function" would need are already in this workspace. 
					 			   // this is a function that is updating things that are already in this workspace
		void handleData( const SensorData& data );
// 
// 		std::vector< std::vector<double> > GetLastRaytraces();
		
    private:

		unsigned int numParticles;
		std::vector<Particle> particleSet;
		
        MyTransitionModel::Ptr myTransitionModel;
		std::vector< std::vector<SensorModel::Ptr> > sensorModels;

        MyResampler my_resampler;

//		WorkerPool workers;
//		Semaphore jobsPending;

        MyMap* wean_map;

        void Initialize( unsigned int numParticles );
		
    }; // class
  
} 

#endif
