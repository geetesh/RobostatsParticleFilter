#include "rspf/ParticleFilter.h"

#include "rspf/RandomDistributions.h"
#include <time.h>

#include "rspf/LaserSensorModel.h"
#include "rspf/LaserModel.h"
#include "rspf/WallSensorModel.h"

namespace rspf 
{
    ParticleFilter::ParticleFilter(const MyMap& _map, const PropertyTree& ptree) :
    my_resampler( ptree.get_child("resampler")),
    workers( ptree.get_child("workers") ),
    jobsPending( 0 ),
    map(_map)
    {

        Initialize( ptree.get<unsigned int>("init_particles") );

        unsigned int numWorkers = workers.size();
        myTransitionModel = std::make_shared<MyTransitionModel>(ptree.get_child("transition_model") );

        BOOST_FOREACH( const PropertyTree::value_type& item, ptree ) 
        {
            std::string identifier = item.first;
            PropertyTree subtree = item.second;
            if( identifier == "sensor_model" ) 
            {
                std::string type = subtree.get<std::string>("type");

                std::vector<SensorModel::Ptr> models;
                if( type == "laser" ) 
                {
                    for( unsigned int i = 0; i < numWorkers; i++ ) 
                    {
                         SensorModel::Ptr model = std::make_shared<LaserModel>( map, ptree.get_child("sensor_model") );
                         models.push_back( model );
                    }
                }
                else 
                {
                    throw std::runtime_error( "Unknown sensor model of type: " + type );
                }
                sensorModels.push_back( models );
             }
        }
    }

    ParticleFilter::~ParticleFilter() {

        for( unsigned int i = 0; i < workers.size(); i++ ) {
            jobsPending.Increment();
        }

    }

    void ParticleFilter::Initialize( unsigned int n )
    {
        // UniformDistribution( double lower, double upper );
        UniformDistribution uniformX( 0, map.RealSize.x - 1E-3 );
        UniformDistribution uniformY( 0, map.RealSize.y - 1E-3 );
        UniformDistribution uniformTheta( 0, 2*M_PI );

        clock_t now = clock() >> 8;
        std::cout << "Seeding with " << now << std::endl;
        std::cout << "Map size: " << map.RealSize.x << std::endl;

        uniformX.SetSeed( now );
        uniformY.SetSeed( now+10000 );
        uniformTheta.SetSeed( now+2000 );

        Particles = std::vector<Particle>(n);
        for( unsigned int i = 0; i < n; i++ )
        {
            while(true){

                // draw a pose  	// explicit PoseSE2( double x, double y, double theta );
                PoseSE2 myPose = PoseSE2( uniformX.Sample(), uniformY.Sample(), uniformTheta.Sample() );

                // check if pose is in map		//get value of map at particle x,y	
                //map is -1 for black regions, [0 1] for probability of fill elsewhere (0 = occupied, 1 = empty)
                double mapOccupancy = map.GetRealValue(myPose.getX(), myPose.getY());

                // if x,y not object, we're ok, so break out of the while loop
                if( mapOccupancy > 0.9 ){

                    // draw a particle
                    // assign pose to Particle
                    Particles[i].Pose = myPose;
                    Particles[i].Weight = 1.0F;

                    break;

                }	                                                  //end if
            }                                                   // end while
        }                                                   // end for
    }                                                   //end initialize

    void ParticleFilter::handleData( const SensorData& data )
    {
        // Have to chunk the indices up
        //      double numWorkers = workers.size();
        //		double chunkSize = Particles.size() / numWorkers;
        //		for( unsigned int i = 0; i < numWorkers; i++ ) {
        //			unsigned int startIndex = std::floor( i*chunkSize );
        //			unsigned int endIndex = std::floor( (i+1)*chunkSize ) - 1;
        //			WorkerPool::Job job =
        //				boost::bind( &ParticleFilter::handleDataSubset, this, data, startIndex, endIndex, i );
        //			workers.EnqueueJob( job );
        //		}

        //		for( unsigned int i = 0; i < numWorkers; i++ ) {
        //			jobsPending.Decrement();
        //		}

        //		for( unsigned int i = 0; i < particleSet.size(); i++ ) {
        //			double w = particleSet[i].getW();
        //		}
        unsigned int start = 0;
        unsigned int end = (Particles.size()-1);
        //        std::printf("\n\t\t\tB4 HANDLSUBSET");
        handleDataSubset(data,start,end,0);
        //        std::printf("\n\t\t\tAFTER HANDLSUBSET");

        //        std::printf("\n\t\t\tB4 RES");
        //        particleSet = resampler->resampleParticles( particleSet, numParticles );

        Particles = my_resampler.resampleParticles(Particles);
    //        std::printf("\n\t\t\tAFTER RES");
    }

    // 	std::vector< std::vector<double> > ParticleFilter::GetLastRaytraces() {
    // 		return sensorModels[0][0]->lastTraces;
    // 	}

    // NOTE Concurrent accesses to particleSet are probably OK
    void ParticleFilter::handleDataSubset( const SensorData& data, unsigned int startIndex,
    unsigned int endIndex, unsigned int instanceNum ) {

        // First transition all the particles and reset their weights, apply sensor models
        // 		sensorModels[0][instanceNum]->lastTraces.clear();

        for(unsigned int i = startIndex; i <= endIndex; i++)
        {
            //			transitionModel[instanceNum]->transitionParticle( particleSet[i], data );
            myTransitionModel->transitionParticle(Particles[i]);
            Particles[i].Weight = 1.0F;

            for( unsigned int j = 0; j < sensorModels.size(); j++ ) {
                //                std::cout<<"inloop2";
                sensorModels[j][instanceNum]->weightParticle( Particles[i], data );
            }
        }
    //		jobsPending.Increment();
    }

}
