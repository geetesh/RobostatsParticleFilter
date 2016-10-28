#include "rspf/ParticleFilter.h"
#include "rspf/Distributions.h"
#include "rspf/LaserModel.h"
#include <boost/foreach.hpp>

namespace rspf {

    ParticleFilter::ParticleFilter(const PropertyTree& ptree ) :
        numParticles( ptree.get<unsigned int>("num_particles") ),
        my_resampler( ptree.get_child("resampler"))
	{
        wean_map = new MyMap("map/wean.dat");
		
		Initialize( ptree.get<unsigned int>("init_particles") );
        myTransitionModel = std::make_shared<MyTransitionModel>(ptree.get_child("transition_model") );

		BOOST_FOREACH( const PropertyTree::value_type& item, ptree ) {

			std::string identifier = item.first;
			PropertyTree subtree = item.second;
			if( identifier == "sensor_model" ) {

				std::string type = subtree.get<std::string>("type");

				std::vector<SensorModel::Ptr> models;
                if( type == "laser" ) {
                        SensorModel::Ptr model = std::make_shared<LaserModel>( ptree.get_child("sensor_model") );
                        models.push_back( model );
                }
				else {
					throw std::runtime_error( "Unknown sensor model of type: " + type );
				}
				sensorModels.push_back( models );
			}
		}
	}

    ParticleFilter::~ParticleFilter() {
	}

	void ParticleFilter::Initialize( unsigned int n )
    {
//        uniformPDF uniformX( 0, map.GetXSize() - 1E-3 );
//        uniformPDF uniformY( 0, map.GetYSize() - 1E-3 );
        uniformPDF uniformX( 0, (wean_map->Size.x/wean_map->Resolution -1) - 1E-3 );
        uniformPDF uniformY( 0, (wean_map->Size.y/wean_map->Resolution -1) - 1E-3 );
        uniformPDF uniformTheta( 0, 2*M_PI );

        std::cout << "\nMap size: " << (wean_map->Size.x/wean_map->Resolution) << std::endl;

		particleSet = std::vector<Particle>(n);
		for( unsigned int i = 0; i < n; i++ )
		{
			while(true){
				
                // draw a pose
				PoseSE2 myPose = PoseSE2( uniformX.Sample(), uniformY.Sample(), uniformTheta.Sample() );
				
				// check if pose is in map		//get value of map at particle x,y	
				//map is -1 for black regions, [0 1] for probability of fill elsewhere (0 = occupied, 1 = empty)
//				double mapOccupancy = map.GetValue( myPose.getX(), myPose.getY() );
                double mapOccupancy = wean_map->GetMapValue(myPose.getX(),myPose.getY());//map.GetValue( myPose.getX(), myPose.getY() );


				// if x,y not object, we're ok, so break out of the while loop
				if( mapOccupancy > 0.9 ){

					// draw a particle
					// assign pose to Particle
						particleSet[i].Pose = myPose;
						particleSet[i].Weight = 1.0F;
						
					break;
					
				}	//end if
			} // end while
		} // end for
	} //end initialize
	
	std::vector<Particle> ParticleFilter::GetParticles()
	{
		return particleSet;
	}

	void ParticleFilter::handleData( const SensorData& data )
    {
        for(int i = 0; i < particleSet.size(); i++)
        {
            myTransitionModel->transitionParticle(particleSet[i],data);
            particleSet[i].Weight = 1.0F;

            for( unsigned int j = 0; j < sensorModels.size(); j++ ) {
                sensorModels[j][0]->weightParticle( particleSet[i], data );
            }
        }
        particleSet = my_resampler.resampleParticles( particleSet);
	}
	
}
