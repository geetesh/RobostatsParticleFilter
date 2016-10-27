#include "rspf/WallSensorModel.h"

namespace rspf {

	WallSensorModel::WallSensorModel( const MyMap& _map ) :
		map( _map ) {

		xMin = 0;
		yMin = 0;
		xMax = map.RealSize.x - 1E-3;
		yMax = map.RealSize.y - 1E-3;
	}

	void WallSensorModel::weightParticle( Particle& particle, const SensorData& data ) {

		PoseSE2 pose = particle.Pose;
		double x = pose.getX();
		double y = pose.getY();

		if( x < xMin || x > xMax
		 || y < yMin || y > yMax ) {
			particle.Weight = 0;
			return;
		}

		double mapValue = map.GetRealValue( x, y );
		if( std::abs( mapValue + 1.0 ) < 1E-6 ) {
			particle.Weight = 0;
		}
		else {
			particle.Weight =  mapValue * particle.Weight;
		}
	}
	
}
