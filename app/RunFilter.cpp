#include "rspf/MyMap.h"
#include "rspf/RobotLogReader.h"
#include "rspf/FilterVisualizer.h"
#include "rspf/Parameterized.h"

#include <iostream>

using namespace rspf;

int main( int argc, char* argv[] ) {

    if( argc < 2 ) {
        std::cout << "Please specify config file." << std::endl;
        return -1;
    }

    std::string configFilename( argv[1] );
	PropertyTree ptree = read_property_xml( configFilename );

	std::cout << "Initializing map..." << std::endl;
    MyMap map( "map/wean.dat" );

	std::cout << "Initializing particle filter nyah..." << std::endl;
    ParticleFilter pf( map, ptree.get_child("particle_filter") );

    std::cout << "Initializing filter visualizer..." << std::endl;
    FilterVisualizer vis( pf, map, ptree.get_child("filter_visualizer") );
	    
	std::cout << "Initializing log reader..." << std::endl;
    RobotLogReader log( ptree.get_child("log_reader") );

    vis.Update(); // Capture first frame
	
    unsigned int lineNumber = 0;

//    cv::namedWindow( "I1", CV_WINDOW_AUTOSIZE );
//    cv::namedWindow( "I2", CV_WINDOW_AUTOSIZE );

    while( log.HasData() ) {

		SensorData data = log.GetNextData();

// 		std::cout << "Read line " << lineNumber << std::endl;
		lineNumber++;
				
        // apply update from data to particles in the pf

		pf.handleData(data);

        vis.Update(data);

        std::cout<<"\ninloop1";


    }
    std::cout<<"\nFINISHED";
    return 0;
    
}
