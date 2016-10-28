#CMake file for src folder

set( RSPF_particlefilter_SOURCES    
    Distributions.cpp
    FilterVisualizer.cpp
    Parameterized.cpp
    Particle.cpp
    ParticleFilter.cpp
    PoseSE2.cpp
    RobotLogReader.cpp
    SensorModel.cpp
    LaserModel.cpp
    MyMap.cpp
    MyTransitionModel.cpp
    MyResampler.cpp
)


add_library( RSPF_particlefilter SHARED ${RSPF_particlefilter_SOURCES} )

set_target_properties( RSPF_particlefilter PROPERTIES
    OUTPUT_NAME RSPF_particlefilter )

target_link_libraries( RSPF_particlefilter ${RSPF_LIBRARIES} armadillo)
