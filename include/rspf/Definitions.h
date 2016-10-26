#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#include <string>

namespace rspf {
    class Definitions
    {
    public:
        static std::string MapFile;
    };
    
    std::string Definitions::MapFile = "map/wean.dat";
}

#endif
