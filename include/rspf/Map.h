#ifndef _MAP_H_
#define _MAP_H_

#include <opencv2/core/core.hpp>
#include <string>

namespace rspf {

class Map {
public:
    double Resolution;
    cv::Point2f RealSize;
    cv::Point2f RealOffset;
    cv::Point2i Size;
    cv::Mat Values;
    
    Map(std::string filename);
    double GetMapValue(double x, double y);
};

}

#endif
