#ifndef _MY_MAP_H_
#define _MY_MAP_H_

#include <opencv2/core/core.hpp>
#include <string>

namespace rspf {

class MyMap {
public:
    double Resolution;
    cv::Point2f RealSize;
    cv::Point2f RealOffset;
    cv::Point2i Size;
    cv::Mat Values;
    
    MyMap(std::string filename);
    double GetRealValue(double x, double y) const;
};

}

#endif

