#include "rspf/MyMap.h"

#include <iostream>
#include <cstdio>
#include <stdexcept>

namespace rspf {

    MyMap::MyMap(std::string filename)
    {
        // Open the map file
        FILE *mapfile;
        if ((mapfile = fopen(filename.c_str(), "rt")) == NULL)
            throw std::runtime_error("Error opening the file " + filename);
        
        // Read headers
        char line[256];
        while ((fgets(line, 256, mapfile) != NULL) && (strncmp("global_map[0]", line, 13) != 0)) 
        {
            if(strncmp(line, "robot_specifications->resolution", 32) == 0)
                sscanf(&line[32], "%lf", &(MyMap::Resolution));
            else if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
                sscanf(&line[35], "%f", &(MyMap::RealOffset.x));
            else if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0)
                sscanf(&line[35], "%f", &(MyMap::RealOffset.y));
            else if(strncmp(line, "robot_specifications->global_mapsize_x", 38) == 0)
                sscanf(&line[35], "%f", &(MyMap::RealSize.x));
            else if(strncmp(line, "robot_specifications->global_mapsize_y", 38) == 0)
                sscanf(&line[35], "%f", &(MyMap::RealSize.y));
        }
        
        // Read the matrix size
        if (sscanf(line, "global_map[0]: %d %d", &(MyMap::Size.x), &(MyMap::Size.y)) != 2)
        {
            fclose(mapfile);
            throw std::runtime_error("Error reading format of the file " + filename);
        }

        // Initialize the map matrix
        Values = cv::Mat::zeros(MyMap::Size.x, MyMap::Size.y, CV_64F);

        // Read the map elements
        for (int i = 0; i < MyMap::Size.x; ++i)
            for (int j = 0; j < MyMap::Size.y; ++j)
                fscanf(mapfile, "%lf", &Values.at<double>(j, i));

        cv::flip(Values,Values,0);
    }
	
    double rspf::MyMap::GetRealValue(double x, double y) const
    {
        // Calculate the matrix element position
        int i = std::round(x * MyMap::Resolution);
        int j = std::round(y * MyMap::Resolution);

        // If the position is out of matrix, return -1 (= I don't know)
        if (i < 0 || i >= MyMap::Size.x || j < 0 || j >= MyMap::Size.y)
            return -1;
        
        // Return map's value
        return Values.at<double>(i, j);
    }
}

