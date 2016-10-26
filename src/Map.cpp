#include "rspf/Map.h"

#include <iostream>
#include <cstdio>
#include <stdexcept>

namespace rspf {

    Map::Map(std::string filename)
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
                sscanf(&line[32], "%lf", &(Map::Resolution));
            else if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
                sscanf(&line[35], "%f", &(Map::RealOffset.x));
            else if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0)
                sscanf(&line[35], "%f", &(Map::RealOffset.y));
            else if(strncmp(line, "robot_specifications->global_mapsize_x", 38) == 0)
                sscanf(&line[35], "%f", &(Map::RealSize.x));
            else if(strncmp(line, "robot_specifications->global_mapsize_y", 38) == 0)
                sscanf(&line[35], "%f", &(Map::RealSize.y));
        }
        
        // Read the matrix size
        if (sscanf(line, "global_map[0]: %d %d", &(Map::Size.x), &(Map::Size.y)) != 2) 
        {
            fclose(mapfile);
            throw std::runtime_error("Error reading format of the file " + filename);
        }

        // Initialize the map matrix
        Values = cv::Mat::zeros(Map::Size.x, Map::Size.y, CV_64F);

        // Read the map elements
        for (int i = 0; i < Map::Size.x; ++i)
            for (int j = 0; j < Map::Size.y; ++j)
                fscanf(mapfile, "%lf", &Values.at<double>(i, j));
    }
	
	double Map::GetMapValue(double x, double y)
    {
        // Calculate the matrix element position
        int i = std::round(x / Map::Resolution);
        int j = std::round(y / Map::Resolution);

        // If the position is out of matrix, return -1 (= I don't know)
        if (i < 0 || i >= Map::Size.x || j < 0 || j >= Map::Size.y)
            return -1;
        
        // Return map's value
        return Values.at<double>(i, j);
    }
}
