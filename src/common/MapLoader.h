#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

class Maploader
{
    public:
        
        /**
         * @brief Construct a new Maploader object
         * 
         */
        Maploader(){};

        /**
         * @brief Destroy the Maploader object
         * 
         */
        ~Maploader(){};

        /**
         * @brief Read the image and store that as a map
         * 
         * @param filePath 
         * @return std::vector<std::vector<int>> 
         */
        std::vector<std::vector<int>> loadMap(const std::string filePath);
};