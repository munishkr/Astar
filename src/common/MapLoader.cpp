#include "MapLoader.h"

std::vector<std::vector<int>> Maploader::loadMap(const std::string filePath)
{
    // Read file
    std::string f = "../drivable_space.pgm";
    cv::Mat image = cv::imread(f, cv::IMREAD_GRAYSCALE);
    if (image.empty())
    {
        throw std::runtime_error("Failed to load image");
    }

    // initialise the grid with read rows/cols
    std::vector<std::vector<int>> grid(image.rows, std::vector<int>(image.cols));

    //Populating the grid
    for(int i = 0; i < image.rows; i++)
    {
        for(int j = 0; j < image.cols; j++)
        {
            grid[i][j] = static_cast<int>(image.at<uint8_t>(i, j)) > 120 ? 0 : 1;
        }
    }
    return grid;
}
