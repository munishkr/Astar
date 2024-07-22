#include "common/MapLoader.h"
#include "core/Astar/HybridAstar.h"

#include <iostream>
#include <string>
#include <fstream>



int main(int argc, char* argv[]) 
{
    try
    {
        // Get the inputs from cmd line
        const std::string imagePath = "drivable_space.png";//argv[1];
        int turningRadius = 1.0;//std::stoi(argv[0]);
    

        //Read the map from image   
        Maploader mapLoader;
        std::vector<std::vector<int>> grid = mapLoader.loadMap(imagePath);

        //Load the grid and put in start and end points
        Model::State start(50, 50, 0);
        Model::State goal(130, 70, 0);

        HybridAStar planner(turningRadius);
        std::vector<Model::State> path = planner.getHybridAstarPath(grid, start, goal);

        if (!path.empty()) 
        {
            std::cout << "Path found:" << std::endl;
            for (const Model::State& s : path) 
            {
                std::cout << "(" << s.x << ", " << s.y << ", " << s.theta << ")" << std::endl;
            }
        } 
        else 
        {
            std::cout << "No path found." << std::endl;
        }
    
    } 
    catch (const std::exception& e) 
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
        
}