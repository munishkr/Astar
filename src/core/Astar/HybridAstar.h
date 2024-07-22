#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "../ompl/pathProcessing.h"
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <queue>


const double PI = 3.14159265358979323846;
const double SQRT2 = 1.41421356237309504880;

class HybridAStar
{
    public:
        /**
         * @brief Construct a new Hybrid A Star object
         * 
         * @param turnRadius 
         */
        HybridAStar(double turnRadius);

        /**
         * @brief Destroy the Hybrid A Star object
         * 
         */
        ~HybridAStar(){};

        /**
         * @brief Get the Heuristic object
         * 
         * @param a 
         * @param b 
         * @return double 
         */
        double getHeuristic(const Model::State& a, const Model::State& b);

        /**
         * @brief Get the Hybrid Astar Path object
         * 
         * @param grid 
         * @param start 
         * @param goal 
         * @return std::vector<Model::State> 
         */
        std::vector<Model::State> getHybridAstarPath(const std::vector<std::vector<int>>& grid, const Model::State& start, const Model::State& goal);
    
    private:
        std::shared_ptr<PathProcessor> m_PathProcessor;
        double m_turnRadius;
};

