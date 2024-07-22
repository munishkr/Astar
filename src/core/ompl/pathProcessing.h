#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include "../utils/utils.h"

class PathProcessor
{
    public:
        
        /**
         * @brief Construct a new Path Processor object
         * 
         * @param trajectoryType 
         * @param turningRadius 
         */
        PathProcessor(double turningRadius);

        /**
         * @brief Destroy the Path Processor object
         * 
         */
        ~PathProcessor(){};

        /**
         * @brief Create a State Space System object
         * 
         * @return std::shared_ptr<ompl::base::ReedsSheppStateSpace> 
         */
        std::shared_ptr<ompl::base::ReedsSheppStateSpace> createStateSpaceSystem();

        /**
         * @brief get the ReedSheep path 
         * 
         * @param start 
         * @param goal 
         * @param stateSpace 
         * @return std::vector<Model::State> 
         */
        std::vector<Model::State> getRSPath(const Model::State& start, const Model::State& goal, const ompl::base::StateSpacePtr& stateSpace);

    private:

        /**
         * @brief Create a State object
         * 
         * @param stateSpace 
         * @return ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> 
         */
        ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> createState(const ompl::base::StateSpacePtr& stateSpace);

    private:

        //Contains the resultant path from state space
        std::vector<Model::State> m_path;

        //ReedSheep shared_ptr
        std::shared_ptr<ompl::base::ReedsSheppStateSpace> m_stateSpace; 
        
        //Turning radius for the model
        double m_turningRadius{0.0};

};