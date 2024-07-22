#include "pathProcessing.h"

PathProcessor::PathProcessor(double turningRadius)
    :m_turningRadius(turningRadius)
    {}

std::shared_ptr<ompl::base::ReedsSheppStateSpace> PathProcessor::createStateSpaceSystem()
{   
    m_stateSpace = std::make_shared<ompl::base::ReedsSheppStateSpace>(m_turningRadius);
    return m_stateSpace;

}

ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> PathProcessor::createState(const ompl::base::StateSpacePtr& stateSpace)
{
    // Create a ScopedState object using ReedsSheppStateSpace
    ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> scopedState(stateSpace);
    return scopedState;
}

std::vector<Model::State> PathProcessor::getRSPath(const Model::State& start, const Model::State& goal, const ompl::base::StateSpacePtr& stateSpace) 
{
    ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> from(stateSpace);
    ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> to(stateSpace);

    from->setXY(start.x, start.y);
    from->setYaw(start.theta);

    to->setXY(goal.x, goal.y);
    to->setYaw(goal.theta);

    std::vector<ompl::base::State*> path;
    auto reedsSheppSpace = std::dynamic_pointer_cast<ompl::base::ReedsSheppStateSpace>(stateSpace);

    if (!reedsSheppSpace) 
    {
        throw std::runtime_error("Failed to cast StateSpacePtr to ReedsSheppStateSpace");
    }

    for (double t = 0; t <= 1; t += 0.1) 
    {
        ompl::base::State* state = reedsSheppSpace->allocState();
        reedsSheppSpace->interpolate(from(), to(), t, state);
        path.push_back(state);
    }

    for (auto& state : path) {
        const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();
        m_path.push_back(Model::State(se2state->getX(), se2state->getY(), se2state->getYaw()));
    }

    // Free the allocated states
    for (auto& state : path)
    {
        reedsSheppSpace->freeState(state);
    }   
    return m_path;
}
