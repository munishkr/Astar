#include "HybridAstar.h"

HybridAStar::HybridAStar(double turnRadius):m_turnRadius(turnRadius)
{
    m_PathProcessor = std::make_shared<PathProcessor>(m_turnRadius);
}

double HybridAStar::getHeuristic(const Model::State& a, const Model::State& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dtheta = std::min(std::abs(a.theta - b.theta), 2 * PI - std::abs(a.theta - b.theta));
    return sqrt(dx * dx + dy * dy) + dtheta;
}

std::vector<Model::State> HybridAStar::getHybridAstarPath(const std::vector<std::vector<int>>& grid, const Model::State& start, const Model::State& goal)
{
    struct StateWithPriority
    {
        Model::State state;
        double priority;

        bool operator>(const StateWithPriority& other) const
        {
            return priority < other.priority;
        }
    };

    std::priority_queue<StateWithPriority, std::vector<StateWithPriority>, std::greater<StateWithPriority>> openSetQueue;
    std::unordered_set<Model::State> closedSet;
    std::unordered_map<Model::State, double> gScore; // Cost from start to state
    std::unordered_map<Model::State, Model::State> cameFrom;

    openSetQueue.push({start, getHeuristic(start, goal)});
    gScore[start] = 0.0;

    while (!openSetQueue.empty()) 
    {
        Model::State current = openSetQueue.top().state;
        openSetQueue.pop();

        if (closedSet.find(current) != closedSet.end()) 
        {
            continue; // Skip already processed nodes
        }

        if (hypot(current.x - goal.x, current.y - goal.y) < 1 )
        {
            std::vector<Model::State> path;
            while (cameFrom.find(current) != cameFrom.end())
            {
                path.push_back(current);
                current = cameFrom.at(current);
            }
            path.push_back(current);
            std::reverse(path.begin(), path.end());

            return path;
        }

        closedSet.insert(current);

        // Generate neighbors 
        auto stateSpace = m_PathProcessor->createStateSpaceSystem();
        std::vector<Model::State> neighbors = m_PathProcessor->getRSPath(current, goal, stateSpace);

        // For visualisation
        std::fstream pathFile("path.txt", std::ios::out);
        if (!pathFile.is_open()) {
            std::cerr << "Error: Could not open path.txt for writing.\n";
            return {};
        }
        for (const auto& state : neighbors) {
            std::cout << "PATH \t" << state.x << " " << state.y << " " << state.theta << "\n";
            pathFile << state.x << " " << state.y << " " << state.theta << "\n";
        }
        pathFile.close();
        
        for (const auto& neighbor : neighbors)
        {
            if (neighbor.x < 0 || neighbor.x >= grid.size() || neighbor.y < 0 || neighbor.y >= grid[0].size() || grid[neighbor.x][neighbor.y] == 0) 
            {
                continue; // Ignoring obstacles or out of grid neighbours
            }

            if (closedSet.find(neighbor) != closedSet.end())
                continue; // Skip if already processed

            double tentative_gScore = gScore[current] + getHeuristic(current, neighbor);

            if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) 
            {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                double fScore = tentative_gScore + getHeuristic(neighbor, goal);
                openSetQueue.push({neighbor, fScore});
            }
        }
    }

    // Write grid to file 
    std::fstream gridFile("grid.txt", std::ios::out);
    if (!gridFile.is_open()) {
        std::cerr << "Error: Could not open grid.txt for writing.\n";
        return {};
    }
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            gridFile << cell << " ";
        }
        gridFile << "\n";
    }
    gridFile.close();

    return {};
}