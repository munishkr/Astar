#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

namespace Model
{
    struct State 
    {
        double x;          // X coordinate
        double y;          // Y coordinate
        double theta;      // Orientation 
        double fScore;     // Cost associated with the state
        
        // Constructors
        State() : x(0.0), y(0.0), theta(0.0), fScore(0.0) {}
        State(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta), fScore(0.0) {}
        State(double _x, double _y, double _theta, double _fScore) : x(_x), y(_y), theta(_theta), fScore(_fScore) {}

        
        bool operator==(const State& other) const 
        {
            return x == other.x && y == other.y && std::abs(theta - other.theta) < 1e-6; 
        }
        
        bool operator!=(const State& other) const {
            return !(*this == other);
        }

        // Define a less-than operator for priority queue 
        bool operator<(const State& other) const {
            return fScore < other.fScore; 
        }
    };
}
namespace std 
{
    template <>
    struct hash<Model::State> 
    {
        std::size_t operator()(const Model::State& state) const
        {
            return std::hash<double>()(state.x) ^ std::hash<double>()(state.y) ^ std::hash<double>()(state.theta);
        }
    };
}

struct CompareStates 
{
    bool operator()(const Model::State& a, const Model::State& b) const 
    {
        // Define priority comparison
        return a.x > b.x; 
    }
};

#endif // UTILS_H
