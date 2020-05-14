#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value = node->distance(*end_node);
    return h_value;
}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();
    
    for(auto neighbor : current_node->neighbors) {
    neighbor -> parent = current_node;
    float curr_g = current_node -> g_value;
    float dist = current_node -> distance(*neighbor);
    neighbor -> g_value = curr_g + dist;
    neighbor-> h_value = CalculateHValue(neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;

    }
}


// NextNode method to sort the open list and return the next node.


RouteModel::Node *RoutePlanner::NextNode() {
    std:sort(open_list.begin(), open_list.end(), [](auto &x, auto &y) {
        return (x->h_value + x->g_value) > (y->h_value + y->g_value);
    } );

    RouteModel::Node *lowest = open_list.back();
    open_list.pop_back();
    return lowest;
}


// Complete the ConstructFinalPath method to return the final path found from A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node -> parent != nullptr) {
        path_found.push_back(*current_node);
        distance += current_node -> distance(*current_node -> parent);
        current_node = current_node -> parent;
    }
    // for start node
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node -> visited = true;
    open_list.push_back(start_node);

    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node -> distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            
        } else AddNeighbors(current_node);    
    }

}