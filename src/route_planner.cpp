#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x , end_y);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
   for(const auto&  node: current_node->neighbors)
   {
       node->h_value= CalculateHValue(node);
       node->g_value = current_node->g_value + node->distance(*current_node);
       node->parent = current_node;
       node->visited = true;
       open_list.emplace_back(node);

   }

}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin() , open_list.end() , [](auto node1 , auto node2){
        float x = (float)(node1->h_value + node1->g_value);
        float y = (float)(node2->h_value + node2->g_value);
        return x > y;
    });
    auto pickedNode = open_list.back();
    open_list.pop_back();
    return pickedNode;

}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    auto tempcurrent_node = current_node;

    while(tempcurrent_node != start_node)
    {
        distance += tempcurrent_node->distance(*tempcurrent_node->parent);
        path_found.emplace_back(*tempcurrent_node);
        tempcurrent_node = tempcurrent_node->parent;
    }
    // pushing the start node;
    path_found.emplace_back(*tempcurrent_node);

    std::reverse(path_found.begin() , path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {

    //visiting the first node
    start_node->visited = true;
    //Adding the neightbours of the start-node
    AddNeighbors(start_node);
    RouteModel::Node* currentnode = nullptr;
    while(!open_list.empty())
    {
        currentnode  = NextNode();
        if(currentnode == end_node)
        {
          m_Model.path = ConstructFinalPath(currentnode);
          return;
        }
        AddNeighbors(currentnode);
    }

}
