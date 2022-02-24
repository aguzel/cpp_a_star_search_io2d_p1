#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to

  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

  return end_node->distance(*node);
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

  current_node->FindNeighbors();

  for (RouteModel::Node *node : current_node->neighbors) {

    node->parent = current_node;
    node->g_value = current_node->g_value + current_node->distance(*node);
    node->h_value = RoutePlanner::CalculateHValue(node);
    open_list.push_back(node);
    node->visited = true;
  }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {

  std::sort(open_list.begin(), open_list.end(),
            [](const RouteModel::Node *node1, RouteModel::Node *node2) {
              return (node1->g_value + node1->h_value) >
                     (node2->g_value + node2->h_value);
            });

  RouteModel::Node *node;
  node = open_list.back();
  open_list.pop_back();
  return node;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found  from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  // TODO: Implement your solution here.

  RouteModel::Node *node = current_node->parent;

  path_found.push_back(*current_node);             //end node is added to path_find
  distance += current_node->distance(*node);       //end node to previous node's distance is counted

  while (true) {
    path_found.push_back(*node);
    distance += (node->parent)->distance(*node);
    node = node->parent;    
    if (node == start_node) {
      path_found.push_back(*node);
      break;
    }
  }

  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

// TODO 7: Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  // TODO: Implement your solution here.

  // Initialization
  current_node = start_node;
  open_list.push_back(current_node);
  current_node->visited = true;

  //A*star Loop 
  while (open_list.size() > 0) {
    current_node = NextNode();    
  
    if (current_node == end_node) {
    this->m_Model.path = ConstructFinalPath(current_node);
    break;
    }  
    AddNeighbors(current_node);
  }
}