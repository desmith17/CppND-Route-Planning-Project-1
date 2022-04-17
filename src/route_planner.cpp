#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //populate current_node.neighbors
    current_node->FindNeighbors();
    //Iterate through the neighbors
    for (RouteModel::Node *node : current_node->neighbors){
        // Check if node has been visited
        if(node->visited != true){
            // set the current node as the nodes parent
            node->parent = current_node;
            //Calculate nodes g_value
            node->g_value = current_node->g_value + node->distance(*current_node);
            //set the nodes h_value
            node->h_value = CalculateHValue(node);
            //Add node to openlist
            open_list.push_back(node);
            //set node as visited
            node->visited = true;
        }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // Reference: https://stackoverflow.com/questions/14419520/sort-vector-of-vectors
    // Sort open list where the vector goes from highest to lowest sum
    std::sort(open_list.begin(),open_list.end(),[](const auto & n1,const auto & n2){return (n1->g_value+n1->h_value) < (n2->g_value+n2->h_value);});
    // Point to the first node in the vector
    RouteModel::Node* pointer_ls = open_list.front();
    // Remove first node in the vector
    open_list.erase(open_list.begin());

    return pointer_ls;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    // TODO: Implement your solution here.

  // Place the node in the front of the path_found vector 
    path_found.emplace(path_found.begin(),*current_node);
    // Iterate through the parent nodes till the parent node is empty
    while(current_node != start_node){
        // Create pnode for the current nodes parent
        RouteModel::Node* pnode = current_node->parent;
        // Add the distance between the node and its parent to the distance variable
        distance += current_node->distance(*pnode);
        // Set the node to be the node's parent
        current_node = pnode;
        // Add the pnode to the front of the path_found vector
        path_found.emplace(path_found.begin(),*pnode);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    // Set current_node to the start_node
    current_node = start_node;
    // Populate open_list with AddNeighbors function
    AddNeighbors(current_node);

    while (open_list.size() > 0){
        // Set current_node to the next node
        current_node = NextNode();
        // If current_node is the end_node
        if (current_node->distance(*end_node) == 0){
            // Call ConstructFinalPath using the current(end)_node
            m_Model.path = ConstructFinalPath(current_node);
            // Exit the while loop
            return;
        }
        // Add the neighbors to the next nodes open_list
        AddNeighbors(current_node);
    }
}
