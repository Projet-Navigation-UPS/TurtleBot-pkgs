/*
  graph.hpp
  Bruno Dato

  Header file 
 
 */
#ifndef _GRAPH_HPP_
#define _GRAPH_HPP_

#include <stdlib.h>
#include "tinyxml.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <vector>
#include <string>

// Definition of the properties of a node/vertex
struct NodeProperty
{
    int id;
    std::string label;
    float x;
    float y;
    float orientation;
} ;

// Define the type of the graph - this specifies a bundled property for vertices and edges
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperty, EdgeWeightProperty> Graph;

// Functions
Graph xmlToGraph(std::string xmlFile);
void displayGraph(Graph g);
NodeProperty nextNode(int sourceId, int targetId, std::string xmlFile);

#endif
