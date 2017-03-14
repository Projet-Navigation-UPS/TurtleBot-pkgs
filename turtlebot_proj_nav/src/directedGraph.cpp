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

#include "graph.hpp"

int main(int argc, char* argv[])
{
    
    Graph Graph_test = xmlToGraph("graph.xml");
    //NodeProperty n = nextNode(0, 3, "graph.xml");
	displayGraph(Graph_test);	
	
	return 0;
}
