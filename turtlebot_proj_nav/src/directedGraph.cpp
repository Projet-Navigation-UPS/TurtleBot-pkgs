#include <stdlib.h>
#include "tinyxml.h"
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <vector>
#include <string>
#include <iterator>
#include <boost/utility.hpp>

#include <boost/graph/bellman_ford_shortest_paths.hpp>


struct NodeProperty
{
    int id;
    std::string label;
    float x;
    float y;
    float orientation;
};

struct LinkProperty
{
    int source;
    int target;
    int cost;
};


// Define the type of the graph - this specifies a bundled property for vertices and edges
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperty, EdgeWeightProperty> Graph;



int main(int argc, char* argv[])
{
    
    Graph g;
    
	// Load the xml file
    TiXmlDocument XMLdoc(ros::package::getPath("turtlebot_proj_nav") + "/rsc/graph.xml");
    bool loadOkay = XMLdoc.LoadFile();
    
    if (loadOkay)
    {
        std::cout << "XML Reading" <<  std::endl;
        std::cout << "graph.xml loaded" << std::endl;
        TiXmlElement *pDirectedGraph, *pNodes, *pNode, *pLinks, *pLink;
        pDirectedGraph = XMLdoc.FirstChildElement( "DirectedGraph" );
        if ( pDirectedGraph )
        {
            // Parse Nodes
            pNodes = pDirectedGraph->FirstChildElement("Nodes");
            if (pNodes)
            {
                pNode = pNodes->FirstChildElement("Node");
                while ( pNode )
                {
                    std::cout << "Node : Id= '" << pNode->Attribute("Id") << "', Label='" << pNode->Attribute("Label") << "'" << std::endl;
                    Graph::vertex_descriptor v = boost::add_vertex(g);
                    g[v].id = atoi(pNode->Attribute("Id"));
                    g[v].label = pNode->Attribute("Label");
                    g[v].x = atof(pNode->Attribute("PositionX"));
                    g[v].y = atof(pNode->Attribute("PositionY"));
                    g[v].orientation = atof(pNode->Attribute("Orientation"));
                                           
                    pNode = pNode->NextSiblingElement( "Node" );
                }
            }
            // Parse Links
            pLinks = pDirectedGraph->FirstChildElement("Links");
            if (pLinks)
            {
                pLink = pLinks->FirstChildElement("Link");
                while ( pLink )
                {
                    std::cout << "Link : Source= '" << pLink->Attribute("Source") << "', Target='" << pLink->Attribute("Target") << "'" << std::endl;
                    EdgeWeightProperty e = atof(pLink->Attribute("Cost"));
                    add_edge(boost::vertex(atoi(pLink->Attribute("Source")), g), boost::vertex(atoi(pLink->Attribute("Target")), g), e, g);
                    pLink = pLink->NextSiblingElement( "Link" );
                }
            }
        }
    }
    else std::cout << "Can't read XML file" << std::endl;
    
    
    
    // Display vertices
    std::cout << "Display vertices" <<  std::endl;
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vertexPair;
    for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
        std::cout << g[*vertexPair.first].id << " " << g[*vertexPair.first].label << " " << g[*vertexPair.first].x << " " << g[*vertexPair.first].y << " " << g[*vertexPair.first].orientation << std::endl;
    }
    
    // Display edges weights
    std::cout << "Display edges weights" <<  std::endl;
    boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);
    typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> edgePair;
    for(edgePair = edges(g); edgePair.first != edgePair.second; ++edgePair.first)
    {
        std::cout << boost::source(*edgePair.first, g) << " --" << EdgeWeightMap[*edgePair.first] << "--> "<< boost::target(*edgePair.first, g) << std::endl;
    }
    
    
    //Dijkstra s
    std::vector<Graph::vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));
    Graph::vertex_descriptor s = vertex(0, g);
    
    dijkstra_shortest_paths(g, s,
                          predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
                          distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));
                          
    std::cout << "distances and parents:" << std::endl;
    typedef boost::graph_traits<Graph>::vertex_iterator vi;
    std::pair<vi, vi> vP;
    for (vP = vertices(g); vP.first != vP.second; ++vP.first) {
    std::cout << "distance(" << g[*vP.first].id << ") = " << d[*vP.first] <<  ", ";
    std::cout << "parent(" << g[*vP.first].id << ") = " << g[p[*vP.first]].id << std::endl;
    }
    std::cout << std::endl;    
    
    
	
	return 0;
}
