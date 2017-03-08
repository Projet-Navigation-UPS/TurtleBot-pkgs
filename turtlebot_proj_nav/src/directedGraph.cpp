#include <stdlib.h>
#include "tinyxml.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <string>


struct NodeProperty
{
    int id;
    std::string label;
    float x;
    float y;
};

struct LinkProperty
{
    int source;
    int target;
    int cost;
};

// Define the type of the graph - this specifies a bundled property for vertices and edges
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty, NodeProperty> Graph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperty, EdgeWeightProperty> Graph;

int main(int argc, char* argv[])
{
    
    Graph g;
    
	// Load the xml file
    TiXmlDocument XMLdoc(ros::package::getPath("turtlebot_proj_nav") + "/rsc/graph.xml");
    bool loadOkay = XMLdoc.LoadFile();
    if (loadOkay)
    {
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
                    /*typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter_edge, v1, v2;
                    std::pair<vertex_iter_edge, vertex_iter_edge> vertexPairEdge;
                    
                    for (vertexPairEdge = vertices(g); vertexPairEdge.first != vertexPairEdge.second; ++vertexPairEdge.first)
                    {
                        if(g[*vertexPairEdge.first].id == atoi(pLink->Attribute("Source"))) *v1 = *vertexPairEdge.first;
                        if(g[*vertexPairEdge.first].id == atoi(pLink->Attribute("Target"))) *v2 = *vertexPairEdge.first;
                        
                    }*/
                    add_edge(boost::vertex(0, g), boost::vertex(3, g), e, g);
                    pLink = pLink->NextSiblingElement( "Link" );
                }
            }
        }
    }
    else std::cout << "not found" << std::endl;
    
    
    
    // Display vertices
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vertexPair;
    for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
        std::cout << g[*vertexPair.first].id <<  std::endl;
        std::cout << g[*vertexPair.first].label <<  std::endl;
        std::cout << g[*vertexPair.first].x <<  std::endl;
        std::cout << g[*vertexPair.first].y <<  std::endl <<  std::endl;
    }
    
    // Display edges
    boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);
    typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> edgePair;
    for(edgePair = edges(g); edgePair.first != edgePair.second; ++edgePair.first)
    {
        std::cout << EdgeWeightMap[*edgePair.first] << " ";
    }
    
    
    	
	
	return 0;
}
