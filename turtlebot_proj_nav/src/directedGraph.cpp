#include <stdlib.h>
#include "tinyxml.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/graph/directed_graph.hpp>
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

// Define the type of the graph - this specifies a bundled property for vertices
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperty> Graph;

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
                    /*g[v].id = pNode->Attribute("Id");
                    g[v].label = pNode->Attribute("Label");
                    g[v].x = pNode->Attribute("PositionX");
                    g[v].y = pNode->Attribute("PositionY");*/
                    g[v].id = 1;
                    g[v].label = pNode->Attribute("Label");
                    g[v].x = 0.0;
                    g[v].y = 0.0;
                    
                       
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
                    pLink = pLink->NextSiblingElement( "Link" );
                }
            }
        }
    }
    else std::cout << "not found" << std::endl;
    
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vertexPair;
    for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
        std::cout << g[*vertexPair.first].id <<  std::endl;
        std::cout << g[*vertexPair.first].label <<  std::endl;
        std::cout << g[*vertexPair.first].x <<  std::endl;
        std::cout << g[*vertexPair.first].y <<  std::endl <<  std::endl;
    }
    
    
    	
	
	return 0;
}
