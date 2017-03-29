/*
  graph.cpp
  Bruno Dato

  Functions for graph parsing and shortest path calculus.
 
 */
#include "graph.hpp"

// Returns a graph parsed from a XML file
Graph xmlToGraph(std::string xmlFile)
{
    Graph g;
    
	// Load the xml file
    TiXmlDocument XMLdoc(ros::package::getPath("turtlebot_proj_nav") + "/rsc/" + xmlFile);
    bool loadOkay = XMLdoc.LoadFile();
    
    if (loadOkay)
    {
        ROS_INFO("XML Reading");
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
                    // Parse Node
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
                    // Parse Link
                    EdgeWeightProperty e = atof(pLink->Attribute("Cost"));
                    add_edge(boost::vertex(atoi(pLink->Attribute("Source")), g), boost::vertex(atoi(pLink->Attribute("Target")), g), e, g);
                    pLink = pLink->NextSiblingElement( "Link" );
                }
            }
        }
    }
    else ROS_ERROR("Can't read XML file"); 
    //displayGraph(g);
    
    return g;
}

// Displays vertices and edges of a graph
void displayGraph(Graph g)
{
    // Display vertices
    std::cout << "Vertices" <<  std::endl;
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vertexPair;
    for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
        std::cout << g[*vertexPair.first].id << " " << g[*vertexPair.first].label << " " << g[*vertexPair.first].x << " " << g[*vertexPair.first].y << " " << g[*vertexPair.first].orientation << std::endl;
    }
    
    // Display edges weights
    std::cout << "Edges weights" <<  std::endl;
    boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);
    typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> edgePair;
    for(edgePair = edges(g); edgePair.first != edgePair.second; ++edgePair.first)
    {
        std::cout << boost::source(*edgePair.first, g) << " --" << EdgeWeightMap[*edgePair.first] << "--> "<< boost::target(*edgePair.first, g) << std::endl;
    }
}

// Return the next node to follow the shortest path from a source node to a target node
NodeProperty nextNode(int sourceId, int targetId, std::string xmlFile)
{
    ROS_INFO("Seaking shortest path to find next move");
    ROS_INFO("PositionId : %d --> GoalId : %d",sourceId,targetId);
    // Parse graph
    Graph g = xmlToGraph(xmlFile); 
    
    NodeProperty node; 
    std::vector<Graph::vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));
    Graph::vertex_descriptor s = vertex(targetId, g); // Target vertex
    
    // Dijkstra shortest path
    dijkstra_shortest_paths(g, s,
                          predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
                          distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));
                          
    //std::cout << "Distances and parents:" << std::endl;
    typedef boost::graph_traits<Graph>::vertex_iterator vi;
    std::pair<vi, vi> vP;
    for (vP = vertices(g); vP.first != vP.second; ++vP.first) {
        //std::cout << "distance(" << g[*vP.first].id << ") = " << d[*vP.first] <<  ", ";
        //std::cout << "parent(" << g[*vP.first].id << ") = " << g[p[*vP.first]].id << std::endl;
        
        // The next node is the parent of the source that is on a shortest path starting from the target
        if (g[*vP.first].id == sourceId) 
        {
            ROS_INFO("Next marker (%lf,%lf)",g[p[*vP.first]].x,g[p[*vP.first]].y);
            node = g[p[*vP.first]];
        }
    }
    return node;
}




