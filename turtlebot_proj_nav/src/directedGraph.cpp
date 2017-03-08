#include <stdlib.h>
#include "tinyxml.h"
#include <iostream>

int main(int argc, char* argv[])
{
	
	// Load the xml file, I put your XML in a file named test.xml
    TiXmlDocument XMLdoc("graph.xml");
    bool loadOkay = XMLdoc.LoadFile();
    if (loadOkay)
    {
        std::cout << "grah.xml loaded" << std::endl;
        TiXmlElement *pDirectedGraph, *pNodes, *pNode, *pApp, *pLineFormat;
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
                pNode = pNode->NextSiblingElement( "Node" );
            }
            }
        }
    }
    else std::cout << "not found" << std::endl;
	
	
	return 0;
}
