#include "visib_init.hpp"
#include "graph.cpp"

void displayGraphVisib(Graph g, float x1[], float y1[])
{
	int a=0;
    // Display vertices
    std::cout << "Display vertices" <<  std::endl;
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vertexPair;
    for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
        std::cout << g[*vertexPair.first].id << " " << g[*vertexPair.first].label << " " << g[*vertexPair.first].x << " " << g[*vertexPair.first].y << " " << g[*vertexPair.first].orientation << std::endl;

	x1[a]=g[*vertexPair.first].x;
	y1[a]=g[*vertexPair.first].y;
	a+=1;
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
}


using namespace std;


void Ecriture_carte_visib()
{
   	int n1=920;
	int n2=900;
	int m=6;
	int x[]={0,0,0,0,0,0};//colonne
    	int y[]={0,0,0,0,0,0};//ligne     
	float t[]={pi/2,pi/2,0,0,0,0}; 
	
	int i,j,k,l,o,p=0,q=0,r=0,s=0,u=0,w=0,a;
	float dist=1.0 ;
	int pix[n1][n2],pix2[n1][n2];
	int v[m][m];
	float yn[m];
	float x1[5],y1[5];
    	float angle[]={0.0,0.0,0.0,0.0,0.0,0.0};
	float alphamax[]={pi/4,pi/4,pi/2,pi/2,pi/2,pi/2};
	int distancemax=30;
	int distancemin=1;

	float distancecm=200,xn;
	//1px = 0.494134897 cm
	// x px = 337cm

	Graph Graph_test = xmlToGraph("graph.xml");
    	displayGraphVisib(Graph_test,x1,y1);

	
	for(a=0;a<m;a++)
	{	
		printf("*************************** x1 = %lf \t y1 = %lf *********************\n",x1[a],y1[a]);
		x[a]=y1[a]*100/0.494134897;
		y[a]=x1[a]*100/0.494134897;
		printf("*************************** x1 = %d \t y1 = %d *********************\n",x[a],y[a]);
	}


	//ros::Publisher pubVisib(node.advertise<std_msgs::Int8>("/nav/Visib", 1));

	//std_msgs::Int8 visib;
	//visib.data=0;

	//distancemax=distancecm/0.494134897;
	printf("Dmax = %d\n",distancemax);
	//distancemin=20/0.494134897;	
	printf("Dmin = %d\n",distancemin);

    	//ros::Rate loop_rate(5); // 2Hz 

	ofstream fichier("src/TurtleBot-pkgs/turtlebot_proj_nav/map/visib.pgm", ios::out | ios::trunc);  
		// ouverture en écriture avec effacement du fichier ouvert

   // while (ros::ok()) //&& r<1) 
    //{
		//ros::spinOnce();
    	
		if(fichier && (r<1))
        		{	
				printf("Création de la carte \n");
				fichier << "P2" << endl;
				fichier << "#Thibaut" << endl;
				fichier << "#Carte de visibilite" << endl;
				fichier << n1 << " " << n2 << endl;
				fichier << "15" << endl;
			}
			//else
                		//cerr << "Impossible d'ouvrir le fichier !" << endl;

    
		for (i=0;i<n2;i++) // colonne
		{
			for (j=0;j<n1;j++) // ligne
			{
				pix[i][j]=15;
				//pix2[i][j]=1000000;
				for(k=0;k<m;k++) // amers
				{
					
					/*if(t[k]>-90 && t[k]<90) //si theta compris entre -90 degre et 90 degre
						v[k][1]=-y[k];
					else if (t[k]<-90 && t[k]>90)
						v[k][1]=x[k];
						else v[k][1]=90;*/
				
					//yn[k]=2;//cos(t[k]);
					
					dist = sqrt(pow(i-y[k],2)+pow(j-x[k],2));

					// xn = -1; yn[k] = tan(t[k])*xn;
					if(t[k]<3*pi/2 && t[k]>pi/2)
						xn = 1;
					else 	xn = -1;				
					
					if(t[k]<pi)
						yn[k] = -(tan(t[k])*xn);
					else yn[k] = (tan(t[k])*xn);
					
					//xvp = 
					//yvp = 

					/*if(i<y[k] && j<x[k])
						angle[k]=180-((asin((i-y[k])/dist))*180/pi);
					else if (i>y[k] && j<x[k])
						angle[k]=(asin((i-y[k])/dist))*180/pi;
					else if (i<y[k] && j>x[k])
						angle[k]=-180+((asin((i-y[k])/dist))*180/pi);
					else if (i>y[k] && j>x[k])
						angle[k]=-((asin((i-y[k])/dist))*180/pi);
					else angle[k]=0;*/
					angle[k]=acos(((j-x[k])*xn+((i-y[k])*yn[k]))/(sqrt(pow((j-x[k]),2)+pow((i-y[k]),2))*(sqrt(pow(xn,2)+pow(yn[k],2)))));//*180/pi;					
					//printf("Angle : %f \n", angle[k]);

					if((dist<=distancemax && dist>=distancemin) && (angle[k]<=alphamax[k] && angle[k]>=-alphamax[k]))
					{	pix[i][j]-=3;
						/*if(k=0) pix2[i][j]+=000001;
						if(k=1) pix2[i][j]+=000010;
						if(k=2) pix2[i][j]+=000100;
						if(k=3) pix2[i][j]+=001000;
						if(k=4) pix2[i][j]+=010000;
						if(k=5) pix2[i][j]+=100000;*/
					}
					else 
						if(dist == 0.0)
						{	pix[i][j]=0;
							pix2[i][j]=0;
							//printf("Amers :\ni=%d\tj=%d\n",i,j);
							
						}
				}

				//if(i<n&&j<n)
				//printf("%d ", pix[i][j]);
				
				q+=1;
				if(r<1)	//flag pour savoir si l'ecriture s'est deja faite 1 fois
				{	fichier << pix[i][j] << " " ;
					p+=1;
				}
				if(p>=70 && (r<1))// condition de retour a la ligne
				{
					fichier << endl;
					p=0;		
				}
				else if((p<70 && (r==1)))
				{
					printf("Integer false, image non generee correctement !\n");
				}
			}
		}

		//printf("i=%d\tj=%d\n",i,j); //Verification iterations ligne et colonne
		//printf("q=%d\n",q); //Verification nb totale d'iterations

		if(r<1)      		        
        	{	
			fichier.close();
			printf("Carte generee correctement, fermeture du fichier, veuillez patienter environ 10 sec... \n");
			r=1;
		}
		
		// --------------- Publish the number of markers ---------

		/*for (i=150;i<n2;i++) // colonne n2
		{
			for (j=600;j<n1;j++) // ligne n1
			{
				for(k=1;k<m;k++) // amers
				{
					printf("pix2[%d][%d]= %d\n",i,j,pix2[i][j]);
					if(pix2[i][j]==1104680)
					{
						visib.data=1;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						//printf("Nombre d'amers visibles : %d \n",k);
						pubVisib.publish(visib);
						printf("Numéro de l amer détectable : 0 \n");						
					}
					else if(pix2[i][j]==17)
					{
						visib.data=2;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						printf("Numéros des amers détectables : %d et %d \n",k-1,k);
					}
					else if(pix2[i][j]==18)
					{
						visib.data=3;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						printf("Numéros des amers détectables : %d, %d et %d \n",k-1,k,k+1);
					}
					else if(pix2[i][j]==19)
					{
						visib.data=4;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						printf("Numéros des amers détectables : %d, %d, %d et %d \n",k-1,k,k+1,k+2);
					}
					/*else
					{
						visib.data=0;
						ROS_INFO("Pas d amer visible : %d", visib.data);
						pubVisib.publish(visib);
					}
				}
			}
		}*/
		
		//loop_rate.sleep();
    //}
	//return 0;
}

/*void Lecture_carte_visib()
{ 
	vector <string> monTableau;
	ifstream fichier("src/TurtleBot-pkgs/turtlebot_proj_nav/map/visib.pgm", ios::in);

        if(fichier)
        {
                int entier1, entier2;
                string chaine1, chaine2;

		while( !fichier.eof() )
        	{
            		monTableau.push_back("");//creation d'une ligne vide

           	 	getline(fichier, monTableau.back());//lecture d'une ligne du fichier

            		int ligne = monTableau.size() - 1;//je recupere la taille du tableau (-1 pour la ligne 0)

            		if(monTableau[ligne].empty())//si la ligne est vide
                		monTableau.pop_back();//on la retire du tableau
        	}

        cout << "nombre de lignes : " << monTableau.size() << endl;//j'affiche le nombre de lignes pour test
    	        
	fichier.close();
        }
        else
                cerr << "Impossible d'ouvrir le fichier !" << endl;
}*/

table pgm_imread(char *argv)			//reads pgm image
	{
	char line[80];
	int imagetype = 0, cols, rows, maxintensity, p2read;
	int **data;
	unsigned char p5read;
	stringstream buffer;
	ifstream infile(argv);			//opens image
	if (infile == NULL)
    		{
	        cerr<<argv[1]<<" either cannot be read or does not exist\n";
        	exit(0);
	        }
	infile.getline(line, 80);
	if(line[0] == 'P')
		switch(line[1])
			{
			case '5' :	imagetype = 1; cout<<"P5 PGM image detected\n"; 		break; //raw image
			case '2' :	imagetype = 2; cout<<"P2 PGM image detected\n"; 		break; //ASCII image
			default	 :	imagetype = 0; cerr<<"unsupported PGM image format\n"; exit(0); break;
			}
	else
		{
		cerr<<"Invalid image format\n";
		exit(0);
		}
	while (infile.peek()=='#')
        	infile.getline(line,80); 	//read all the comments and oomit them.
	infile >> cols >> rows >> maxintensity;	//read the no of coloumns rows and pixel intensity
	data = new int* [cols];		//memory allocation
	if (!data)
		{
      	     cout << "allocation failure in matrix";
	     exit(1);
    		}
	for(int i = 0; i<cols;i++)
		{
		data[i] = new int[rows];	//making allocation for 2d matrix
	        if (!data[i])
			{
	       		cout << "allocation failure in matrix";
	       		exit(1);
      			}
		}
	if(imagetype == 1)			//raw image mode
		for (int i=0;i<cols;i++)
        	{
        	 for (int j=0;j<rows;j++)
			{
         	        infile>>p5read;
			data[i][j] = (int)p5read;
			//printf("data[%d][%d]= %d \n",i,j,data[i][j]);
			}
	        }
	else if(imagetype == 2)			//ASCII image mode
		{
		buffer << infile.rdbuf();
		for (int i=0;i<cols;i++)
       			{
       			 for (int j=0;j<rows;j++)
				{
       			        buffer>>p2read;
				data[i][j] = p2read;
				//printf("data[%d][%d]= %d \n",i,j,data[i][j]);
				}
       			 }
		}
	table image = {data, cols, rows};
	cout<<"Image read complete\n";
	infile.close();
    	return image; 		 		//returns the image as a 2-D array in a structure
}
