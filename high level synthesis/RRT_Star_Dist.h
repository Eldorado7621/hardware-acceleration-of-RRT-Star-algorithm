#include "RRT_Star.hpp"
#include "ports.h"


#define DATALENGHT  4


/* Node_RRT Rand_Conf( Region &Range );
 int Nearest( Node_RRT &x_rand, Node_RRT RRT_Star[Node], int N );
 float Cal_Dist( Node_RRT &x_A, Node_RRT &x_B );
 float Cal_Squared_Dist( Node_RRT &x_A, Node_RRT &x_B );

 Node_RRT StreamReader( Node_RRT &x_nearest, Node_RRT &x_rand );

 bool Obstacle_Free( Position_Holonomic &A, Position_Holonomic &B, float Squared_R_Robot,
        Polygon_2 &Geo);

 bool Colition( Point_2 &A, float Squared_R_Robot, Polygon_2 &Geo );

 void Near( std::list<int> &X_near, Node_RRT RRT_Star[Node], Node_RRT &x_new, float Squared_r_n, int N );
 float Cost( Position_Holonomic &A, Position_Holonomic &B );
 float Cal_Angle( float Theta_A, float Theta_B );
 void Rewire_the_tree( int Pos_x, Node_RRT RRT_Star[Node] );

 void Range_search( Node_RRT Graph[Node], Region &Region, std::list<int> &List_Point );
 */
void Build_RRT_Star(hls::stream<axis_data> &dataOutStream,Initial_pos Pos_Init,int Total_Num_Node,float R_Robot,
    		float Gamma_rrt,Min_max_region min_max_region,float d,float mu,hls::stream<axis_data_inp> &envDataInputStream);
