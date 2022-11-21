#include "RRT_Star_Dist.cpp"
#include "Auxiliary.cpp"

int main(int argc, char* argv[]) 
{
    
    //Version that uses a distance cost funtion
    //Version that uses Kd-tree
    cout << "Main" <<endl;
    srand ( 0 ); 
    unsigned t0, t1;
    int Total_Num_Node;//Number of nodes in the RRT*
    float R_Robot;//Robot's radio
    float Radio_Goal;//Radio goal
    float Gamma_rrt;//Constant Gamma
    float Dim = 2;//Dimension of space
    float Mu;//Constant Mu
    Position_Holonomic Pos_Init; //Initial position of robot
    Position_Holonomic Pos_Goal; //Position of Goal
    Polygon_2 Environment; //Environment
    
    string Name = argv[ 1 ]; //Select Environment
    cout<<"Environment: "<<Name<<endl;
    bool Verbose = true;

    Auxiliary::Read_data( Pos_Init, Pos_Goal, Total_Num_Node, R_Robot, Radio_Goal,
	    Gamma_rrt, Mu, Environment, Name );
    
    //vector<Node_RRT> RRT_Star( Total_Num_Node ); //RRT*
    Node_RRT RRT_Star[Total_Num_Node];
    Region RRT_Star_Region( Environment, R_Robot );
    
    t0 = clock();
    RRT_Star_Dist::Build_RRT_Star( RRT_Star, Pos_Init, Total_Num_Node, R_Robot, Gamma_rrt, 
		    RRT_Star_Region, Dim, Mu, Environment, Verbose);//Build RRT*
    //int Best_Node = RRT_Star_Dist::Best_Trajectory(RRT_Star, Pos_Goal, Radio_Goal);
    t1 = clock();
    //if( Best_Node != -1 ) Auxiliary:: Save_Trajectory( RRT_Star, Best_Node, Name ); 
    //RRT_Star_Dist::Save_Graph( RRT_Star, Name );//Save RRT 
    double time = ( double( t1 - t0 ) / CLOCKS_PER_SEC );
    cout << "Execution Time: " << time << endl;
    
   // ofstream fileName1;
    //fileName1.open( "Save/Execution_Information.txt" );
    //fileName1<<Name<<endl;
    //fileName1.close();
    
    return 0;
} 
