 #include "RRT_Star.hpp"

using namespace std;
class Auxiliary
{
    public:
    static void Read_data( Position_Holonomic &Pos_Init, Position_Holonomic &Pos_Goal,
	    int &Total_Num_Node, float &Radio_Robot, float &Radio_Goal,
	    float &Gamma_RRT, float &Mu, Polygon_2 &Geo, string Name)
    {
        ifstream fileName1;
        fileName1.open( "Information/Information_"+Name + ".txt" );
        string cadena = "";
        int i = 1;
        int Num_H = 0;
        list<float>x;
        list<float>y;
        while( !fileName1.eof() ) 
        {
            fileName1 >> cadena;
            if( i == 2 ) Pos_Init.x = stof( cadena );
            if( i == 3 ) Pos_Init.y = stof( cadena );
            if( i == 5 ) Pos_Goal.x = stof( cadena );
	    if( i == 6 ) Pos_Goal.y = stof( cadena );
	    if( i == 8 ) Total_Num_Node = stoi( cadena );
	    if( i == 10 ) Radio_Robot = stof( cadena );
	    if( i == 12 ) Radio_Goal = stof( cadena );
	    if( i == 14 ) Gamma_RRT = stof( cadena );
	    if( i == 16 ) Mu = stof( cadena );
            if( i == 18 ) Num_H = stoi(cadena);
            i++;
        }
        fileName1.close();
        fileName1.open( "Information/Information_Pol_x_" + Name + ".txt" );
        i = 1;
        while( !fileName1.eof() ) 
        {
            fileName1 >> cadena;
            if( i > 1 ) x.push_back( stof( cadena ) );
            i++;
        }
        fileName1.close();
        fileName1.open( "Information/Information_Pol_y_" + Name + ".txt" );
        i = 1;
        while( !fileName1.eof() ) 
        {
            fileName1 >> cadena;
            if( i > 1 ) y.push_back( stof( cadena ) );
            i++;
        }
        fileName1.close();
        x.pop_back();
        y.pop_back();
        Polygon_2 polygon;
        if( x.size() == y.size() )
        {
            list<float>::iterator i_x = x.begin();
            list<float>::iterator i_y = y.begin();
            while( i_x != x.end() )
            {
                polygon.push_back( Point_2( *i_x, *i_y ) );
                i_x++;
                i_y++;
            }
            //if( polygon.is_clockwise_oriented() ) polygon.reverse_orientation();
        }
        else 
	    cout<<"Error 1"<<endl;
    
	Geo = polygon;
    }
    static void Save_Trajectory( vector<Node_RRT> &G, int i, string Name )
    {
        ofstream fileName1;
        ofstream fileName2;

        fileName1.open( "Save/Trajectory_x_" + Name + ".txt" );
        fileName2.open( "Save/Trajectory_y_" + Name + ".txt" );
        while( i != 0 )
        {
            fileName1 << G.at( i ).Position.x<<endl;
            fileName2 << G.at( i ).Position.y<<endl;
            i = G.at( i ).Location_Parent;
        }
        fileName1 << G.at( i ).Position.x<<endl;
        fileName2 << G.at( i ).Position.y<<endl;

        fileName1.close();
        fileName2.close();
    }
}; 

