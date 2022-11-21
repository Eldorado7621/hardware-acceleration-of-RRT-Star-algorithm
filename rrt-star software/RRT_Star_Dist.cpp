#include "RRT_Star.hpp"

using namespace std;
class RRT_Star_Dist
{
    public: 
    ~RRT_Star_Dist()
    {
        std::cout<<"destructor RRT_Star"<<std::endl;
    }
    static void Build_RRT_Star( Node_RRT RRT_Star[10000], Position_Holonomic &Pos_Init, 
        int Total_Num_Node, float R_Robot, float Gamma_rrt, Region &RRT_Star_Region, 
        float d, float mu, Polygon_2 &Environment, bool Verbose)
    {
        if( Verbose ) cout << "Buil RRT*" <<endl;

        list<int> X_near;
        list<int> List_pos_near;
        list<int> List_Path;
        Node_RRT x_rand;
        int x_nearest;
        Node_RRT x_new;
        int x_min;
        float c_min = 0;
        float c_min_local = 0;
        float cost_tem_local = 0;
        float cost_tem = 0;
        float r_n = 0;
        float Squared_R_Robot = R_Robot*R_Robot;
        float Squared_r_n = 0;

	Point_2 Point_init( Pos_Init.x, Pos_Init.y );
        
	RRT_Star[0].Position = Pos_Init;
        RRT_Star[0].Point = Point_2( Pos_Init.x, Pos_Init.y );
        RRT_Star[0].Location_Parent = 0;
        int i = 1;
        while( i < Total_Num_Node )
        {
            x_rand = Rand_Conf( RRT_Star_Region );
            x_nearest = Nearest( x_rand, RRT_Star, i );

            
            x_new = StreamReader( RRT_Star[x_nearest], x_rand);
            if( Obstacle_Free( RRT_Star[x_nearest].Position, x_new.Position, 
			Squared_R_Robot, Environment ) )
            {
                r_n = fmin( Gamma_rrt*pow( log( i ) / i, 1.0 / d ), mu );
                X_near.clear();
                Squared_r_n = r_n*r_n;
                Near( X_near, RRT_Star, x_new, Squared_r_n, i );

                x_min = x_nearest;
                c_min_local = Cost( RRT_Star[x_nearest ].Position, x_new.Position );
                c_min = RRT_Star[x_nearest ].Cost + c_min_local;
                List_pos_near.clear();

                for( auto pos_x_near : X_near)
                {
                    cost_tem_local = Cost( RRT_Star[pos_x_near ].Position, x_new.Position );
                    cost_tem = RRT_Star[pos_x_near ].Cost + cost_tem_local;
                    if( Obstacle_Free( RRT_Star[pos_x_near ].Position, x_new.Position,
                        Squared_R_Robot, Environment))
                    {
                        List_pos_near.push_back( pos_x_near );
                        if(cost_tem < c_min)
                        {
                            x_min = pos_x_near;
                            c_min = cost_tem;
                            c_min_local = cost_tem_local;
                        }
                    }
                }
                List_pos_near.remove( x_min );

                x_new.Location_Parent = x_min;
                x_new.Cost = c_min;
                x_new.Local_Cost = c_min_local;
                x_new.Location_List = i;
                RRT_Star[x_min ].List_Children.push_back( i );
                RRT_Star[i].Copy( x_new );

                for( auto tem_pos_x_near : List_pos_near)
                {
                    cost_tem_local = Cost( RRT_Star[i].Position, 
		    	    RRT_Star[tem_pos_x_near ].Position );
                    cost_tem = RRT_Star[i].Cost + cost_tem_local;

                    if( cost_tem < RRT_Star[tem_pos_x_near ].Cost )
                    {
                        RRT_Star[RRT_Star[tem_pos_x_near ].Location_Parent ].List_Children.remove( tem_pos_x_near );
                        RRT_Star[tem_pos_x_near ].Location_Parent = i;
                        RRT_Star[i ].List_Children.push_back( tem_pos_x_near );
                        Rewire_the_tree( tem_pos_x_near, RRT_Star );
                    }
                }
                i++;
                if( i%10000 == 0 && Verbose ) cout<< "Nodo: " + to_string( i ) + ", r_n: " + to_string( r_n )<<endl;
            }
        }
    } 
    static Node_RRT Rand_Conf( Region &Range )
    {
        Node_RRT Tem_Node;
        float x = ( ( ( float ) rand() / ( RAND_MAX ) ) )*( Range.x_max - Range.x_min ) + Range.x_min;
        float y = ( ( ( float ) rand() / ( RAND_MAX ) ) )*( Range.y_max - Range.y_min ) + Range.y_min;
        Tem_Node.Position.Insert_Position( x, y );
        return Tem_Node;
    }
    static int Nearest( Node_RRT &x_rand, Node_RRT RRT_Star[10000], int N )
    {
        int Pos_Nearest_Node = 0;
        int i = 0;
        float Dist = 0;
        float Tem_Dist = 0;
        for( auto j = 0; j < N; j++)
        {
            Tem_Dist = Cal_Squared_Dist( x_rand, RRT_Star[j] );
            if( i == 0)
            {
                Dist = Tem_Dist;
                Pos_Nearest_Node = i;
            }
            else{
                if( Tem_Dist < Dist )
                {
                    Dist = Tem_Dist;
                    Pos_Nearest_Node = i;
                }
            }
            i++;
        }
        return Pos_Nearest_Node;
    }
    static float Cal_Dist( Node_RRT &_A, Node_RRT &_B )
    {
        float D = 0;
        Point_2 A( _A.Position.x, _A.Position.y );
        Point_2 B( _B.Position.x, _B.Position.y );
        D = sqrt( A.squared_distance( B ) );
        return D;
    }
    static float Cal_Squared_Dist( Node_RRT &_A, Node_RRT &_B )
    {
        float D = 0;
        Point_2 A( _A.Position.x, _A.Position.y );
        Point_2 B( _B.Position.x, _B.Position.y );
        D = A.squared_distance( B );
        return D;
    }
    static Node_RRT StreamReader( Node_RRT &x_nearest, Node_RRT &x_rand )
    {
        Node_RRT tem;
        float aux = Cal_Dist( x_nearest, x_rand );
        float rad = 1;
        float t = rad/aux;

        if( aux <= rad ) tem.Position.Insert_Position( x_rand.Position.x, x_rand.Position.y );
        else
        {
            tem.Position.x = ( x_rand.Position.x - x_nearest.Position.x )*t +
                x_nearest.Position.x;
            tem.Position.y = ( x_rand.Position.y - x_nearest.Position.y )*t +
                x_nearest.Position.y;
        }
        
        tem.Point = Point_2( tem.Position.x, tem.Position.y );
        return tem;
    }
    static bool Obstacle_Free( Position_Holonomic &A, Position_Holonomic &B, float Squared_R_Robot, 
        Polygon_2 &Geo)
    {
        bool Bool = true;
        int N = 5;
        Position_Holonomic Tem_Positon;
        float Delta_x = (A.x - B.x)/(N*1.0);
        float Delta_y = (A.y - B.y)/(N*1.0);

        for( int i = 0; i <= N; i++)
        {
            Tem_Positon.x = Delta_x*i + B.x;
            Tem_Positon.y = Delta_y*i + B.y;
            Point_2 t( Tem_Positon.x, Tem_Positon.y );
            if( Colition( t, Squared_R_Robot, Geo ) )
            {
                Bool = false;
                return Bool;
            }
        } 
        return Bool;
    }
    static bool Colition( Point_2 &A, float Squared_R_Robot, Polygon_2 &Geo )
    {
        bool Bool = false;
        for ( auto ei :  Geo.edge )
        {
            if( ei.squared_distance( A ) < Squared_R_Robot) return true;
        }
        return Bool;
    }
    static void Near( list<int> &X_near, Node_RRT RRT_Star[10000], Node_RRT &x_new, float Squared_r_n, int N )
    {
        for( int i=0; i<N; i++ )
        {
            if( Cal_Squared_Dist( x_new, RRT_Star[i] ) < Squared_r_n ) X_near.push_back( RRT_Star[i ].Location_List );
	}
    }
    static float Cost( Position_Holonomic &A, Position_Holonomic &B )
    {
        //Fucion de costo dependiente de la distancia
        float Dist = sqrt( pow( A.x - B.x, 2 ) + pow( A.y - B.y, 2 ) );
        return Dist;
    }
    static float Cal_Angle( float Theta_A, float Theta_B )
    {
        if( Theta_A < 0 ) Theta_A = 2*M_PI + Theta_A;
        if( Theta_B < 0 ) Theta_B = 2*M_PI + Theta_B;
        float Theta_min = min( Theta_A, Theta_B );
        float Theta_max = max( Theta_A, Theta_B );

        float Theta_1 = Theta_max - Theta_min;
        float Theta_2 = 2.0*M_PI + Theta_min - Theta_max;
        float Angle = min( Theta_1, Theta_2 );

        return Angle;
    }
    static void Rewire_the_tree( int Pos_x, Node_RRT RRT_Star[10000] )
    {
	list<int> list_n;
	list_n.push_back( Pos_x );
	int n;
	float local_cost;
	while( !list_n.empty() )
	{
	    n = list_n.back();
	    list_n.pop_back();
	    local_cost = Cost( RRT_Star[RRT_Star[n ].Location_Parent ].Position, 
		    RRT_Star[n].Position );
	    RRT_Star[n].Cost = RRT_Star[RRT_Star[n].Location_Parent ].Cost + local_cost;
	    RRT_Star[n ].Local_Cost = local_cost;

	    for( auto Pos_Son_int_List : RRT_Star[n ].List_Children )
		list_n.push_back( Pos_Son_int_List );
	}
    }
    static void Save_Graph( vector<Node_RRT> &RRT_Star, string Name )
    {
        ofstream fileName1;
        ofstream fileName2;
        ofstream fileName3;
        ofstream fileName4;
        ofstream fileName5;

        fileName1.open( "Save/Solx_" + Name + ".txt" );
        fileName2.open( "Save/Soly_" + Name + ".txt" );
        fileName3.open( "Save/Nodo_" + Name + ".txt" );
        fileName4.open( "Save/Padre_" + Name + ".txt" );
        fileName5.open( "Save/Costo_" + Name + ".txt" );
        for( const auto N:RRT_Star)
        {
            fileName1 << N.Position.x <<endl;
            fileName2 << N.Position.y <<endl;
            fileName3 << N.Location_List <<endl;
            fileName4 << N.Location_Parent <<endl;
            fileName5 << N.Cost <<endl;
        }

        fileName1.close();
        fileName2.close();
        fileName3.close();
        fileName4.close();
        fileName5.close();
    }
    static void Range_search( vector<Node_RRT> &Graph, Region &Region, list<int> &List_Point )
    {
	for( int i = 0; i < Graph.size(); i++ )
	{
	    if( Region.in_region( Graph.at( i ).Position ) ) List_Point.push_back( i );
	}
    }
    static int Best_Trajectory( vector<Node_RRT> &RRT_Star, Position_Holonomic &Goal, float Radio_Goal )
    {
	list<int> X_near;
	int best = -1;
	float Tem_Cost = Constant_Max;
	float Squared_Radio_Goal = Radio_Goal*Radio_Goal;
	Region region( Goal, Radio_Goal, Squared_Radio_Goal);
	Range_search( RRT_Star, region, X_near);
	for(int i: X_near)
	{
	    if( RRT_Star.at( i ).Cost < Tem_Cost)
	    {
		Tem_Cost = RRT_Star.at( i ).Cost;
		best = i;
	    }
	}
	if( best == -1 ) cout<<"Error best"<<endl;
	return best; 
    }
};
