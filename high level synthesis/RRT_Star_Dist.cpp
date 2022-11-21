#include "RRT_Star_Dist.h"


    //( Node_RRT RRT_Star[Node], Position_Holonomic &Pos_Init,
   // int Total_Num_Node, float R_Robot, float Gamma_rrt, Region &RRT_Star_Region,
    //float d, float mu, Polygon_2 &Environment)
     void Build_RRT_Star(hls::stream<axis_data> &dataOutStream,Initial_pos Pos_Init,int Total_Num_Node,float R_Robot,
    		float Gamma_rrt,Min_max_region min_max_region,float d,float mu,hls::stream<axis_data_inp> &envDataInputStream)
    {

#pragma HLS INTERFACE axis register both port=dataOutStream
#pragma HLS INTERFACE axis register both port=envDataInputStream
#pragma HLS INTERFACE s_axilite port=Pos_Init bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=Total_Num_Node bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=R_Robot bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=Gamma_rrt bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=min_max_region bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=d bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=mu bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL_BUS

    	Env_poly env_poly_input;
    	axis_data_inp env_poly_input_stream;
    	axis_data output_stream;

    	Polygon_2 Environment;
    	//Region RRT_Star_Region;
    	//Node_RRT RRT_Star[Node];

    for(int i=0;i<DATALENGHT;i++)
    	{

    	env_poly_input_stream=envDataInputStream.read();
    		env_poly_input.env_reg=env_poly_input_stream.data;
    		 Environment.push_back(Point_2(env_poly_input.env.x,env_poly_input.env.y));

    	}
/*    	RRT_Star_Region.x_min=min_max_region.x_min;
    	RRT_Star_Region.x_max=min_max_region.x_max;
    	RRT_Star_Region.y_min=min_max_region.y_min;
		RRT_Star_Region.y_max= min_max_region.y_max;*/

/*        std::list<int> X_near;
        std::list<int> List_pos_near;
        std::list<int> List_Path;
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

		RRT_Star[0].Position.x = Pos_Init.x;
		RRT_Star[0].Position.y = Pos_Init.y;
        RRT_Star[0].Point = Point_2( Pos_Init.x, Pos_Init.y );
        RRT_Star[0].Location_Parent = 0;*/


    }
