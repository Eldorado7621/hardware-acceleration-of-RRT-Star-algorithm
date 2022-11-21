#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <hls_stream.h>



typedef ap_axis<256,2,5,6> axis_data;
typedef ap_axis<64,2,5,6> axis_data_inp;

typedef struct{
	float x;
	float y;
}Env;


typedef union{
	Env env;
	int64_t env_reg;
}Env_poly;

typedef struct{
	float x;
	float y;
}Initial_pos;

typedef struct{
	float x_min;
	float x_max;
	float y_min;
	float y_max;
}Min_max_region;

typedef struct{
	float x;
	float y;

}Posn;
typedef struct{
	float x;
	float y;

}Pt;

typedef struct{
	Posn posn;
	Pt pt;
	int loc_parnt;
	float loc_cost;
	float cost;
	int loc_list;

}Struct_RRT_Star;

typedef union{
	Struct_RRT_Star struct_RRT_Star;

	int64_t struct_RRT_Star_reg;
}Union_RRT_Star;
