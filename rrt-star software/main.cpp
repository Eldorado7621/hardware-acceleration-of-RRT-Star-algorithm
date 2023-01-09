#include "RRT_Star_Dist.cpp"
#include "Auxiliary.cpp"
#include "xil_printf.h"

#include <xtime_l.h>
#include <time.h>

FIL* fptr;


char dataBuffer[32];
char *dataPntr=dataBuffer;

int main()
{

	  //-----------------BEGIN -------------------example of how to write to sd card-----
/*	float test_data[10]={1.02,2.02,3.09,4.09,3.04,5.09,5.90,6.90,8.98,9.89};

	int Status;
	Status = SD_Init();
	if(Status != XST_SUCCESS)
		xil_printf("SD card init failed");

	fptr = openFile("fft.txt",'w');

	if(fptr == 0)
		printf("File opening failed\n\r");

	    for (int i=0;i<10;i++)
	    {
	    	tempdata=test_data[i];
		sprintf(dataBuffer,"%0.4f\n",test_data[i]);
		//dataPntr = dataPntr+8;

		//write the temp data from the buffer to the SD card after every 10 iterations

			xil_printf("Updating SD card...\n\r");
			writeFile(fptr, strlen(dataBuffer),(u32) dataBuffer);
			//dataPntr = (char *)dataBuffer;

	    }
	    closeFile(fptr);
	    SD_Eject();*/

	   //____________END -------------------example of how to write to sd card-------------------------------------


	XTime sw_processor_start, sw_processor_end;
    cout << "Main" <<endl;
    srand ( 0 );
    int Total_Num_Node;//Number of nodes in the RRT*
    float R_Robot;//Robot's radio
    float Radio_Goal;//Radio goal
    float Gamma_rrt;//Constant Gamma
    float Dim = 2;//Dimension of space
    float Mu;//Constant Mu
    Position_Holonomic Pos_Init; //Initial position of robot
    Position_Holonomic Pos_Goal; //Position of Goal
    Polygon_2 Environment; //Environment
    Node_RRT RRT_Star[Node];

    string Name = "1"; //Select Environment
    cout<<"Environment: "<<Name<<endl;
    bool Verbose = true;

    Auxiliary::Read_data( Pos_Init, Pos_Goal, Total_Num_Node, R_Robot, Radio_Goal,
	    Gamma_rrt, Mu, Environment, Name );
    Total_Num_Node=Node;
    Region RRT_Star_Region( Environment, R_Robot );

/*    for (int i = 0; i < 5; i++)
          cout << rand() << " ";*/

    XTime_GetTime(&sw_processor_start);
    RRT_Star_Dist::Build_RRT_Star( RRT_Star, Pos_Init, Total_Num_Node, R_Robot, Gamma_rrt,
    		    RRT_Star_Region, Dim, Mu, Environment, Verbose);//Build RRT*


    	//cout<<"ffi:"<<1<<endl;

     //int Best_Node = RRT_Star_Dist::Best_Trajectory(RRT_Star, Pos_Goal, Radio_Goal);
     XTime_GetTime(&sw_processor_end);
    // cout<<"best node: "<<Best_Node<<endl;

    /*if( Best_Node != -1 ) Auxiliary:: Save_Trajectory( RRT_Star, Best_Node, Name );
    RRT_Star_Dist::Save_Graph( RRT_Star, Name );//Save RRT*/
    float sw_processing_time=1000000.0*(sw_processor_end-sw_processor_start)/(COUNTS_PER_SECOND);
//    //double time = ( double( t1 - t0 ) / CLOCKS_PER_SEC );
    cout << "Execution Time: " << sw_processing_time << endl;

	//write the temp data to the data buffer
    int Status;
    Status=SD_Eject();
    if(Status != XST_SUCCESS)
    		xil_printf("SD card Eject failed\n");
    else
    	xil_printf("SD card can be safely removed\n");
    cout << "hhh";


//
//   /* ofstream fileName1;
//    fileName1.open( "Save/Execution_Information.txt" );
//    fileName1<<Name<<endl;
//    fileName1.close();*/

    return 0;
}

