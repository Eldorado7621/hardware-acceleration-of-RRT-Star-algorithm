#ifndef RRT_H
    #define RRT_H
    #include <iostream>
    #include <string>
    #include <list>
    #include <cmath>
    #include "Aux_Lib.hpp"
	#include "sdCard.h"

	#define Node 10000


    class Position_Holonomic
    {
        public:
            float x;
            float y;
            Position_Holonomic()//builder
            {
                x = 0;
                y = 0;
            }
            Position_Holonomic(float _x, float _y)//builder
            {
                x = _x;
                y = _y;
            }
            ~Position_Holonomic()//destroyer
            {
                //cout<< "Destructor Position_Holonomic"<<endl;
            }
            void Insert_Position(float _x, float _y)
            {
                x = _x;
                y = _y;
            }
            void Copy(Position_Holonomic &A)
            {
                x = A.x;
                y = A.y;
            }
    };
    class Node_RRT
    {
        public:
            int Location_Parent;//Location of the parent in the Graph
            int Location_List;//Location of the Node in the Graph
            Position_Holonomic Position;//Position of the Node
            Point_2 Point;
            float Cost;//Cost from root to node
            float Local_Cost;//Cost from parent to node

            //std::list<int> List_Children;//Node's children list

            short int Llist_Children[70]={0};//Node's children list
            short int Children_Count;


            Node_RRT()//builder
            {
                Location_Parent = 0;
                Location_List = 0;
                Cost = 0;
                Local_Cost = 0;
                Children_Count=0;
            }
            ~Node_RRT()//destroyer
            {
                //cout<<"Destructor Node_RRT"<<endl;
            }
            void Copy(Node_RRT &A)//Copy Node
            {
                Location_Parent = A.Location_Parent;
                Location_List = A.Location_List;
                Position.Copy(A.Position);
                Point = A.Point;
                Cost = A.Cost;
                Local_Cost = A.Local_Cost;

                //List_Children.splice(List_Children.begin(), A.List_Children);

               Children_Count=A.Children_Count;

                for(int i=0;i<A.Children_Count;i++)
		        {
		        	Llist_Children[i]=A.Llist_Children[i];
		        }
            }
    };
    class Region//this class is used by the kd-tree structure
    {
        public:
            Position_Holonomic Center_Position;
            float x_min;
            float x_max;
            float y_min;
            float y_max;
            float Squared_r_n;
            float r_n;
            ~Region(){};
            Region()
            {
                x_min = 0;
                x_max = 0;
                y_min = 0;
                y_max = 0;
                r_n = 0;
                Squared_r_n = 0;
            };
            //Algoritms
            Region( Position_Holonomic &Center_Position_, float r_n_, float Squared_r_n_, Polygon_2 &Environment)
            {
                r_n = r_n_;
                Center_Position.x = Center_Position_.x;
                Center_Position.y = Center_Position_.y;

                x_min = Center_Position.x - r_n;
                x_max = Center_Position.x + r_n;
                y_min = Center_Position.y - r_n;
                y_max = Center_Position.y + r_n;
                Squared_r_n = Squared_r_n_;

		x_min = y_min = Constant_Max;
		x_max = y_max = Constant_Min;
		for( auto const vi : Environment.vertex )
		{
		    if( vi.x() < x_min ) x_min = vi.x();
		    if( vi.x() > x_max ) x_max = vi.y();
		    if( vi.y() < y_min ) y_min = vi.y();
		    if( vi.y() > y_max ) y_max = vi.y();
		}
            };
            //RRT
            Region( Position_Holonomic &Center_Position_, float r_n_, float Squared_r_n_)
            {
                r_n = r_n_;
                Center_Position.x = Center_Position_.x;
                Center_Position.y = Center_Position_.y;
                x_min = Center_Position.x - r_n;
                x_max = Center_Position.x + r_n;
                y_min = Center_Position.y - r_n;
                y_max = Center_Position.y + r_n;
                Squared_r_n = Squared_r_n_;
            };
            Region( float x_min_, float x_max_, float y_min_, float y_max_ )
            {
                x_min = x_min_;
                x_max = x_max_;
                y_min = y_min_;
                y_max = y_max_;
            };
	    Region( Polygon_2 &Environment, float r_n_ )
	    {
		r_n = r_n_;
		Squared_r_n = r_n_*r_n_;
		x_min = Constant_Max;
		x_max = Constant_Min;
		y_min = Constant_Max;
		y_max = Constant_Min;
		for( auto const vi:Environment.vertex )
		{
		    if( vi.x() < x_min ) x_min = vi.x();
		    if( vi.x() > x_max ) x_max = vi.x();
		    if( vi.y() < y_min ) y_min = vi.y();
		    if( vi.y() > y_max ) y_max = vi.y();
		}
	    };
            bool in_region( Position_Holonomic &Pos )
            {
		if( pow( Pos.x - Center_Position.x, 2) + pow( Pos.y - Center_Position.y, 2 ) < Squared_r_n )
                    return true;
		else return false;
            };
    };
#endif
