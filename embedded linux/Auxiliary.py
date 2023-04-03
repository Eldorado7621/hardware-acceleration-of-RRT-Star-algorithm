from RRT_Star import *

class Auxiliary:
    @staticmethod
    def Read_data(Pos_Init, Pos_Goal, Total_Num_Node, Radio_Robot, Radio_Goal, Gamma_RRT, Mu, Geo, Name):
        with open("Information/Information_" + Name + ".txt") as fileName1:
            
            cadena = ""
            i = 1
            Num_H = 0
            x = []
            y = []
            for line in fileName1:
                cadena = line.strip()
                if i == 2:
                    Pos_Init[0].x = float(cadena)
                elif i == 3:
                    Pos_Init[0].y = float(cadena)
                elif i == 5:
                    Pos_Goal[0].x = float(cadena)
                elif i == 6:
                    Pos_Goal[0].y = float(cadena)
                elif i == 8:
                    Total_Num_Node[0] = int(cadena)
                elif i == 10:
                    Radio_Robot[0] = float(cadena)
                elif i == 12:
                    Radio_Goal[0] = float(cadena)
                elif i == 14:
                    Gamma_RRT[0] = float(cadena)
                elif i == 16:
                    Mu[0] = float(cadena)
                elif i == 18:
                    Num_H = int(cadena)
                i += 1
           

        fileName1.close()

        with open("Information/Information_Pol_x_" + Name + ".txt") as fileName1:
            i = 1
            for line in fileName1:
                cadena = line.strip()
                if i > 1:
                    x.append(float(cadena))
                i += 1

        fileName1.close()

        with open("Information/Information_Pol_y_" + Name + ".txt") as fileName1:
            i = 1
            for line in fileName1:
                cadena = line.strip()
                if i > 1:
                    y.append(float(cadena))
                i += 1

        fileName1.close()

        x.pop()
        y.pop()
        
        polygon = Polygon_2()
       
     
        if len(x) == len(y):
            i_x = iter(x)
            i_y = iter(y)
            for _ in range(len(x)):
                polygon.push_back(Point_2(next(i_x), next(i_y)))
            #if polygon.is_clockwise_oriented() : polygon.reverse_orientation()
        else:
            print("Error 1")

        Geo[0] = polygon
        
        print("zuio",Geo[0].edge[0].AABB)
        
        
    @staticmethod
    def Op_Read_data(Name):
        
        Total_Num_Node = 0 # Number of nodes in the RRT*
        R_Robot =0  # Robot's radio
        Radio_Goal = 0  # Radio goal
        Gamma_rrt =0  # Constant Gamma
       
        Mu = 0  # Constant Mu
        Pos_Init = Position_Holonomic()  # Initial position of robot
        Pos_Goal = Position_Holonomic()  # Position of Goal
        
        
        with open("Information/Information_" + Name + ".txt") as fileName1:
            
            cadena = ""
            i = 1
            Num_H = 0
            x = []
            y = []
            for line in fileName1:
                cadena = line.strip()
                if i == 2:
                    Pos_Init.x = float(cadena)
                elif i == 3:
                    Pos_Init.y = float(cadena)
                elif i == 5:
                    Pos_Goal.x = float(cadena)
                elif i == 6:
                    Pos_Goal.y = float(cadena)
                elif i == 8:
                    Total_Num_Node = int(cadena)
                elif i == 10:
                    Radio_Robot = float(cadena)
                elif i == 12:
                    Radio_Goal = float(cadena)
                elif i == 14:
                    Gamma_RRT = float(cadena)
                elif i == 16:
                    Mu = float(cadena)
                elif i == 18:
                    Num_H = int(cadena)
                i += 1
           

        fileName1.close()

        with open("Information/Information_Pol_x_" + Name + ".txt") as fileName1:
            i = 1
            for line in fileName1:
                cadena = line.strip()
                if i > 1:
                    x.append(float(cadena))
                i += 1

        fileName1.close()

        with open("Information/Information_Pol_y_" + Name + ".txt") as fileName1:
            i = 1
            for line in fileName1:
                cadena = line.strip()
                if i > 1:
                    y.append(float(cadena))
                i += 1

        fileName1.close()

        x.pop()
        y.pop()
        
        polygon = Polygon_2()
       
     
        if len(x) == len(y):
            i_x = iter(x)
            i_y = iter(y)
            for _ in range(len(x)):
                polygon.push_back(Point_2(next(i_x), next(i_y)))
            #if polygon.is_clockwise_oriented() : polygon.reverse_orientation()
        else:
            print("Error 1")
        
        return Pos_Init, Pos_Goal, Total_Num_Node, Radio_Robot, Radio_Goal, Gamma_RRT, Mu, polygon 
        

    @staticmethod
    def Save_Trajectory(G, i, Name):
        with open("Save/Trajectory_x_" + Name + ".txt", "w") as fileName1, \
                open("Save/Trajectory_y_" + Name + ".txt", "w") as fileName2:
            while i != 0:
                fileName1.write(str(G[i].Position.x) + "\n")
                fileName2.write(str(G[i].Position.y) + "\n")
                i = G[i].Location_Parent
            fileName1.write(str(G[i].Position.x) + "\n")
            fileName2.write(str(G[i].Position.y) + "\n")

        fileName1.close()
        fileName2.close()
        
        
    def read_xrand(x_rand):
        with open("Information/XRANDX.txt") as fileName1:
            line_buf = ""
            i=0
            
            for line in fileName1:
                line_buf=line.strip()
                x_rand[i].Position.x=float(line_buf)
                i=i+1
            fileName1.close()
        
        with open("Information/XRANDY.txt") as fileName2:
            line_buf = ""
            i=0
            
            for line in fileName2:
                line_buf=line.strip()                
                x_rand[i].Position.y=float(line_buf)

                i=i+1
            fileName2.close()
        del line_buf
       
        
        