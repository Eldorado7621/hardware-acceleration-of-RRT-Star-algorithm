import math
import random
from RRT_Star import *
Node=10000
SIZE_XNEAR=1000
from pynq import MMIO
import numpy as np
from pynq import Overlay
from pynq import allocate
import sys

ol=Overlay("rrt_pynq08.bit")
    #ol.download() # flash the FPGA


    #load dma
dma=ol.axi_dma_0
hls_ip=ol.rrt_0

class RRT_Star_Dist:
    
    def Rand_Conf(self,Range):
        Tem_Node = Node_RRT()
        x = random.uniform(0, 1) * (Range.x_max - Range.x_min) + Range.x_min
        y = random.uniform(0, 1) * (Range.y_max - Range.y_min) + Range.y_min
        Tem_Node.Position.Insert_Position(x, y)
        return Tem_Node
    
    def Cal_Squared_Distc(self,_A, _B):
        A = Point_2(_A[0].Position.x, _A[0].Position.y)
        B = Point_2(_B["position_x"], _B["position_y"])
        #D = (A.x - B.x)**2 + (A.y - B.y)**2
        D = A.squared_distance( B )
        return D
    def Cal_Squared_Dist(self,_A, _B):
        A = Point_2(_A["position_x"],  _A["position_y"])
        B = Point_2(_B["position_x"],  _B["position_y"])
        #D = (A.x - B.x)**2 + (A.y - B.y)**2
        D = A.squared_distance( B )
        return D

    def Nearest(self,x_rand, RRT_Star, N):
        Pos_Nearest_Node = 0
        i = 0
        Dist = 0
        Tem_Dist = 0
        for j in range(N):
            Tem_Dist = self.Cal_Squared_Dist(x_rand, RRT_Star[j])
            if i == 0:
                Dist = Tem_Dist
                Pos_Nearest_Node = i
            else:
                if Tem_Dist < Dist:
                    Dist = Tem_Dist
                    Pos_Nearest_Node = i
            i += 1
        return Pos_Nearest_Node




    def Cal_Dist(self,_A, _B):
       
        A = Point_2(_A["position_x"],  _A["position_y"])
        
        B = Point_2(_B["position_x"],  _B["position_y"])
     
        D = math.sqrt(A.squared_distance(B))
        #print("cd",D)
        return D
    


    def StreamReader(self,x_nearest, x_rand):
        
        tem = Node_RRT_hp()
        aux = self.Cal_Dist(x_nearest, x_rand)
        if aux==0:
            aux=1
        rad = 1
        t = rad / aux
       
        if aux <= rad:            
            tem.Position.Insert_Position( x_rand["position_x"], x_rand["position_y"] )
           
            
        else:
            #print("pi",i[0])
            tem.Position.x = (x_rand["position_x"] - x_nearest["position_x"]) * t + x_nearest["position_x"]
            tem.Position.y = (x_rand["position_y"] - x_nearest["position_y"]) * t + x_nearest["position_y"]
           
        tem.Point = Point_2(tem.Position.x, tem.Position.y)
        
        return tem
    
    def Obstacle_Free(self,A, B, Squared_R_Robot, Geo):
        Bool = True
        N = 5
        Tem_Position = Position_Holonomic()
        Delta_x = (A.x - B.x) / (N * 1.0)
        Delta_y = (A.y - B.y) / (N * 1.0)
        
        #if nn<100:
        #    print(nn,A.x,B.x)

        for i in range(0,N+1,1):
            Tem_Position.x = Delta_x * i + B.x
            Tem_Position.y = Delta_y * i + B.y
            t = Point_2(Tem_Position.x, Tem_Position.y)
            
            if self.Colition(t, Squared_R_Robot, Geo):               
                Bool = False
                return Bool
        return Bool
    
    def Colition(self,a, squared_r_robot, geo):
        
        for ei in geo[0].edges:
            
            eii=ei.squared_distance(a)
            
            if eii < squared_r_robot:
                return True
        return False

    def Near(self,x_near, rrt_star, x_new, squared_r_n, n):
       
        x_near_count=0
    
        
        #print("sqr",squared_r_n)
        for i in range(n):                        
            if self.Cal_Squared_Distc(x_new, rrt_star[i]) < squared_r_n:
                x_near[x_near_count] = rrt_star[i]["Location_List"]
                x_near_count += 1
        return x_near_count
        

    def Cost(self,a, b):
        dist = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
        return dist
    
    def Cal_Angle(self,Theta_A, Theta_B):
        if Theta_A < 0:
            Theta_A = 2 * math.pi + Theta_A
        if Theta_B < 0:
            Theta_B = 2 * math.pi + Theta_B
        Theta_min = min(Theta_A, Theta_B)
        Theta_max = max(Theta_A, Theta_B)

        Theta_1 = Theta_max - Theta_min
        Theta_2 = 2.0 * math.pi + Theta_min - Theta_max
        Angle = min(Theta_1, Theta_2)

        return Angle
    
    def RRT_SW(self,x_new,RRT_Star,X_near,i,xx_rand,Squared_r_n,Squared_R_Robot,Environment):
        x_nearest = self.Nearest(xx_rand, RRT_Star, i)        
        c_min_local=0.0
        X_near_count=0

        RRT_Starhp_copy = Node_RRT_hp()
        #x_new = self.StreamReader(RRT_Star[x_nearest], x_rand[xrand_index],pi)
        x_new[0] = self.StreamReader(RRT_Star[x_nearest], xx_rand)
        RRT_Starhp_copy.Position.x=RRT_Star[x_nearest]["position_x"]
        RRT_Starhp_copy.Position.y=RRT_Star[x_nearest]["position_y"]
        obstacle_free=self.Obstacle_Free(RRT_Starhp_copy.Position, x_new[0].Position, Squared_R_Robot, Environment)
           
        if obstacle_free:
            X_near_count=self.Near(X_near, RRT_Star, x_new, Squared_r_n, i)
            c_min_local = self.Cost(RRT_Starhp_copy.Position, x_new[0].Position)
            
        return obstacle_free,x_nearest,c_min_local,X_near_count
    
    def RRT_HW(self,x_new,in_buffer,X_near,i,xx_rand,Squared_r_n,Squared_R_Robot):
        import struct
        
        rrt_ip = MMIO(0x00A0010000, 0x10000) # (IP_BASE_ADDRESS, ADDRESS_RANGE), 
        c_min_local=0.0
        X_near_count=0
        
        #RRT_Star=np.asarray(RRT_Star)
    
        #RRT_Star= allocate(shape=(1,), dtype=RRT_Star)

        '''rrt_ip.write(0x18,xx_randn.physical_address ) # write xrand
        rrt_ip.write(0x20, i) # write i
        rrt_ip.write(0x28, sqrn.physical_address) # write Squared_r_n
        rrt_ip.write(0x30, sqrrbt.physical_address) # write Squared_R_Robot
        rrt_ip.write(0x00, 0x81)# set ap_start to 1 which initiates the process we wrote to the fabric'''
        
        
        
        
      
        
        '''xx_randnx = allocate(shape=(1,), dtype=np.float32)
        xx_randny = allocate(shape=(1,), dtype=np.float32)
        sqrn = allocate(shape=(1,), dtype=np.float32)
        sqrrbt=allocate(shape=(1,), dtype=np.float32)
        
        np.copyto(xx_randnx, xx_rand["position_x"])
        np.copyto(xx_randny, xx_rand["position_y"])
        np.copyto(sqrn, Squared_r_n)
        np.copyto(sqrrbt,Squared_R_Robot)
        
        hls_ip.write(0x18,xx_randnx.physical_address) # write xrand
        hls_ip.write(0x20, xx_randny.physical_address) # write i
        hls_ip.write(0x28, i) # write i
        hls_ip.write(0x30, sqrn.physical_address) # write Squared_Rn
        hls_ip.write(0x38, sqrrbt.physical_address) # write Squared_Robot'''
        
        
        sqrrbt_bytes=bytes(struct.pack("f",Squared_R_Robot))
        sqrn_bytes=bytes(struct.pack("f",Squared_r_n))
        xx_randx_bytes=bytes(struct.pack("f",xx_rand["position_x"]))
        xx_randy_bytes=bytes(struct.pack("f",xx_rand["position_y"]))
        
        
        hls_ip.write(0x18,xx_randx_bytes ) # write xrand
        hls_ip.write(0x20, xx_randy_bytes) # write i
        hls_ip.write(0x28, i) # write i
        hls_ip.write(0x30, sqrn_bytes) # write Squared_Rn
        hls_ip.write(0x38, sqrrbt_bytes) # write Squared_Robot
        
        ss=struct.unpack('f', struct.pack('I', hls_ip.read(0x18)))[0]
        yy=struct.unpack('f', struct.pack('I', hls_ip.read(0x20)))[0]
        mm=struct.unpack('f', struct.pack('I', hls_ip.read(0x38)))[0]
        nn=struct.unpack('f', struct.pack('I', hls_ip.read(0x30)))[0]
        
        
        #print("HH", ss, yy,mm,nn)
        
        
        '''hls_ip.write(0x18,xx_rand ) # write xrand
        hls_ip.write(0x20, i) # write i
        hls_ip.write(0x28, sqrn.physical_address) # write Squared_r_n
        hls_ip.write(0x30, Squared_R_Robot[0]) # write Squared_R_Robot'''
        
        CONTROL_REGISTER=0x00
        hls_ip.write(CONTROL_REGISTER,0x81)#start the IP
        
        dma.sendchannel.transfer(in_buffer[0:i])
        
        #dma.sendchannel.wait()
        dma.recvchannel.transfer(X_near)
        #dma.sendchannel.wait()
        #dma.recvchannel.wait()
        
        #while(hls_ip.read(0x0c)):#check if IP is done
            
            #xpos_hw=[0]*5
        x_min=struct.unpack('f', struct.pack('I', hls_ip.read(0x40)))[0]
        obstacle_free=hls_ip.read(0x10) #ap_return
        #print("obstacle_free",obstacle_free,x_min)
           
        if(obstacle_free):
            c_min_local=struct.unpack('f', struct.pack('I', hls_ip.read(0x44)))[0]
            x_new[0].Position.x=struct.unpack('f', struct.pack('I', hls_ip.read(0x48)))[0]
            x_new[0].Position.y=struct.unpack('f', struct.pack('I', hls_ip.read(0x4C)))[0]
            X_near_count=np.int32(struct.unpack('f', struct.pack('I', hls_ip.read(0x50)))[0])
            #print("cm",c_min_local,x_new[0].Position.x,x_new[0].Position.y,X_near_count)

        return obstacle_free,np.int32(x_min),c_min_local,X_near_count
        
              
    
    def ARewire_The_Tree(self,Pos_x, RRT_Star):
        list_n = [0] * 30
        count_list = 0
        list_n[0] = Pos_x
        count_list += 1
        n = 0
        local_cost = 0.0
    
        while count_list != 0:
            n = list_n[count_list-1]
            list_n[count_list-1] = 0
            count_list -= 1
            local_cost = self.Cost(RRT_Star[RRT_Star[n].Location_Parent].Position, RRT_Star[n].Position)
            RRT_Star[n].Cost = RRT_Star[RRT_Star[n].Location_Parent].Cost + local_cost
            RRT_Star[n].Local_Cost = local_cost
        
            for i in range(RRT_Star[n].Children_Count):                
                list_n[count_list] = RRT_Star[n].Llist_Children[i]
                count_list += 1

    def Build_RRT_Star(self,RRT_Star, RRT_Starhp,Pos_Init, Total_Num_Node, R_Robot, Gamma_rrt, RRT_Star_Region, d, mu, Environment,x_rand,X_near,Verbose):
        

        #if Verbose:
            #print("Build RRT*")
                
        '''Environment = [Polygon_3()]
        Environment[0].edges[0].Source=[0]
        Environment[0].edges[1].Source=[0]
        Environment[0].edges[2].Source=[0]
        Environment[0].edges[3].Source=[0]'''
        
        Environment[0].edges[0].Source=Point_2(-10,10)
        Environment[0].edges[0].Target=Point_2(10,10)
        
        Environment[0].edges[1].Source=Point_2(10,10)
        Environment[0].edges[1].Target=Point_2(10,-10)
        
        Environment[0].edges[2].Source=Point_2(10,-10)
        Environment[0].edges[2].Target=Point_2(-10,-10)
        
        Environment[0].edges[3].Source=Point_2(-10,-10)
        Environment[0].edges[3].Target=Point_2(-10,10)
        
        for i in range(4):
            A = ( Environment[0].edges[i].Target.y() - Environment[0].edges[i].Source.y() )
            B = ( Environment[0].edges[i].Target.x() - Environment[0].edges[i].Source.x() )
            Environment[0].edges[i].AABB = A*A + B*B

       
        
        List_pos_near = [0 for _ in range(70)]
       
        
        x_nearest = 0
        x_new_pos = [Node_RRT_fp()]
        RRT_Starhp_copy = Node_RRT_hp()
        x_new = [Node_RRTop()]
        
        c_min = 0.0
        
        cost_tem_local = 0.0
        cost_tem = 0.0
        r_n = 0.0
        Squared_R_Robot = R_Robot * R_Robot
        
        #Point_init = Point_2(Pos_Init.x, Pos_Init.y)
        RRT_Starhp[0]["position_x"] = Pos_Init.x
        RRT_Starhp[0]["position_y"] = Pos_Init.y
        #print("hh",RRT_Starhp[0]["position_x"],RRT_Starhp[0]["position_y"])
        
        RRT_Star[0].Point = Point_2(Pos_Init.x, Pos_Init.y)
        RRT_Star[0].Location_Parent = 0
        i = 1
        xrand_index=1
        
        #allocate memory for the generated random nodes
        kernel_rand_node = np.dtype([('position_x', np.float32), ('position_y', np.float32)])
        xx_rand_hp = allocate(shape=(1,), dtype=kernel_rand_node)
        #xx_rand_hp=Node_Rand()
        
        #print("gg",RRT_Starhp[0].Position.x )
        #in_buffer[0]=(RRT_Starhp[0].Position.x,RRT_Starhp[0].Position.y,RRT_Starhp[0].Location_List,RRT_Starhp[0].Cost)

        #children_list_index = 0
        #f = open("demofile3.txt", "w")
        
        while i < Node:   
                       
            xx_rand = self.Rand_Conf(RRT_Star_Region)
            #xx_rand_hp.Position.x=np.float16(xx_rand.Position.x)
            #xx_rand_hp.Position.y=np.float16(xx_rand.Position.y)
            
            xx_rand_hp["position_x"]=xx_rand.Position.x
            xx_rand_hp["position_y"]=xx_rand.Position.y
            
            r_n = min(Gamma_rrt * pow(math.log(i) / i, 1.0 / d), mu)
            #print("buHH", r_n,Gamma_rrt,d,mu,i)
            Squared_r_n = r_n * r_n  
            #print("buHH", Squared_r_n)
            #Squared_r_n=Squared_r_n.astype(np.float16)
            
            obstacle_free,x_min,c_min_local,X_near_count=self.RRT_SW(x_new_pos,RRT_Starhp,X_near,i,xx_rand_hp,Squared_r_n, Squared_R_Robot, Environment)
            #obstacle_free,x_min,c_min_local,X_near_count = self.RRT_HW(x_new_pos,RRT_Starhp,X_near,i,xx_rand_hp, Squared_r_n,Squared_R_Robot)
            
                                  
            print(i)    
            if obstacle_free:      
                                             
                c_min = RRT_Starhp[x_min]["Cost"]+ c_min_local
                List_pos_near_index = 0
                for xnear_index in range(X_near_count):
                    pos_x_near = X_near[xnear_index]
                    RRT_Starhp_copy.Position.x=RRT_Starhp[pos_x_near]["position_x"]
                    RRT_Starhp_copy.Position.y=RRT_Starhp[pos_x_near]["position_y"]
                    cost_tem_local = self.Cost(RRT_Starhp_copy.Position, x_new_pos[0].Position)
                    cost_tem = RRT_Starhp[pos_x_near]["Cost"] + cost_tem_local
                    if self.Obstacle_Free(RRT_Starhp_copy.Position, x_new_pos[0].Position, Squared_R_Robot, Environment):
                     
                        List_pos_near[List_pos_near_index] = pos_x_near
                        List_pos_near_index += 1
                        if cost_tem < c_min:
                            x_min = pos_x_near
                            c_min = cost_tem
                            c_min_local = cost_tem_local
                            # List_pos_near.remove( x_min );
                ctt = 0
            
                while ctt < List_pos_near_index:
                    if List_pos_near[ctt] == x_min:
                        cttt = ctt
                        while cttt < List_pos_near_index:
                            List_pos_near[cttt] = List_pos_near[cttt + 1]
                            cttt += 1
                        List_pos_near_index -= 1
                    ctt += 1

                x_new[0].Location_Parent = x_min
                
                x_new[0].Local_Cost = c_min_local
                
               
                x_new_pos[0].Location_List = i

                index = RRT_Star[x_min].Children_Count
                
                #print("xmin",x_min,index)
                RRT_Star[x_min].Llist_Children[index] = i
                RRT_Star[x_min].Children_Count += 1
                #RRT_Star[i].Location_Parent=x_min
                #RRT_Star[i].Local_Cost=c_min_local
                
                RRT_Star[i].Copy(x_new[0])
                
                
                RRT_Starhp[i]["position_x"]=x_new_pos[0].Position.x
                RRT_Starhp[i]["position_y"]=x_new_pos[0].Position.y
                RRT_Starhp[i]["Location_List"]=i
                RRT_Starhp[i]["Cost"]=c_min
                
                #in_buffer[i]=(RRT_Starhp[i].Position.x,RRT_Starhp[i].Position.y,RRT_Starhp[i].Location_List,RRT_Starhp[i].Cost)
                #print("dd",RRT_Starhpp[i])

                #RRT_Star[i].Copy(x_new[0])
                
                #print("rrt",i,RRT_Star[i].Position.x,RRT_Star[i].Position.y)

                for list_pos_ele_ind in range(List_pos_near_index):
                    tem_pos_x_near = List_pos_near[list_pos_ele_ind]
                    
                    RRT_Starhp_copy.Position.x=RRT_Starhp[tem_pos_x_near]["position_x"]
                    RRT_Starhp_copy.Position.y=RRT_Starhp[tem_pos_x_near]["position_y"]
                    
                    x_new_pos[0].Position.x=RRT_Starhp[i]["position_x"]
                    x_new_pos[0].Position.y=RRT_Starhp[i]["position_y"]
                    
                    cost_tem_local = self.Cost(x_new_pos[0].Position, RRT_Starhp_copy.Position)
                    cost_tem = RRT_Starhp[i]["Cost"] + cost_tem_local
    
                    if cost_tem < RRT_Starhp[tem_pos_x_near]["Cost"]:
                        ctt = 0
        
                        while ctt < RRT_Star[RRT_Star[tem_pos_x_near].Location_Parent].Children_Count:
                            if RRT_Star[RRT_Star[tem_pos_x_near].Location_Parent].Llist_Children[ctt] == tem_pos_x_near:
                                cttt = ctt
                                while cttt < RRT_Star[RRT_Star[tem_pos_x_near].Location_Parent].Children_Count:
                                    RRT_Star[RRT_Star[tem_pos_x_near].Location_Parent].Llist_Children[cttt] = RRT_Star[RRT_Star[tem_pos_x_near].Location_Parent].Llist_Children[cttt+1]
                                    cttt += 1
                                RRT_Star[RRT_Star[tem_pos_x_near].Location_Parent].Children_Count -= 1
                            ctt += 1
        
                        RRT_Star[tem_pos_x_near].Location_Parent = i
        
                        RRT_Star[i].Llist_Children[RRT_Star[i].Children_Count] = tem_pos_x_near
                        RRT_Star[i].Children_Count += 1
                        #self.ARewire_The_Tree(tem_pos_x_near, RRT_Star)
                i=i+1
            #else:
            #    print(i)
                 #f.write(str(i)+"\n")
            xrand_index+=1
            
        #f.close()

