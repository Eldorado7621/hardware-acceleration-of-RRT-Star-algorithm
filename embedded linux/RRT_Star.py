import math
from Aux_Lib import *
import numpy

class Position_Holonomic:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def Insert_Position(self, x, y): #x.y are floating points
        self.x = x
        self.y = y

    def Copy(self, A): #A is of class Position_Holonomic
        self.x = A.x
        self.y = A.y
    
    def get_x(self):
        return self.x
    
    
class Position_Holonomichp:
    def __init__(self, x=0, y=0):
        self.x = numpy.float16(x)
        self.y = numpy.float16(y)

    def Insert_Position(self, x, y): #x.y are floating points
        self.x = numpy.float16(x)
        self.y = numpy.float16(y)

    def Copy(self, A): #A is of class Position_Holonomic
        self.x = A.x
        self.y = A.y
    
    def get_x(self):
        return self.x


class Node_Rand:
    def __init__(self):
        self.Position = Position_Holonomichp()
    
class Node_RRT_hp:
    def __init__(self):
        self.Position = Position_Holonomichp()
        self.Location_List=numpy.float16(0.0)
        self.Cost=numpy.float16(0.0)
        
    def Copy(self, A): #A is of class Position_Holonomic
        self.Position.Copy(A.Position)
        self.Location_List = A.Location_List
        self.Cost=A.Cost
        
class Node_RRT_fp:
    def __init__(self):
        self.Position = Position_Holonomic()
        self.Location_List=0
        self.Cost=0
        
    def Copy(self, A): #A is of class Position_Holonomic
        self.Position.Copy(A.Position)
        self.Location_List = A.Location_List
        self.Cost=A.Cost
    
class Node_RRTop:
    def __init__(self):
        self.Location_Parent = 0
        
        
        self.Point = Point_2()
        
        self.Local_Cost = 0
        self.Llist_Children = [0 for _ in range(100)]
        self.Children_Count = 0
        


    def Copy(self, A): #A is of type Node_RRT
        self.Location_Parent = A.Location_Parent
        
        
        self.Point = A.Point
        
        self.Local_Cost = A.Local_Cost
        self.Children_Count = A.Children_Count
        for i in range(A.Children_Count):
            self.Llist_Children[i] = A.Llist_Children[i]
            
            
class Node_RRT:
    def __init__(self):
        self.Location_Parent = 0
        self.Location_List = 0
        self.Position = Position_Holonomic()
        self.Point = Point_2()
        self.Cost = 0
        self.Local_Cost = 0
        self.Llist_Children = [0 for _ in range(100)]
        self.Children_Count = 0
        


    def Copy(self, A): #A is of type Node_RRT
        self.Location_Parent = A.Location_Parent
        self.Location_List = A.Location_List
        self.Position.Copy(A.Position)
        self.Point = A.Point
        self.Cost = A.Cost
        self.Local_Cost = A.Local_Cost
        self.Children_Count = A.Children_Count
        for i in range(A.Children_Count):
            self.Llist_Children[i] = A.Llist_Children[i]

            
class Region:
    def __init__(self):
        self.Center_Position = Position_Holonomic()
        self.x_min = 0
        self.x_max = 0
        self.y_min = 0
        self.y_max = 0
        self.r_n = 0
        self.Squared_r_n = 0

    def __del__(self):
        pass

    def __init__(self, Center_Position_, r_n_, Squared_r_n_, Environment):
        self.r_n = r_n_
        self.Center_Position.x = Center_Position_.x
        self.Center_Position.y = Center_Position_.y

        self.x_min = self.Center_Position.x - r_n_
        self.x_max = self.Center_Position.x + r_n_
        self.y_min = self.Center_Position.y - r_n_
        self.y_max = self.Center_Position.y + r_n_
        self.Squared_r_n = Squared_r_n_

        self.x_min = self.y_min = Constant_Max
        self.x_max = self.y_max = Constant_Min

        for vi in Environment.vertex:
            if vi.x() < self.x_min:
                self.x_min = vi.x()
            if vi.x() > self.x_max:
                self.x_max = vi.y()
            if vi.y() < self.y_min:
                self.y_min = vi.y()
            if vi.y() > self.y_max:
                self.y_max = vi.y()

    def __init__(self, Center_Position_, r_n_, Squared_r_n_):
        self.r_n = r_n_
        self.Center_Position.x = Center_Position_.x
        self.Center_Position.y = Center_Position_.y
        self.x_min = self.Center_Position.x - r_n_
        self.x_max = self.Center_Position.x + r_n_
        self.y_min = self.Center_Position.y - r_n_
        self.y_max = self.Center_Position.y + r_n_
        self.Squared_r_n = Squared_r_n_

    def __init__(self, x_min_, x_max_, y_min_, y_max_):
        self.x_min = x_min_
        self.x_max = x_max_
        self.y_min = y_min_
        self.y_max = y_max_

    def __init__(self, Environment, r_n_):
        self.r_n = r_n_
        self.Squared_r_n = r_n_ * r_n_
        self.x_min = Constant_Max
        self.x_max = Constant_Min
        self.y_min = Constant_Max
        self.y_max = Constant_Min

        for vi in Environment.vertex:
            if vi.x() < self.x_min:
                self.x_min = vi.x()
            if vi.x() > self.x_max:
                self.x_max = vi.x()
            if vi.y() < self.y_min:
                self.y_min = vi.y()
            if vi.y() > self.y_max:
                self.y_max = vi.y()

    def in_region(self, Pos):
        if pow(Pos.x - self.Center_Position.x, 2) + pow(Pos.y - self.Center_Position.y, 2) < self.Squared_r_n:
            return True
        else:
            return False
