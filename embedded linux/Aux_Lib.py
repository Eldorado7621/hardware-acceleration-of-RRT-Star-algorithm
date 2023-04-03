import math

Constant_Max = 10000
Constant_Min = -10000
MAX_VERTICES = 10
MAX_EDGES    = 4


class Point_2:
    def __init__(self, x=0.0, y=0.0):
        self._x = x
        self._y = y
    
    def x(self):
        return self._x
    
    def y(self):
        return self._y
    
    def squared_distance(self, p):
        return (self._x - p.x())**2 + (self._y - p.y())**2

class Segment_2:
    def __init__(self):
        self.A=0.0
        self.B=0.0
        self.AABB=1.0
        
    def __init__(self, source=Point_2(), target=Point_2()):
        self.Source = source
        self.Target = target
        self.A = (target.y() - source.y())
        self.B = (target.x() - source.x())
        self.AABB = math.pow(self.A, 2) + math.pow(self.B, 2)
        

    def source(self):
        return self.Source

    def target(self):
        return self.Target

    def squared_distance(self, p):
        
        
        U = ((p.x() - self.Source.x()) * (self.Target.x() - self.Source.x()) + 
            (p.y() - self.Source.y()) * (self.Target.y() - self.Source.y())) / self.AABB
        
        #if ui[0]==83:
        #    print("shit",U,p.x(),self.Source.x())
        #if ui[0]==4521:
        #    print("u",U)
        #    print("shit",p.x(),self.Source.x(),self.Target.x(),self.Source.y(),self.Target.y(),self.AABB)
        if U >= 0 and U <= 1:
            val=math.pow(((self.Target.x() - self.Source.x()) * 
                           (p.y() - self.Source.y()) - (self.Target.y() - self.Source.y()) *
                           (p.x() - self.Source.x())), 2) / self.AABB
            #if ui[0]==4521:
            #    print("val",val)
            #if ui[0]==83:
            #    print("shit",U,p.x(),self.Source.x(),val)
            return val
        
      
        
        elif U > 1:
            return self.Target.squared_distance(p)
        else:
            return self.Source.squared_distance(p)

class Polygon_2:
    def __init__(self):
        self.Size = 0
        self.edge = [Segment_2()]
        self.vertex = [Point_2()]

    def push_back(self, _vertex):
        if self.Size > 0:
            if self.Size > 1:
              self.edge.pop()
        
            self.edge.append(Segment_2(self.vertex[self.Size - 1], _vertex))
            self.edge.append(Segment_2(_vertex, self.vertex[0]))
        self.vertex.append(_vertex)
        self.Size += 1

    def size(self):
        return self.Size

class Segment_3:
    def __init__(self, source, target):
        self.Source = source
        self.Target = target

    def source(self):
        return self.Source

    def target(self):
        return self.Target

    def squared_distance(self, p):
        U = ((p.x() - self.Source.x()) * (self.Target.x() - self.Source.x()) +
             (p.y() - self.Source.y()) * (self.Target.y() - self.Source.y())) / 1
        if 0 <= U <= 1:
            return pow((self.Target.x() - self.Source.x()) * (p.y() - self.Source.y()) -
                       (self.Target.y() - self.Source.y()) * (p.x() - self.Source.x()), 2) / 1
        elif U > 1:
            return self.Target.squared_distance(p)
        else:
            return self.Source.squared_distance(p)
        
class Polygon_4:
    # Note implement:
    # "is_clockwise_orientation"
    # "reverse_orientation"
    def __init__(self):
        self.Size = 0
        self.edges = [Segment_3() for _ in range(MAX_EDGES)]
        self.vertices = [Point_2() for _ in range(MAX_VERTICES)]

    def size(self):
        return self.Size

class Polygon_3:
    # Note implement:
    # "is_clockwise_orientation"
    # "reverse_orientation"
    def __init__(self):
        self.Size = 0
        self.edges = [Segment_2() for _ in range(MAX_EDGES)]
        self.vertices = [Point_2()for _ in range(MAX_VERTICES)]

    def size(self):
        return self.Size

