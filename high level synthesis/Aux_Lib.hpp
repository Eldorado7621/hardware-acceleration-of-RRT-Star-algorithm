#ifndef Aux_Lib_H
    #define Aux_Lib_H
    #include <fstream>
    #include <ctime>
    #include <iostream>
    #include <list>
    #include <vector>
    #include <cmath>
    ////////////////////////////////////////////////

    static const float Constant_Max = 10000;
    static const float Constant_Min = -10000;

    class Point_2
    {
	private:
	    float _x;
	    float _y;
	public:
	    Point_2()
	    {
		_x = 0.f;
		_y = 0.f;
	    }
	    ~Point_2(){}
	    Point_2( const float _x, const float _y )
	    {
		this->_x = _x;
		this->_y = _y;
	    }
	    float x() const
	    {
		return _x;
	    }
	    float y() const
	    {
		return _y;
	    }
	    float squared_distance( Point_2 &p )
	    {
		return pow( _x - p.x(), 2 ) + pow( _y - p.y(), 2 );
	    }
    };

    class Segment_2
    {
	//Note Ax + By + C = 0
	//A = ( y_2 - y_1 )
	//B = -( x_2 - x_1 )
	//C = ( x_2 - x_1 )y_1 - ( y_2 - y_1 )x_1
	private:
	    Point_2 Source;
	    Point_2 Target;
	    float A;
	    float B;
	    float AABB;
	public:
	    Segment_2()
	    {
		A = 0.f;
		B = 0.f;
		AABB = 0.f;
	    }
	    ~Segment_2(){}
	    Segment_2( const Point_2 &_source, const Point_2 &_target )
	    {
		Source = _source;
		Target = _target;
		A = ( Target.y() - Source.y() );
		B = ( Target.x() - Source.x() );
		AABB = A*A + B*B;
	    }
	    Point_2 source() const
	    {
		return Source;
	    }
	    Point_2 target() const
	    {
		return Target;
	    }
	    float squared_distance( Point_2 &p )
	    {
		float U = ( ( p.x() - Source.x() ) * ( Target.x() - Source.x() ) +
		    ( p.y() - Source.y() ) * ( Target.y() - Source.y() ) ) / AABB;
		if( U >= 0 && U <= 1 )
		    return pow( ( ( Target.x() - Source.x() ) * ( p.y() - Source.y() ) -
			( Target.y() -Source.y() ) * ( p.x() - Source.x() ) ), 2 )/ AABB;
		else
		    if( U > 1 )
			return Target.squared_distance( p );
		    else
			return Source.squared_distance( p );
	    }
    };
    class Polygon_2
    {
	//Note implement:
	//"is_clockwise_orientation"
	//"reverse_orientation"
	private:
	    int Size;
	public:
	    std::vector<Segment_2> edge;
	    std::vector<Point_2> vertex;
	    Polygon_2()
	    {
		Size = 0;
	    }
	    ~Polygon_2(){}
	    void push_back( const Point_2 &_vertex )
	    {
	/*	if( Size > 0 )
		{
		    if( Size > 1 )
			edge.pop_back();
		    edge.push_back( Segment_2( vertex.at( Size -1 ), _vertex ) );
		    edge.push_back( Segment_2( _vertex, vertex.at( 0 ) ) );
		}
		vertex.push_back ( _vertex );*/
		Size++;
	    }
	    int size()
	    {
		return Size;
	    }

    };
#endif
