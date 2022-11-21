# -*- coding: utf-8 -*-
"""
Created on Mon May 18 15:59:04 2020
 
@author: Necrospartan
"""
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from sympy.geometry import Point
import math

inf = open( '../Save/Execution_Information.txt' )
inf = inf.readlines()
Name = inf[ 0 ][ 0 ]

Information = open( '../Information/Information_' + Name + '.txt', 'r' )
Information = Information.readlines()

X = open ( '../Save/Solx_' + Name + '.txt', 'r' )
X = X.readlines()
Y = open ( '../Save/Soly_' + Name + '.txt', 'r' )
Y = Y.readlines()

Nodo = open( '../Save/Nodo_' + Name + '.txt', 'r' )
Nodo = Nodo.readlines()
Padre = open( '../Save/Padre_' + Name + '.txt', 'r' )
Padre = Padre.readlines()
Costo = open( '../Save/Costo_' + Name + '.txt', 'r' )
Costo = Costo.readlines()

Point_Init = Point( float( Information[ 1 ]), float( Information[ 2 ] ) )
Point_Goal = Point( float( Information[ 4 ]), float( Information[ 5 ] ) )
Radio_Robot = float( Information[ 9 ] )
Radio_Goal = float( Information[ 11 ] )
Num_Hole = int( Information[ 17 ] )

fig = plt.figure( figsize = ( 12, 12 ), dpi=100,)
ax = fig.add_subplot( 111 )

Pol_x = open( '../Information/Information_Pol_x_' + Name + '.txt' )
Pol_x = Pol_x.readlines()
Pol_y = open( '../Information/Information_Pol_y_' + Name + '.txt' )
Pol_y = Pol_y.readlines()

x = []
y = []
for i in range( 1, len( Pol_x ) ):
    x.append( float( Pol_x[ i ] ) )
    y.append( float( Pol_y[ i ] ) )

ax.fill( x, y, facecolor = 'white', edgecolor = 'k' )

if Num_Hole > 0:
    for hole in range( 1, Num_Hole +1 ):
        Hole_x = open( '../Information/Information_Pol_Hole_'+ str( hole ) + '_x_' + Name + '.txt' )
        Hole_x = Hole_x.readlines()
        Hole_y = open( '../Information/Information_Pol_Hole_'+ str( hole ) + '_y_' + Name + '.txt' )
        Hole_y = Hole_y.readlines()
        xx = []
        yy = []
        for i in range( 1, len( Hole_x ) ):
            xx.append( float( Hole_x[ i ] ) )         
            yy.append( float( Hole_y[ i ] ) )

        ax.fill( xx, yy, facecolor = 'gainsboro', edgecolor = 'k' )

x_root = float( X[ 0 ] )
y_root = float( Y[ 0 ] )

cost = 0.0
for j in range( 1, len( Costo ) ):
    if cost < float( Costo[ j ] ):
        cost = float( Costo[ j ] )
        
for i in range( 1, len( Nodo ) ):
    x_t = []
    y_t = []
    x_t.append( float( X[ i ] ) )
    x_t.append( float( X[ int( Padre[ i ] ) ] ) )
    y_t.append( float( Y[ i ] ) )
    y_t.append( float( Y[ int( Padre[ i ] ) ] ) )
 
    alpha_tem = math.sqrt( ( x_root-x_t[ 0 ] ) * ( x_root - x_t[ 0 ] ) +
                     ( y_root - y_t[ 0 ] ) * ( y_root - y_t[ 0 ] ) )

    alpha_tem = 0.3 * ( ( ( cost - float( Costo[ i ] ) ) / cost ) + 0.01 )
    line = plt.Line2D( x_t, y_t, color = 'b', alpha = alpha_tem )
    ax.add_line( line )

plt.savefig('RRT_Star.png')
plt.show()
