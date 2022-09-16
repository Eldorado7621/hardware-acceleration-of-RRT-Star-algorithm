#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 11:38:10 2020

@author: Necrospartan
"""

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
from sympy.geometry import Point
import math

inf = open( '../Save/Execution_Information.txt' )
inf = inf.readlines()
Name = inf[ 0 ][ 0 ]

Information = open( '../Information/Information_' + Name + '.txt', 'r' )
Information = Information.readlines()

x_trajectory = open( '../Save/Trajectory_x_' + Name + '.txt','r' )
x_trajectory = x_trajectory.readlines()
y_trajectory = open( '../Save/Trajectory_y_' + Name + '.txt', 'r' )
y_trajectory = y_trajectory.readlines()

Point_Init = Point( float( Information[ 1 ]), float( Information[ 2 ] ) )
Point_Goal = Point( float( Information[ 4 ]), float( Information[ 5 ] ) )
Radio_Robot = float( Information[ 9 ] )
Radio_Goal = float( Information[ 11 ] )
Num_Hole = int( Information[ 17 ] )

fig = plt.figure( figsize = ( 12, 12 ), dpi = 100, )
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

for i in range( 1, len( x_trajectory ) ):
    x_t = []
    y_t = []
    x_t.append( float( x_trajectory[i-1] ) )
    x_t.append( float( x_trajectory[i] ) )
    y_t.append( float( y_trajectory[i-1] ) )
    y_t.append( float( y_trajectory[i] ) )
    line = Line2D( x_t, y_t, color='b' )
    ax.add_line( line )

circle = plt.Circle( Point_Init, Radio_Robot, edgecolor = 'b', fill = False, zorder=2 )
ax.add_artist( circle )
circle = plt.Circle( Point_Goal, Radio_Goal, edgecolor = 'r', fill = False, zorder=2 )
ax.add_artist( circle )

plt.savefig('Trajectory.png')
plt.show()
