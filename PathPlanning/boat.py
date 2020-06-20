#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 19 11:10:45 2020

@author: agathe
"""
from numpy import *
from numpy.linalg import *
import matplotlib.pyplot as plt
import math as m
global q  
  
    
def f(x,u):
    x,u=x.flatten(),u.flatten()
    θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
    w_ap = array([[awind*cos(ψ-θ) - v],[awind*sin(ψ-θ)]])
    ψ_ap = arctan2(w_ap[1,0],w_ap[0,0])
    a_ap=norm(w_ap)
    sigma = cos(ψ_ap) + cos(δsmax)
    if sigma < 0 :
        δs = pi + ψ_ap
    else :
        δs = -sign(sin(ψ_ap))*δsmax
    # rudder force
    fr = p4*v*sin(δr)
    # sail force
    fs = p3*a_ap* sin(δs - ψ_ap)
    # model equations
    dx=v*cos(θ) + p0*awind*cos(ψ)
    dy=v*sin(θ) + p0*awind*sin(ψ)
    dv=(fs*sin(δs)-fr*p10*sin(δr)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
    xdot=array([ [dx],[dy],[w],[dv],[dw]])
    return xdot,δs  

def controler(x,a,b):
    global q
    m = array([[x[0,0]],[x[1,0]]])
    θ = x[2,0]
    # algebraic distance calculation
    e = det(hstack((b-a,m-a)))*1/norm(b-a)
    # update of the hysteresis variable
    if abs(e) > r:
        q = sign(e)
    # line angle calculation
    phi = arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])
     # nominal heading calculation (angle to reach without wind constraint)
    θbar = phi - arctan(e/r)
    # no go zone belonging check
    if (cos(ψ-θbar)+cos(gamma)<0) or (abs(e)<r and cos(ψ-phi)+cos(gamma)<0):
          θbar = pi+ψ-q*gamma
    # rudder control 
    deltar = 2*(deltarmax/pi)*arctan(tan((θ-θbar)/2))
    # sail control 
    deltasmax = (pi/2)*((cos(ψ-θbar)+1)/2)**(log(pi/(2*beta))/log(2))
    u = array([[deltar],[deltasmax]])
    return(u)
        
    
def init_figure(xmin,xmax,ymin,ymax): 
    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')	
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax       
    
def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)    

def draw_arrow(x,y,θ,L,col):
    e=0.2
    M1=L*array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M= append(M1,[[1,1,1,1,1]],axis=0)
    R=array([[cos(θ),-sin(θ),x],[sin(θ),cos(θ),y],[0,0,1]])
    plot2D(R@M,col)
    
def plot2D(M,col='black',w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth = w)   

def draw_sailboat(x,δs,δr,ψ,awind):
    x=x.flatten()
    θ=x[2]
    hull=array([[-1,5,7,7,5,-1,-1,-1],[-2,-2,-1,1,2,2,-2,-2],[1,1,1,1,1,1,1,1]])
    sail=array([[-7,0],[0,0],[1,1]])
    rudder=array([[-1,1],[0,0],[1,1]])
    R=array([[cos(θ),-sin(θ),x[0]],[sin(θ),cos(θ),x[1]],[0,0,1]])
    Rs=array([[cos(δs),-sin(δs),3],[sin(δs),cos(δs),0],[0,0,1]])
    Rr=array([[cos(δr),-sin(δr),-1],[sin(δr),cos(δr),0],[0,0,1]])
    draw_arrow(x[0]+5,x[1],ψ,5*awind,'red')
    plot2D(R@hull,'black');       
    plot2D(R@Rs@sail,'red');       
    plot2D(R@Rr@rudder,'red');
    

p0 = 0.03  # drift coeff
p1 = 40    # tangeantial friction
p2 = 6000  # angular friction
p3 = 200   # sail lift
p4 = 1500 # rudder lift
p5 = 0.5   # distance to sail
p6 = 0.5   # distance to mast
p7 = 2     # distance to rudder
p8 = 300   # mass of boat
p9 = 400   # moment of inertia
p10 = 0.2  # rudder break coeff

x = array([[10,-40,-3,1,0]]).T   #x=(x,y,θ,v,w)

dt = 0.10

awind = 2     # true wind force
ψ = -2        # true wind angle
r = 10        # maximale acceptable distance from target line
gamma = pi/4  # closed hauled angle for the no-go zone
deltarmax = 1 # maximal rudder angle
beta = pi/4   # angle of the sail in crosswind 

A = array([[-80],[-80]])   
B = array([[50],[50]])
C = array([[-80],[0]])
D = array([[25],[-50]])
path = [A,B,C,D]
a,b = path[0], points[1]

ax=init_figure(-100,100,-100,100)

for t in arange(0,10000,0.1):
    u=controler(x,a,b)
    xdot,δs=f(x,u)
    x = x + dt*xdot
    if t%2==0:
        plt.plot([a[0,0],b[0,0]],[a[1,0],b[1,0]],'--r')
        plt.plot([i[0,0] for i in path],[j[1,0] for j in path],'co')
        draw_sailboat(x,δs,u[0,0],ψ,awind)
        clear(ax)
    if ((b-a).T)@(b-array([[x[0,0]],[x[1,0]]]))<0:
        path.append(path.pop(0))
        a,b = a,b = path[0], path[1]

    