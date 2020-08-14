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
from scipy.integrate import solve_ivp
        
class Boat():
    """
    A class that modelizes an autonomous sailboat dynamics, considering 
    wind-related constraints, and a path to follow.
    
    Attributes
    ----------
    p : list of integers
        [p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10] sailboat-related constant values.
        p0: drift coefficient, p1: tangeantial friction, p2: angular friction,
        p3: sail lift, p4: rudder lift, p5: distance to the sail, 
        p6: distance to the mast, p7: distance to the rudder, 
        p8: mass of the sailboat, p9: moment of inertia, 
        p10: rudder break coefficient.
        
    x : 5x1 numpy array
        state vector of the sailboat, x = [x,y,θ,v,w].
        
    a_tw : float
        the true wind force.
        
    ψ_tw : float in radians
        the true wind angle.
        
    r : float
        the maximale acceptable distance. If the sailboat is facing wind, it
        will follow a zigzag pattern not exceding a r meters distance from the 
        line to follow.   
        
    ζ : float in radians
        the closed hauled angle for the no-go zone, generally between pi/3 and 
        pi/5 (or 40 to 60 degrees).
        
    δrmax: float in radians
         the maximal rudder angle.
         
    β : float in radians
        angle of the sail in crosswind. 
    
    u : 2*1 umpy array
        current command to control the sailboat.
        u = [rudder_angle, sail_max_angle].
        
    trajectory : numpy array
        all the previous positions of the boat to plot its trajecory on a graph.
        
    Methods
    -------
    
    draw_sailboat(δs,δr,showTrajectory)
    controler(point_a,point_b)
    f()
    nextStep(path,dt,showTrajectory)
    
    """
    def __init__(self, x, a_tw, ψ_tw, r, ζ, δrmax, β):
        self.p = (0.03,40,6000,200,1500,0.5,0.5,2,300,400,0.2)
        self.x = x
        self.a_tw = a_tw
        self.ψ_tw = ψ_tw
        self.r = r
        self.ζ = ζ
        self.δrmax = δrmax
        self.β = β
        self.u = array([[0],[0]])
        self.trajectory = []
        self.q = -1.0
        self.t = 0
        
    def draw_sailboat(self,δs,δr, showTrajectory=False):
        """
        gives a graphical representation of the sailboat with its position,
        its orientation, the wind perceived. The trajectory can be shown with
        the boolean showTrajectory.
        """
        x = self.x.flatten()
        θ = x[2]
        hull=array([[-1,5,7,7,5,-1,-1,-1],[-2,-2,-1,1,2,2,-2,-2],[1,1,1,1,1,1,1,1]])
        sail=array([[-7,0],[0,0],[1,1]])
        rudder=array([[-1,1],[0,0],[1,1]])
        R=array([[cos(θ),-sin(θ),x[0]],[sin(θ),cos(θ),x[1]],[0,0,1]])
        Rs=array([[cos(δs),-sin(δs),3],[sin(δs),cos(δs),0],[0,0,1]])
        Rr=array([[cos(δr),-sin(δr),-1],[sin(δr),cos(δr),0],[0,0,1]])
        draw_arrow(x[0]+5,x[1],self.ψ_tw,5*self.a_tw,'red')
        plot2D(R@hull,'black');       
        plot2D(R@Rs@sail,'red');       
        plot2D(R@Rr@rudder,'red'); 
        if(showTrajectory):
            plt.plot([i[0] for i in self.trajectory],[j[1] for j in self.trajectory],'--',color="yellowgreen")
            
    def f(self):
        """
        calculates the derivative of state vector x and the current sail angle. 
        The model equations are based upon "Modeling, control and state-
        estimation for an autonomous sailboat" of Jon Melin.
        """
        p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10 = self.p
        x,u=self.x.flatten(),self.u.flatten()
        θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
        a_tw,ψ_tw = self.a_tw, self.ψ_tw
        # apparent wind calculation
        w_aw = array([[a_tw*cos(ψ_tw-θ) - v],[a_tw*sin(ψ_tw-θ)]])
        ψ_aw = arctan2(w_aw[1,0],w_aw[0,0])
        a_aw=norm(w_aw)
        sigma = cos(ψ_aw) + cos(δsmax)
        if sigma < 0 :
            δs = pi + ψ_aw
        else :
            δs = -sign(sin(ψ_aw))*δsmax
        # rudder force
        fr = p4*v*sin(δr)
        # sail force
        fs = p3*a_aw* sin(δs - ψ_aw)
        # model equations
        dx=v*cos(θ) + p0*a_tw*cos(ψ_tw)
        dy=v*sin(θ) + p0*a_tw*sin(ψ_tw)
        dv=(fs*sin(δs)-fr*p10*sin(δr)-p1*v**2)/p8
        dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
        xdot=array([ [dx],[dy],[w],[dv],[dw]])
        return xdot,δs  
    
    def controller(self,a,b):
        """
        calculates the angle for the rudder and the maximum angle for the sail 
        to control the sailboat (following the line from a point a to another 
        point b). This controller is based upon "A  simple controller for line 
        following of sailboats" of Luc Jaulin and Fabrice Le Bars.
        """
        m = array([[self.x[0,0]],[self.x[1,0]]])
        θ = self.x[2,0]
        ψ_tw, ζ, r, δrmax, β = self.ψ_tw, self.ζ, self.r, self.δrmax, self.β
        # algebraic distance calculation
        e = det(hstack((b-a,m-a)))*1/norm(b-a)
        # update of the hysteresis variable
        if abs(e) > r:
            self.q = sign(e)
        # line angle calculation
        phi = arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])
         # nominal heading calculation (angle to reach without wind constraint)
        θbar = phi - arctan(e/r)
        # no go zone belonging check
        if (cos(ψ_tw-θbar)+cos(ζ)<0) or (abs(e)<r and cos(ψ_tw-phi)+cos(self.ζ)<0):
              θbar = pi+ψ_tw-self.q*ζ
        # rudder control 
        δr = 2*(δrmax/pi)*arctan(tan((θ-θbar)/2))
        # sail control 
        δsmax = (pi/2)*((cos(ψ_tw-θbar)+1)/2)**(log(pi/(2*β))/log(2))
        self.u = array([[δr],[δsmax]])
        

    
    def nextStep(self,ax,path,dt=0.1,showTrajectory=False,plot_frequency=2):
        """
        manages automatically the evolution of the sailboat over time.
        """
        a,b = path[0], path[1]
        self.controller(a,b)
        xdot,δs = self.f()
        self.x = self.x + dt*xdot
        self.t += dt
        if round(self.t,2)%plot_frequency==0:
            plt.plot([a[0,0],b[0,0]],[a[1,0],b[1,0]],linestyle='dotted',color="red")
            plt.plot([i[0,0] for i in path],[j[1,0] for j in path],'co')
            self.draw_sailboat(δs,self.u[0,0],showTrajectory)
            clear(ax)
        if showTrajectory:
            self.trajectory.append((self.x[0],self.x[1]))
        if ((b-a).T)@(b-array([[self.x[0,0]],[self.x[1,0]]]))<0:
            path.append(path.pop(0))
            a,b = a,b = path[0], path[1]
    
def f_ode_45(t,x,boat,ax,path,showTrajectory=False):
    boat.x = array([x]).T
    a,b = path[0], path[1]
    boat.controller(a,b)
    xdot,δs = boat.f()
    if showTrajectory:
        boat.trajectory.append((boat.x[0],boat.x[1]))
    if round(t,10)%(1)==0:
        plt.plot([a[0,0],b[0,0]],[a[1,0],b[1,0]],linestyle='dotted',color="red")
        plt.plot([i[0,0] for i in path],[j[1,0] for j in path],'co')
        boat.draw_sailboat(δs,boat.u[0,0],showTrajectory)
        clear(ax)
    if ((b-a).T)@(b-array([[boat.x[0,0]],[boat.x[1,0]]]))<0:
        path.append(path.pop(0))
        a,b = a,b = path[0], path[1]
    return((xdot.T).tolist()[0])
      

def draw_arrow(x,y,θ,L,col):
    e=0.2
    M1=L*array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M= append(M1,[[1,1,1,1,1]],axis=0)
    R=array([[cos(θ),-sin(θ),x],[sin(θ),cos(θ),y],[0,0,1]])
    plot2D(R@M,col)
    
def plot2D(M,col='black',w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth = w) 
    
def init_figure(path, xmin = None, xmax = None, ymin = None, ymax = None): 
    fig = plt.figure(3)
    ax = fig.add_subplot(111, aspect='equal')	
    if xmin == None:
        ax.xmin=min(path, key=lambda x: x[0])[0,0]-20
        ax.xmax=max(path, key=lambda x: x[0])[0,0]+20
        ax.ymin=min(path, key=lambda x: x[1])[1,0]-20
        ax.ymax=max(path, key=lambda x: x[1])[1,0]+20
    else:
        ax.xmin, ax.xmax, ax.ymin, ax.ymax = xmin, xmax, ymin, ymax
    clear(ax)
    return ax     
    
def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)    
    

if __name__=='__main__':
    
#---------------------Parameters to set----------------------------------------

    x0= array([[10,-40,-3,1,0]]).T   #x=(x,y,θ,v,w)
    dt = 0.10
    
    a_tw = 2    # true wind force
    ψ_tw = -2   # true wind angle
    r = 10      # maximale acceptable distance from target line
    ζ = pi/4    # closed hauled angle for the no-go zone
    δrmax = 1   # maximal rudder angle
    β = pi/4    # angle of the sail in crosswind 
    
    Boat = Boat(x0,a_tw, ψ_tw, r, ζ, δrmax, β)
    
    A = array([[-80],[-80]])   
    B = array([[50],[50]])
    C = array([[-80],[0]])
    D = array([[25],[-50]])
    path = [A,B,C,D]
    a,b = path[0], path[1]
    
    ax=init_figure(path)
    
#---------------------Choose an integration method-----------------------------
    
    # Euler
#    for t in arange(0,10000,dt):
#        Boat.nextStep(ax,path,dt,showTrajectory=True,plot_frequency=10)
    
    # Runge-Kutta 45
    plot_rate = 0.7  # every x time period
    for t in range(0,10000):
        y = solve_ivp(f_ode_45, (0,plot_rate),(x0.T).tolist()[0],method='RK45',
                      args =(Boat,ax,path,True))
        x0 = y.y[:,-1].reshape((5,1))
    
    