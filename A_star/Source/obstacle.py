import math
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt


class Obstacle():
    def __init__(self, width = 400, height = 250, radius = 1, clearance = 1, threshold=0.5, thetaStep = 30):
        self.threshold = threshold 
        self.W = int(width/threshold) 
        self.H = int(height/threshold) 
        self.radius = radius
        self.clearance = clearance
        self.thetaStep = thetaStep 
        self.explore = np.zeros([self.H, self.W, 360//thetaStep, 4])
        self.plotData_X = []
        self.plotData_Y = []
        self.plotData_U = []
        self.plotData_V = []

    def image_plot(self, ax):
        # Hexagonal
        hexa = [[183,165,183,217,235,218,183],[130,100,70,70,100,130,130]] 

        # Arrow 
        arrow = [[115,36,105,80,115],[210,185,100,180,210]]


        # Circle
        centX, centY,radii = 300,185,40 
        x = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        
        plt.plot(hexa[0],hexa[1])
        plt.plot(arrow[0], arrow[1])
        plt.plot(x, y)

        return ax

    def checkBoundary(self,i,j):
        if i < (400 - self.radius - self.clearance) and i > (self.radius + self.clearance) and j < (250 - self.radius - self.clearance) and j > (self.radius + self.clearance):
            return False
        else:
            return True

    def obst_flag(self, x, y):
        if self.checkBoundary(x,y):
            return False
        elif self.obst_circle(x,250-y,(300,65),40):
            return False
        elif self.obst_hexa(x,250-y):
            return False
        elif self.obst_arrow(x,250-y):
            return False
        else:
            return True
    
    def obst_circle(self, x, y, center, radius):

        center_x, center_y = center[0], center[1]
        if ((x - center_x) ** 2 + (y - center_y) ** 2) <= (radius + self.radius + self.clearance) ** 2:
            return True
        else:
            return False

    def obst_hexa(self, x, y):
        contours= [np.array([[200, 109], [235, 129], [235, 170], [200, 190], [165, 170], [165, 129]], dtype=np.int32)] 
        drawing=np.zeros([400,250],np.uint8)
        cv2.drawContours(drawing,contours, -1, (36, 255, 12), 2)
        result1 = cv2.pointPolygonTest(np.array([[200, 109], [235, 129], [235, 170], [200, 190], [165, 170], [165, 129]]), (x,y), False)
        if(result1>=0):
            return True
        return False

    def obst_arrow(self, x,y):
        contours= [np.array([[115,40], [36,65], [105,150], [80,70]], dtype=np.int32)] 
        drawing=np.zeros([400,250],np.uint8)
        cv2.drawContours(drawing,contours, -1, (36, 255, 12), 2)
        result1 = cv2.pointPolygonTest(np.array([[115,40], [36,65], [105,150], [80,70]]), (x,y), False)
        if(result1>=0):
            return True
        return False

    def check_visit(self, node):
        print(node)
        if(node[2]>=400):
            node[2]=399
        if(node[1]>=250):
            node[1]=249
        X = int(round(node[1]/self.threshold))
        Y = int(round(node[2]/self.threshold))
        ang = int(node[3]//self.thetaStep)
        if self.explore[Y, X, ang,3] != 0:
            return True
        else:
            return False

    def visit_node(self, node):
        X = int(round(node[1]/self.threshold))
        Y = int(round(node[2]/self.threshold))
        A = int(node[3]//self.thetaStep)
        return self.explore[Y,X,A, :]

    def append_visited(self, node, p_Node):
        X = int(round(node[1]/self.threshold))
        Y = int(round(node[2]/self.threshold))
        A = int(node[3]//self.thetaStep)
        self.plotData_X.append(p_Node[1])
        self.plotData_Y.append(p_Node[2])
        self.plotData_U.append(node[1] - p_Node[1]) 
        self.plotData_V.append(node[2] - p_Node[2])
        self.explore[Y, X, A, :] = np.array(p_Node)
        return

    def plot_shapes(self, path):
        plt.ion()
        fig, ax = plt.subplots()
        ax = self.image_plot(ax)
        for i in range(1,len(self.plotData_X)+1,2000):
            plt.xlim(0,400)
            plt.ylim(0,250)
            q = ax.quiver(self.plotData_X[i:(i+2000)], self.plotData_Y[i:(i+2000)], 
                self.plotData_U[i:(i+2000)], self.plotData_V[i:(i+2000)], units='xy' ,
                scale = 1, headwidth = 0.1, headlength=0,
                width=0.2)
            plt.pause(0.0001)

        X,Y,U,V = [],[],[],[] 
        for i in range(len(path)-1):
            X.append(path[i][1])
            Y.append(path[i][2])
            U.append(path[i+1][1] - path[i][1])
            V.append(path[i+1][2] - path[i][2])
            
        for i in range(len(X)):
            # plt.cla()
            plt.xlim(0,400)
            plt.ylim(0,250)
            q = ax.quiver(X[i], Y[i], U[i], V[i], units='xy', 
                scale=1, color='r', headwidth = 0.1, 
                headlength=0, width = 0.7)
            plt.pause(0.0001)
        plt.ioff()
        plt.show()

