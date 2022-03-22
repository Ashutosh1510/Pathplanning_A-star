from heapq import heappush, heappop
import argparse
import sys
from obstacle import *

class Astar_algo():
    def __init__(self, initial, goal, thetaStep = 30, stepSize = 1, goalThreshold = 2,
        width = 400, height = 250, threshold = 0.5, r = 1, c = 1):
        self.initial = initial
        self.goal = goal
        self.nodeData = []
        self.Data = []
        self.allData = []
        self.thetaStep = thetaStep
        self.stepSize = stepSize
        self.goalThreshold = goalThreshold
        self.setActions()
        self.Obstacle = Obstacle(width, height, radius = r, clearance = c, threshold=threshold, thetaStep=self.thetaStep)
    
    def setActions(self):
        self.actionSet = []
        for theta in np.arange(0, 360, self.thetaStep):
            ang = math.radians(theta)
            x = self.stepSize*math.cos(ang)
            y = self.stepSize*math.sin(ang)
            cost = 1
            self.actionSet.append([x, y, theta, cost])

    def initialCheck(self):
        if not self.Obstacle.obst_flag(self.goal[0], 250-self.goal[1]):
            print("Goal point present in Obstacle shape!")
            return False
        elif not self.Obstacle.obst_flag(self.initial[0], 250-self.goal[1]):
            print("Initial point present in Obstacle shape!")
            return False
        else:
            cost = math.sqrt((self.initial[0] - self.goal[0])**2 + (self.initial[1] - self.goal[1])**2)
            heappush(self.Data, [cost, self.initial[0], self.initial[1], self.initial[2], 0])
            self.nodeData.append([self.initial[0], self.initial[1], self.initial[2], 0])
            return True

    def eucd_dist(self, current): 
        # euclidian distance between current node and goal
        eucd = math.sqrt((current[1] - self.goal[0])**2 + (current[2] - self.goal[1])**2)
        return eucd

    def goal_flag(self, current):  
        # current point is inside threshold area around the goal or not
        x, y = current[1], current[2]
        if (x -  self.goal[0])**2 + (y -  self.goal[1])**2 <= (self.goalThreshold)**2:
            return True
        else:
            return False

    def Backtracking(self, presentNode):
        bt = []
        currentNode = presentNode[:4]
        bt.append(currentNode)
        while currentNode[1:] != self.initial:
            currentNode = list(self.Obstacle.visit_node(currentNode))
            bt.append(currentNode)
        print("Backtracking ...")
        bt.reverse()
        return bt

    def a_star(self):
        if self.initialCheck():
            while len(self.Data)>0:
                presentNode = heappop(self.Data)
                previousCost, previousCostToCome = presentNode[0], presentNode[4]
                if self.goal_flag(presentNode):
                    self.goalReach = True
                    print("Destination Arrived ...")
                    path = self.Backtracking(presentNode)
                    self.Obstacle.plot_shapes(path)
                    return
                for i in self.actionSet:
                    newNodeX = presentNode[1] + i[0]
                    newNodeY = presentNode[2] + i[1]
                    newNodeA = i[2]
                    newNode = [0, newNodeX, newNodeY, newNodeA, 0]
                    newCostToCome = previousCostToCome + i[3]
                    newNode[4] = newCostToCome
                    costToGo = self.eucd_dist(newNode)

                    if self.Obstacle.obst_flag(newNodeX, newNodeY):
                        if not self.Obstacle.check_visit(newNode):
                            presentNode[0] = newCostToCome
                            self.Obstacle.append_visited(newNode, presentNode[:4])
                            newNode[0] = newCostToCome + costToGo
                            heappush(self.Data, newNode)
                        else: 
                            previousVisited = self.Obstacle.visit_node(newNode)
                            previousCost = previousVisited[0]
                            if previousCost > newCostToCome:
                                presentNode[0] = newCostToCome
                                self.Obstacle.append_visited(newNode, presentNode[:4])
                                
        print("Error - Goal cannot be reached")
        return


Parser = argparse.ArgumentParser()
Parser.add_argument('--Start', default="[0,0,0]", help='start point')
Parser.add_argument('--End', default="[0,0,0]", help='goal point')
Parser.add_argument('--RobotRadius', default=int(input("Radius :")), help='robot radius')
Parser.add_argument('--Clearance', default=input("Clearance : "), help='robot clearance')
Parser.add_argument('--ShowAnimation', default=1, help='1 if want to show animation else 0')
Parser.add_argument('--Framerate', default=30, help='Will show next step after this many steps. Made for fast viewing')
Parser.add_argument('--thetaStep', default=int(input("Enter theta Angle :")), help='Possibilities of action for angle')
Parser.add_argument('--StepSize', default=input("Step Size : "), help='Step size')
Parser.add_argument('--Threshold', default=0.5, help='Threshold value for appriximation')
Parser.add_argument('--GoalThreshold', default=1.5, help='Circle radius for goal point')
Args = Parser.parse_args()
Xs=int(input("Xs : "))
Ys=int(input("Ys : "))
Xg=int(input("Xg : "))
Yg=int(input("Yg : "))
start=Args.Start
end=Args.End
r = int(Args.RobotRadius)
c = int(Args.Clearance)
animation = int(Args.ShowAnimation)
framerate = int(Args.Framerate)
thetaStep = int(Args.thetaStep)
StepSize = int(Args.StepSize)
Threshold = float(Args.Threshold)
GoalThreshold = float(Args.GoalThreshold)

initial = [int(i) for i in start[1:-1].split(',')]
goal = [int(i) for i in end[1:-1].split(',')]
initial[0]=Xs
initial[1]=Ys
goal[0]=Xg
goal[1]=Yg 
obj = Astar_algo(initial, goal, thetaStep=thetaStep, stepSize=StepSize,
        goalThreshold = GoalThreshold, width = 400, height = 250, threshold = Threshold,
        r=r, c=c)
obj.a_star()

