#!/usr/bin/env python


import math
import heapq
import time
import datetime
'''
    Global Definitions

    rhs,GRID,g, priorityQueue

    costmap (previous)
    plan (previous)
    start (previous)

'''

'''
    in make_plan():
    when dstarlite is called, first costmap change is identified:

    and start_node is checked


    ===== if costmap is changed =======
    1) scan for changed vertices
    2) D* Lite Replanning Process:
        for all directed edges (u, v) with changed edge costs
                Update the edge cost c(u, v); 
                UpdateVertex(u);
        ComputeShortestPath();
    3) 

    ===== if start_node is changed and costmap is not changed ======== ✅
    1) find the closest node in the previous plan for the start_node
    2) release the previous plan - upto current node
'''

previous_start = 0
previousGoal = 0
previousGridPlan  = []
previousPlan = []
previousCostMap = []
initialCostMap = []

absMaxChange = 0

notalreadyrun = 1

class Vertex:
    '''
        Defines a vertex by its grid position (x,y) and stores its key 
        K - Key has two values [k1,k2]
    '''
    def __init__(self, x, y, k1, k2):
        self.x  = int(x)
        self.y  = int(y)
        self.k1 = k1
        self.k2 = k2
        self.k  = [k1,k2]
        self.info = [k1,k2,[x,y]]

    def updateinfo(self):
        self.info = [self.k1, self.k2 , [self.x,self.y]]
km = 0

#define width 50
#define height 50

width  = 74
height = 74 #will be overwitten

rhs  = [] #!
GRID = [] #!
g    = [] #!

priorityQueue = [] #! create empty list

s_start = Vertex(0,0,0,0)
s_previous = Vertex(0,0,0,0)
s_last  = Vertex(0,0,0,0)
s_goal = Vertex(0,0,0,0)

previousVisits = []

def isVertexEqual(v1,v2):
    if(v1.x == v2.x and v1.y == v2.y):
        return 1
    return 0

def h(s1,s2):
    #heuristic function
    return math.sqrt((s1.x-s2.x)**2 + (s1.y-s2.y)**2)

def CalculateKey(s):
    global km
    global s_start
    global g
    global rhs

    k1 = min(g[s.x][s.y],rhs[s.x][s.y]) + h(s_start,s) + km
    k2 = min(g[s.x][s.y],rhs[s.x][s.y])

    s.k1 = k1
    s.k2 = k2
    s.updateinfo()
    #updatedVertex = Vertex(s.x,s.y,k1,k2)
    return s

def initialize():
    global priorityQueue
    global s_goal
    global km 
    global rhs
    global g

    priorityQueue = []
    heapq.heapify(priorityQueue) #heapify

    km = 0

    for i in range(0,width):
        for j in range(0,height):
            rhs[i][j] = float('inf')
            g[i][j]   = float('inf')
    
    rhs[s_goal.x][s_goal.y] = 0
    s_goal = CalculateKey(s_goal)
    heapq.heappush(priorityQueue,s_goal.info)

def cg_cost(a, b):
    global GRID
    global g

    if(b.x < 0 or b.x > width - 1 or b.y < 0 or b.y > height - 1): #checks if intended place is outof bounds
        return float('inf')


    blocked = GRID[a.x][a.y] + GRID[b.x][b.y]

    if(blocked > 0):
        return float('inf')
    else:
        cost =  math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2) + g[b.x][b.y]

        if(cost >= float('inf')):
            return float('inf')
        else:
            return cost

def removeIfExist(u):
    '''
        Checks if vertex u exists in Priority Queue, and removes it
    '''
    global priorityQueue
    for item in priorityQueue:
        if([u.x,u.y] == item[2]):
            priorityQueue.remove(item)
            priorityQueue.sort()
    
    return True
    
    #return False

def UpdateVertex(u):
    global priorityQueue
    global s_goal
    global width
    global height
    global rhs
    global g
    #check if out of bounds
    if(u.x < 0 or u.x >= width or u.y < 0 or u.y >= height):
        return
    if(not(isVertexEqual(u,s_goal))):
        c1 = float('inf')
        c2 = float('inf')
        c3 = float('inf')
        c4 = float('inf')
        c5 = float('inf')
        c6 = float('inf')
        c7 = float('inf')
        c8 = float('inf')

        if(u.y+1 > height):c1 = float('inf')
        else: c1 = cg_cost(u,Vertex(u.x,u.y+1,0,0))

        if(u.x+1 > width):c2 = float('inf')
        else: c2 = cg_cost(u,Vertex(u.x+1,u.y,0,0))

        if(u.y-1 < 0):c3 = float('inf')
        else: c3 = cg_cost(u,Vertex(u.x,u.y-1,0,0))

        if(u.x-1 < 0):c4 = float('inf')
        else: c4 = cg_cost(u,Vertex(u.x-1,u.y,0,0))

        if(u.x-1 < 0 or u.y - 1 < 0):c5 = float('inf')
        else: c5 = cg_cost(u,Vertex(u.x-1,u.y-1,0,0))

        if(u.x-1 < 0 or u.y + 1 > height): c6 = float('inf')
        else: c6 = cg_cost(u,Vertex(u.x-1,u.y+1,0,0))

        if(u.x + 1 > width or u.y - 1 < 0): c7 = float('inf')
        else: c7 = cg_cost(u,Vertex(u.x+1,u.y-1,0,0))

        if(u.x + 1 > width or u.y + 1 > height): c8 = float('inf')
        else: c8 = cg_cost(u,Vertex(u.x+1,u.y+1,0,0))

        rhs[u.x][u.y] = min(min(min(c3,c4),min(c1,c2)),min(min(c7,c8),min(c5,c6)))
    
    removeIfExist(u) #if U in PriorityQueue, remove it
    
    if(rhs[u.x][u.y]!=g[u.x][u.y]):
        u = CalculateKey(u)
        heapq.heappush(priorityQueue,u.info)

def isCostLower(b, a):  
    if(b.k1 < a.k1):
        return 1
    elif(a.k1 == b.k1):
        if(b.k2 < a.k2):
            return 1
        else:
            return 0;
    else:
        return 0;

def TopKey():
    '''
        Returns top vertex as the vertex class type
    '''
    global priorityQueue
    if(len(priorityQueue)==0):
        return Vertex(0,0,float('inf'),float('inf'));
    top_i = priorityQueue[0]

    top_vertex = Vertex(top_i[2][0],top_i[2][1],top_i[0],top_i[1])
    return top_vertex

def ComputeShortestPath():
    global priorityQueue
    global s_start
    global g
    global rhs

    while(isCostLower(TopKey(),CalculateKey(s_start)) or 
            rhs[s_start.x][s_start.y] != g[s_start.x][s_start.y]):

        k_old = TopKey()

        cell_index = getIndex(k_old.x,k_old.y)
        #print('size = '+ str(len(priorityQueue))+ " at - " + str(k_old.x)+','+str(k_old.y))

        heapq.heappop(priorityQueue)
        u     = k_old;

        #print('rhs,g ' + str(rhs[s_start.x][s_start.y])+','+str(g[s_start.x][s_start.y]))

        #if(k_old.k1 == float('inf') ): #break if path doesn't exist
        #    return;

        if(isCostLower(k_old,CalculateKey(u))):
            u = CalculateKey(u);
            heapq.heappush(priorityQueue,u.info)

        elif(g[u.x][u.y] > rhs[u.x][u.y]):
            g[u.x][u.y] = rhs[u.x][u.y];

            UpdateVertex(Vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y+1,0,0));

            UpdateVertex(Vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(Vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x-1 ,u.y  ,0,0));


        else:
            g[u.x][u.y] = float('inf');

            UpdateVertex(Vertex(u.x   ,u.y  ,0,0));

            UpdateVertex(Vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(Vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x-1 ,u.y  ,0,0));

            UpdateVertex(Vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y+1,0,0));

def getIndex(i,j):
    return j*width + i;

def convertToCellIndex(x,y,resolution):
    cellIndex = 0
    newX = x/resolution
    newY = y/resolution

    cellIndex = getIndex(newY,newX)
    return int(cellIndex)

def getRow(index):
    return int(index % width)
    
def getCol(index):
    return index//width

def outofbounds(v):
    if(v.x < 0 or v.x >= height or v.y < 0 or v.y >=width ):
        return 1
    else:
        return 0

def onestep():
    
    '''
       Selecting the next lowest cost node and assigns it to s_start
    '''

    moves = [[-1,0],[0,-1],[1,0],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]]

    global s_start
    global s_goal
    global s_previous
    global previousVisits

    
    #print("currently -> "+str(s_start.x) + "," + str(s_start.y) + ' => going to '+str(s_goal.x) + "," + str(s_goal.y))



    if(s_start.x == s_goal.x and s_start.y == s_goal.y):
        return 0

    arr = []

    for i in range(0,len(moves)):
        next_step = [s_start.x + moves[i][0],s_start.y + moves[i][1]]
        if(next_step in previousVisits):
            arr.append(float('inf'))
        else:
            arr.append(cg_cost(s_start, Vertex(s_start.x + moves[i][0] ,s_start.y + moves[i][1],0,0)))
 
    min_index = arr.index(min(arr))

    previousVisits.append([s_start.x,s_start.y])

    s_start.x = s_start.x + moves[min_index][0]
    s_start.y = s_start.y + moves[min_index][1]

    
    if(not(outofbounds(s_start)) and arr[min_index]!=float('inf')):       
        return 1
    else:
        print("went out of bounds")
        return 0

def d_star_lite(start_index, goal_index, width_, height_, costmap, resolution, origin):
    ''' 
    Performs DStarLite's shortes path algorithm search on a costmap with a given start and goal node
    '''
    global s_start
    global s_last
    global s_goal
    global width
    global height
    global rhs
    global GRID
    global g
    global previousVisits

    width = width_
    height = height_

    rhs  = [[0 for x in range(width)] for y in range(height)] 
    GRID = [[0 for x in range(width)] for y in range(height)] 
    g    = [[0 for x in range(width)] for y in range(height)] 


    print('DStarLite: Starting... ')
    print('width ' + str(width))
    print('height ' + str(height))


    s_start = Vertex(getRow(start_index),getCol(start_index),0,0)
    s_last  = Vertex(s_start.x,s_start.y,0,0)
    s_goal  = Vertex(getRow(goal_index),getCol(goal_index),0,0)

    print("start = " + str(s_start.x) +','+ str(s_start.y))
    print("goal  = " + str(s_goal.x)  +','+ str(s_goal.y))

    initialize()

    #print(costmap)

    x = 0
    for row in range(0,height):
        for col in range(0,width):
            if(costmap[x] > 70):
                GRID[col][row] = 1
            else:
                GRID[col][row] = 0
            x+=1
    '''
    for i in range(0,height):
        for j in range(0,width):
            if(i == s_goal.x and j == s_goal.y):
                print("X",end="")
            elif(i == s_start.x and j == s_start.y):
                print("S",end="")
            else:
                print(GRID[i][j],end="")
        print()
    '''
    '''
    output = []
    for i in range(0,width):
        output.append([])
        for j in range(0,height):
            output[i].append(str(GRID[i][j]))

    for row in output:
        print(row)
    '''

    for row in range(0,height):
        for col in range(0,width):
            print(GRID[row][col],end="")
        print()
    print('DStarLite: Done with initialization')

    ComputeShortestPath()

    bestPath = []
    gridPath = []

    #if(g[s_start.x][s_start.y]!=float('inf')):
    print("DStarLite: Going to construct Path")
    previousVisits = []
    print(" - previousVisits <=====>"+ str(previousVisits))
    while(onestep()):
        bestPath.append(getIndex(s_start.x,s_start.y))
        gridPath.append([s_start.x,s_start.y])

    print('DStarLite: Done reconstructing path')

    return bestPath , gridPath


def h_(x,y,s2,s_goal):
  #heuristic function //can add a weight
  return math.sqrt((x-s2.x)**2 + (y-s2.y)**2)#*2 + math.sqrt((x-s_goal.x)**2 + (y-s_goal.y)**2)

def costChange(map1, map2):
    cost = 0

    for i in range(0,len(map1)):
        if(abs(map1[i]-map2[i]) > 225):
            cost += (map1[i]-map2[i])**2

    return math.sqrt(cost)

def detectChanges(map1, map2):
  global GRID
  global initialCostMap

  initialCostMap = list(initialCostMap)

  changed_nodes = []
      
  x = 0
  for row in range(0,height):
    for col in range(0,width):
      if(abs(map1[x]-map2[x]) > 100):
        GRID[col][row] = 1
        #print(str(type(initialCostMap)) + " , " + str(type(map1)))
        initialCostMap[x] = map1[x]
        current_vertex = Vertex(col,row,0,0)
        changed_nodes.append(current_vertex)
      x+=1


  for row in range(0,height):
    for col in range(0,width):
      print(GRID[row][col],end="")
    print()
      

  return changed_nodes


def modify_plan(GridPlan,costmap,start_index,goal_index):
  '''
    extract path based on curernt start_index
  '''
  global previousPlan
  global previousGridPlan
  global previousCostMap
  global absMaxChange
  global initialCostMap
  global s_last
  global s_start
  global km
  global notalreadyrun
  global priorityQueue
  global g
  global previousVisits
  global previous_start

  current_start = Vertex(getRow(start_index),getCol(start_index),0,0)

  #start_index = previous_start + 1
  #print(" gridPlan[0] = " + str(GridPlan[0][0]) + ","+ str(GridPlan[0][1])+ " , Actual start " +  str(getRow(previous_start)) + "," + str(getCol(previous_start)) )
 

  s_start = Vertex(getRow(start_index),getCol(start_index),0,0)
  s_goal  = Vertex(getRow(goal_index),getCol(goal_index),0,0)

  

  minIndex = 0
  minDist  = float('inf')

  for i in range(0,len(GridPlan)):
    #find the nearest starting vertex of already generated plan 
    # (since starting vertex has changed due to local planner)
    dist = h_(GridPlan[i][0],GridPlan[i][1],current_start,s_goal)

    if(dist < minDist):
        minDist = dist
        minIndex = i

  change = 0
  if(costmap != previousCostMap):
    #Scan Cost Map for changes
    #print(" ===== COST MAP CHANGED ! ====== ")
    
    change = costChange(costmap, initialCostMap)
    if (change > absMaxChange):
        absMaxChange = change
    #print(" ======> Change Value " + str(change) + " , Max Change: " + str(absMaxChange))

  #print(" Actual Start = " + str(getRow(start_index)) + ","+ str(getCol(start_index))+ " , Initial start " +  str(getRow(previous_start)) + "," + str(getCol(previous_start)) )
  #print(" Closest to Path = " + str(getRow(previousPlan[minIndex])) + "," + str(getCol(previousPlan[minIndex])))

  #if(h_(GridPlan[minIndex][0],GridPlan[minIndex][1],s_start,s_goal) > 2):
  #  print(" ~!!!!!!!! Too far")
  #
  

  if(change > 200): ## If Map has changed => Reconstruct Path
    # D* lite Replanning
    changed_nodes = detectChanges(costmap, initialCostMap) #get changed nodes list

    #s_start = Vertex(GridPlan[minIndex][0],GridPlan[minIndex][1],0,0)

    km = km + h(s_last,s_start)
    s_last = Vertex(s_start.x,s_start.y,0,0)

    '''
    for i in range(0,height):
        for j in range(0,width):
            if(g[i][j]==float("inf")):
                print("XX",end=",")
            else:
                if(g[i][j]<10):
                    print(" "+str(int(g[i][j])),end=",")
                else:
                    print(int(g[i][j]),end=",")
        print()
    '''
    print(" =>>>> " + str(len(changed_nodes))+ " Vertices Changed!")
    for i in range(0,len(changed_nodes)): #for all changed vertices, updateVertex()
      current_vertex = changed_nodes[i]
      UpdateVertex(current_vertex)
      print(" ===> changed vertex: (" + str(current_vertex.x) + ',' + str(current_vertex.y)+")")
    

    #print(" > key of s_start " + str(CalculateKey(s_start).info))
    #print(str(priorityQueue))
    ComputeShortestPath();

    #print(" > key of s_start " + str(CalculateKey(s_start).info))
    #print("<== Priority Queue After the Change ===>")
    #print(str(priorityQueue))

    bestPath = []
    gridPath = []

    #print("DStarLite: Going to Re-construct Path <=====>")
    previousVisits = []
    #print(" - previousVisits <=====>"+ str(previousVisits))

    #print(" g[start] " + str(g[s_start.x][s_start.y]))
    
    if(g[s_start.x][s_start.y]!=float('inf')):
        while(onestep()):
            bestPath.append(getIndex(s_start.x,s_start.y))
            gridPath.append([s_start.x,s_start.y])
            #print("==> next "+str(s_start.x) + "," + str(s_start.y)+' g value = ' + str(g[s_start.x][s_start.y]))
        #print("DStarLite: Re-Constructed the path <=====>")
    else:
        print(" g[start] is infinity " + str(g[s_start.x][s_start.y]))
    
    previousGridPlan = gridPath ### Update both since we reconstructed the path
    previousPlan     = bestPath
    
    for i in range(0,height):
        for j in range(0,width):
            if(g[i][j]==float("inf")):
                print("XX",end=",")
            else:
                if(g[i][j]<10):
                    print(" "+str(int(g[i][j])),end=",")
                else:
                    print(int(g[i][j]),end=",")
        print()

    if(bestPath[-1]==goal_index):
        return bestPath 

    else:
        print(" Path not found")
        return []
  return previousPlan[minIndex::]
  #return []


def make_plan(costmap,start_index):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  global previous_start
  global previousPlan
  global previousGridPlan
  global previousCostMap
  global previousGoal
  global initialCostMap
  global absMaxChange

  # costmap as 1-D array representation
  
    
  # number of columns in the occupancy grid
  width = 74
  # number of rows in the occupancy grid
  height = 74
  #start_index = 1874
  goal_index = 2419
  # side of each grid map square in meters
  resolution = 0.2
  # origin of grid map
  origin = [-7.4, -7.4, 0]

  #viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)

  # time statistics
  start_time = datetime.datetime.now()

  # calculate the shortes path using DStarLite

  

  if(previousGoal!=goal_index):
    print('========== Calling D*Lite! ============')
    path , previousGridPlan = d_star_lite(start_index, goal_index, width, height, costmap, resolution, origin)
    previousPlan = path 
    previousGoal = goal_index
    previous_start  = start_index
    initialCostMap = costmap
    absMaxChange = 0 #measure again
    print('===> ' + str(path) + ' <===' )
  else:
    print('========== Modify path ============')
    path  = modify_plan(previousGridPlan,costmap,start_index,goal_index) ## extract path
    print('===> ' + str(path) + ' <===' )

  
  previousCostMap = costmap


  if not path:
    rospy.logwarn("No path returned by DStarLite's shortes path algorithm")
    path = []
  else:
    execution_time = datetime.datetime.now() - start_time
    print("\n")
    #print('++++++++ DStarLite execution metrics ++++++++')
    print('Total execution time: %s seconds', str(execution_time))
    #print('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    print('DStarLite: Path sent to navigation stack')

  
  return path

costmap_before = (255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 163, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 163, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 254, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 71, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 163, 22, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 255, 255, 255, 255, 254, 163, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 163, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 163, 22, 163, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 163, 22, 163, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 163, 163, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 163, 254, 255, 255, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 254, 255, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255)

costmap_after = (255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 163, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 163, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 254, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 254, 163, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 0, 22, 71, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 163, 163, 163, 163, 163, 163, 163, 71, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 0, 71, 163, 163, 254, 254, 254, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 254, 163, 22, 22, 163, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 71, 0, 22, 163, 254, 255, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 254, 163, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 254, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 255, 255, 255, 255, 254, 163, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 163, 71, 0, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 22, 163, 254, 254, 163, 22, 163, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 163, 22, 22, 22, 0, 0, 0, 71, 163, 254, 163, 22, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 163, 163, 163, 163, 71, 0, 22, 163, 254, 254, 163, 163, 163, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 163, 71, 22, 163, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 163, 22, 71, 163, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 163, 22, 0, 22, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 254, 254, 254, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 71, 163, 163, 163, 163, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 22, 22, 22, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 163, 22, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 71, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 71, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 163, 254, 255, 255, 255, 254, 163, 22, 0, 22, 163, 254, 255, 255, 254, 163, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 163, 254, 255, 255, 255, 254, 163, 22, 22, 22, 163, 254, 255, 255, 254, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 254, 255, 255, 255, 254, 163, 163, 163, 163, 163, 254, 255, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255)

if __name__ == '__main__':

    print(make_plan(costmap_before,1650))
    for i in range(0,height):
        for j in range(0,width):
            if(g[i][j]==float("inf")):
                print("XX",end=",")
            else:
                if(g[i][j]<10):
                    print(" "+str(int(g[i][j])),end=",")
                else:
                    print(int(g[i][j]),end=",")
        print()


    print(make_plan(costmap_after,1650))
