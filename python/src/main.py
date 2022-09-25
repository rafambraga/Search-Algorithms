#! /usr/bin/env python3

from environment import Environment
from vector2d import Vector2D
import math


class objt:
    
    vertex = Vector2D
    cost = 0
    
    def __init__(self, vert, cost):
        self.vertex = vert
        self.cost = cost


class p:
    
    pList = []
    
    def __init__(self, env):
        pList = []
    
    def removeSm(self):
    
        sm = Vector2D(0,0)
        smallest = objt(sm, math.inf)
        
        for obj in self.pList:
            if obj.cost <= smallest.cost:
                smallest = obj
        
        self.pList.remove(smallest)
        
        return smallest
  
      
def ccw(v1, v2, v3):
    return (v3.y - v1.y) * (v2.x - v1.x) > (v2.y - v1.y) * (v3.x - v1.x)


def intersect(v1, v2, v3, v4):
    return ccw(v1, v3, v4) != ccw(v2, v3, v4) and ccw(v1, v2, v3) != ccw(v1, v2, v4)


def distance(startVert, endVert):
    
    disX = endVert.x - startVert.x
    disY = endVert.y - startVert.y
    
    return math.sqrt(pow(disX,2)+pow(disY,2))


def checkPolygon(vertices, start, end):
 
    for index in range(len(vertices)):
        
        if((index == len(vertices) - 1) and (vertices[index]==start) and (vertices[0] == end)):
            return True
        
        if((vertices[0] == start) and (vertices[len(vertices) - 1] == end)):
            return True
        
        if((index != 0) or (index != (len(vertices) - 1))):
            if((vertices[index] == start) and (vertices[index - 1]==end)):
                return True
            if((vertices[index] == start) and (vertices[index + 1]==end)):
               return True
        
        return False
            
                                                          
def intersection(env, start, end):
     
     obstacles = env.obstacles
     
     for ply in obstacles:
                      
           for index in range(len(ply.vertices)):
                if(ply.vertices[index] == start): 
                    if(checkPolygon(ply.vertices, start, end)):
                        return True
                    else:
                        continue
                
                if(index == len(ply.vertices) -1):
                    if(intersect(start, end, ply.vertices[index], ply.vertices[0])):
                        return False
                    else:
                        break
                
                elif(intersect(start, end, ply.vertices[index], ply.vertices[index+1])):
                    return False
     return True


def visited(vertex, checkList):
    for item in checkList:
        if(item == vertex):
            return True
    return False


def greedySearch(env):
    
    start = env.start
    end = env.goal
    path = []
    options = []
    curr = start
    moves = 0
    totalDistance = 0
    
    while(moves < env.verts):
        
        options = []
        
        for ply in env.obstacles:
            for vert in ply.vertices:
                if(intersection(env, curr, vert)):
                    options.append(vert)
        
        if(intersection(env, curr, end)):
            options.append(end)
        
        greedChoice = curr
        
        for vertex in options:
            if((distance(vertex, end) <= distance(greedChoice, end)) and (visited(curr, path) == False)):
                greedChoice = vertex
        
        options.clear()
        path.append(curr)
        totalDistance = totalDistance + distance(greedChoice, curr)
        curr = greedChoice
        print ("Coordinates: (", '%.0f'%curr.x, ",", '%.0f'%curr.y, ")")
        
        if(curr == end):
            path.append(curr)
            print ("The total cost was: ", '%.0f'%totalDistance)
            return path
        moves += 1
    
    
    
    
def uniformCostSearch(env):
    
    prioq = p(env)
    nodeList = []
    retrace = []
    end = objt(env.goal,math.inf)
    path = []
    start = objt(env.start, 0.0)
    prioq.pList.append(start)
    moves = 0
    totalDistance = 0
    
    for ply in env.obstacles:
        for vert in ply.vertices:
            addition = objt(vert, math.inf)
            nodeList.append(addition)
    
    while(moves < env.verts):        
        
        curr = prioq.removeSm()
        retrace.append(curr)
        
        if (curr == end):
            break
        
        for obj in nodeList:
            if(intersection(env, curr.vertex, obj.vertex) and (obj.cost == math.inf)):
                obj.cost = curr.cost + distance(curr.vertex, obj.vertex)
                prioq.pList.append(obj)
        
        if(intersection(env, curr.vertex, end.vertex)):
            end.cost = curr.cost + distance(curr.vertex, end.vertex)
            prioq.pList.append(end)
        moves += 1

    current = retrace[len(retrace)-1]
    
    for index in range(len(retrace)):
        
        path.append(current.vertex)
        
        for objec in retrace:
            if(((current.cost - distance(current.vertex, objec.vertex)) <= (objec.cost + .01)) and
               ((current.cost - distance(current.vertex, objec.vertex)) >= (objec.cost - .01))):
                
                totalDistance = totalDistance + distance(current.vertex, objec.vertex)
                
                current = objec
                
                print("Coordinates: (", '%.0f'%current.vertex.x, ",", '%.0f'%current.vertex.y, ")\t Path Cost: ", '%.0f'%current.cost)
                break
        
        if(current == start):
            path.append(start.vertex)
            break
                
    path.reverse()
    print ("The total cost was: ", '%.0f'%totalDistance)
    return path


def astarSearch(env):
    
    prioq = p(env)
    prioq.pList.clear()
    nodeList = []
    nodeList.clear()
    retrace = []
    retrace.clear()
    end = objt(env.goal, math.inf)
    path = []
    path.clear()
    start = objt(env.start,0.0)
    start.cost = distance(start.vertex, end.vertex)
    prioq.pList.append(start)
    moves = 0
    totalDistance = 0
    
    for ply in env.obstacles:
        for vert in ply.vertices:
            addition = objt(vert, math.inf)
            nodeList.append(addition)
    
    while(moves < env.verts):        
        
        curr = prioq.removeSm()
        retrace.append(curr)
        
        if (curr == end):
            break
        
        for obj in nodeList:
            if(intersection(env, curr.vertex, obj.vertex) and (obj.cost == math.inf)):
                obj.cost = curr.cost + distance(curr.vertex, obj.vertex) + distance(obj.vertex, end.vertex)
                prioq.pList.append(obj)
        
        if(intersection(env, curr.vertex,end.vertex)):
            end.cost = curr.cost + distance(curr.vertex, end.vertex)
            prioq.pList.append(end)
        moves += 1

    current = retrace[len(retrace)-1]
    
    for index in range(len(retrace)):
        path.append(current.vertex)
       
        for objec in retrace:
            if(((current.cost - (distance(current.vertex,objec.vertex) + distance(current.vertex,end.vertex))) <= (objec.cost +.01)) and 
                ((current.cost - (distance(current.vertex,objec.vertex) + distance(current.vertex,end.vertex))) >= (objec.cost -.01))):
                
                totalDistance = totalDistance + distance(current.vertex, objec.vertex)
                
                current = objec
                
                print("Coordinates: (", '%.0f'%current.vertex.x, ",", '%.0f'%current.vertex.y, ")\t Path Cost: ", '%.0f'%current.cost)
                break
        
        if(current == start):
            path.append(start.vertex)
            break
                
    path.reverse()
    print ("The total cost was: ", '%.0f'%totalDistance)
    return path

if __name__ == '__main__':
    env = Environment('output/environment.txt')
    print("Loaded an environment with {} obstacles.".format(len(env.obstacles)))

    searches = {
        'greedy': greedySearch,
        'uniformcost': uniformCostSearch,
        'astar': astarSearch
    }

    for name, fun in searches.items():
        print("\n", "Attempting a search with " + name)
        Environment.printPath(name, fun(env))
