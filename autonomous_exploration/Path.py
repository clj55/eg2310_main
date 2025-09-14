
import numpy as np
import heapq
import math

def heuristic(a, b):
    return math.dist(a, b)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 100:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        print("no path to goal")
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False