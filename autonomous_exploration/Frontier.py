import numpy as np
import math
expansion_size = 1

def costmap(data,width,height): #convert data from 1D array to 2D array 
    data = np.array(data).reshape(height,width)
    condition = np.logical_and(data > -1, data < 50)
    data = np.where(condition, 0, data) #make everything < 5e+01 but not -1 open space
    data = np.where(data > 50, 100, data) #make everything > 5e+01 but not -1 obstacle
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1) #make sure cells changing is within the map
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    
    return data

def find_frontier_cells(matrix):
    #go through entire matrix and mark frontier cells as 2 
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0: #open space 
                #convert cell into frontier cell 2 as long as an unmapped cell -1 is adjacent to it 
                if i > 0 and matrix[i-1][j] < 0: #top
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0: #bottom
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0: #left 
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0: #right 
                    matrix[i][j] = 2
    return matrix

def assign_groups(matrix):
    group_num = 1 #key in dictionary
    groups = {}
    for i in range(len(matrix)): #shd work with len -1 i think
        for j in range(len(matrix[0])):
            #do dfs to find neighbouring frontier cells 
            if matrix[i][j] == 2:
                group_num = dfs(matrix, i, j, group_num, groups)
    return groups

def dfs(matrix, i, j, group_num, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group_num
    if matrix[i][j] != 2:
        return group_num
    
    # if frontier cell found
    # if prev cell was also frontier append to same grp
    if group_num in groups:
        groups[group_num].append((i, j))
    
    # else start new group 
    else:
        groups[group_num] = [(i, j)]

    matrix[i][j] = 0 
    #change frontier cell back to open space cell so both functions dont search same area again
    dfs(matrix, i + 1, j, group_num, groups)
    dfs(matrix, i - 1, j, group_num, groups)
    dfs(matrix, i, j + 1, group_num, groups)
    dfs(matrix, i, j - 1, group_num, groups)
    dfs(matrix, i + 1, j + 1, group_num, groups) # bottom right 
    dfs(matrix, i - 1, j - 1, group_num, groups) # top left 
    dfs(matrix, i - 1, j + 1, group_num, groups) # top right 
    dfs(matrix, i + 1, j - 1, group_num, groups) # bottom left 

    #increase group number when all frontier cells in 1 grp found 
    return group_num + 1

def calculate_centroid(group):
    x_coords = []
    y_coords = []
    for cell in group:
        x_coords.append(cell[0])
        y_coords.append(cell[1])
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid 

def findClosesttoGoal(groups, target_x, target_y):
    centroids = []
    dists = []
    for group in groups:
        centroids.append(calculate_centroid(group))
    for centroid in centroids:
        dists.append(math.dist(centroid, (target_x, target_y)))
    min_index = dists.index(min(dists))
    return centroids[min_index][0], centroids[min_index][1]


