from sys import argv, maxsize
from itertools import permutations
from math import sqrt

#########################
#helper functions


def euclidean_distance(x, y):
    return sqrt((y[0]-x[0])*(y[0]-x[0])+(y[1]-x[1])*(y[1]-x[1]))


def discrete_distance(x, y):
    return (y[0]-x[0])+(y[1]-x[1])


#########################
#parsing arguments
# argument structure:<whitespace><1|0:=return_to_start|dont_return_tostart>
# <whitespace><first_coordinate_start><second_coordinate_start><whitespace>
# <first_coordinate_start+1><second_coordinate_start+1><whitespace>
#...<whitespace><first_coordinate_start+N><second_coordinate_start+N>
if len(argv) > 1:
    coords = []
    return_to_start = int(argv[1])
    start = (int(argv[2][0]), int(argv[2][1]))
    for arg in argv[3:]:
        coords.append((int(arg[0]), int(arg[1])))

    #########################
    #greedy tsp solver algorithm
    #all permutations get checked
    min_path_cost = maxsize
    res_path = []
    path_permutations = permutations(coords)
    for path in path_permutations:
        current_path_cost = euclidean_distance(start, path[0])
        for i, point in enumerate(path[:-1]):
            current_path_cost += euclidean_distance(point, path[i+1])
        if return_to_start:
            current_path_cost += euclidean_distance(path[-1], start)
        if current_path_cost < min_path_cost:
            res_path = path
            min_path_cost = current_path_cost
        #print(path)
        #print(current_path_cost)

    #####################
    #print(list(res_path))
    #return results to console with starting point at start
    res_path = [start] + list(res_path)
    if return_to_start:
        res_path += [start]
    #print("result")
    #print(res_path)
    for v in res_path[:-1]:
        print(f"{v[0]}{v[1]} ", end='')
    print(f"{res_path[-1][0]}{res_path[-1][1]}")
    #print(min_path_cost)
