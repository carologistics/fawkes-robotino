import mlrose_hiive as mlrose
import numpy as np
from sys import argv

#########################
#parsing arguments
# argument structure:<whitespace>
# <whitespace><first_coordinate_start><second_coordinate_start><whitespace>
# <first_coordinate_start+1><second_coordinate_start+1><whitespace>
#...<whitespace><first_coordinate_start+N><second_coordinate_start+N>
if len(argv) > 1:
    coords = []
    for arg in argv[2:]:
        coords.append((int(arg[0]), int(arg[1])))

    #coords = np.array(coords)

    problem_no_fit = mlrose.TSPOpt(length = len(coords), coords=coords, maximize=False)

    best_state, best_fitness, _ = mlrose.genetic_alg(problem_no_fit, random_state=2)

    print(f"{coords[0][0]}{coords[0][1]} ", end='')
    for v in best_state[:-1]:
        print(f"{coords[v][0]}{coords[v][1]} ", end='')
    print(f"{coords[best_state[-1]][0]}{coords[best_state[-1]][1]}")
    print(best_fitness)
