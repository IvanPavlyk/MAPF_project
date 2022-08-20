#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from icbs import ICBSSolver

from results import Results
from visualize import Animation
from common_for_search import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--h', type=int, default=0,
                        help='The heuristic to use (one of: {none,cg,dg,wdg}), defaults to none')
    parser.add_argument('--r', type=int, default=0,
                        help='The number of repeats')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()


    result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)
        global_cost = 0
        results = Results()
        for i in range(args.r):
            if args.solver == "CBS":
                print("***Run CBS***")
                cbs = CBSSolver(my_map, starts, goals)

                time, expanded, generated, paths = cbs.find_solution(args.disjoint)
                results.addValues(time, expanded, generated)
                global_cost += get_sum_of_cost(paths)
            
            elif args.solver == "ICBS":
                print("***Run ICBS***")
                icbs =  ICBSSolver(my_map, starts, goals)
                time, expanded, generated, paths, h_ave = icbs.find_solution(args.disjoint, args.h)
                results.addValues(time, expanded, generated,h_ave)
                global_cost += get_sum_of_cost(paths)        
            else:
                raise RuntimeError("Unknown solver!")

        print("Global average cost of paths: ", global_cost/args.r)
        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))
        results.printResults(args.r)

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            animation.show()
    result_file.close()