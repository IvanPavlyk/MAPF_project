import sys
from queue import Queue
from minimumVertexCover import greedyMatching

MAX_COST = sys.maxsize/2

def minimumWeightedVertexCover(HG, num_of_agents):
    rst = weightedVertexCover(HG, num_of_agents)
    return rst


def weightedVertexCover(CG, num_of_agents):
    rst = 0
    done = [False] * num_of_agents
    i = 0
    while i < num_of_agents:
        if done[i]:
            i += 1
            continue
        range_list = []
        indices = []
        num = 0
        num_states = 1
        Q = Queue()
        Q.put(i)
        done[i] = True
        while not Q.empty():
            j = Q.get()
            range_list.append(0)
            indices.append(j)
            k = 0
            while k < num_of_agents:
                if CG[j * num_of_agents + k] > 0:
                    range_list[num] = max(range_list[num], CG[j * num_of_agents + k])
                    if not done[k]:
                        Q.put(k)
                        done[k] = True
                elif CG[k * num_of_agents + j] > 0:
                    range_list[num] = max(range_list[num], CG[k * num_of_agents + j])
                    if not done[k]:
                        Q.put(k)
                        done[k] = True
                k += 1
            num += 1
        if num == 1:# no edges
            i+=1
            continue
        elif num == 2:  # only one edge
            rst += max(CG[indices[0] * num_of_agents + indices[1]],
                       CG[indices[1] * num_of_agents + indices[0]])  # add edge weight
            i += 1
            continue
        G = [0] * (num*num)
        for j in range(0, num):
            for k in range(j + 1, num):
                G[j * num + k] = max(CG[indices[j] * num_of_agents + indices[k]],
                                     CG[indices[k] * num_of_agents + indices[j]])
        if num > 8:  # solve by greedy matching
            rst += greedyMatching(G, len(range_list))
        else:  # solve by dynamic programming
            x = [0] * num
            best_so_far = MAX_COST
            rst += DPForWMVC(x, 0, 0, G, range_list, best_so_far)
        i += 1
    return rst

def DPForWMVC(x, i, sum_costs, CG, range_list, best_so_far):
    if sum_costs >= best_so_far:
        return MAX_COST
    elif i == int(len(x)):
        best_so_far = sum_costs
        return sum_costs
    elif range_list[i] == 0: # vertex i does not have any edges.
        rst = DPForWMVC(x, i + 1, sum_costs, CG, range_list, best_so_far)
        if rst < best_so_far.arg_value:
            best_so_far = rst
        return best_so_far
    cols = len(x)
    # find minimum cost for this vertex
    min_cost = 0
    for j in range_list(0, i):
        if min_cost + x[j] < CG[j * cols + i]: # infeasible assignment
            min_cost = CG[j * cols + i] - x[j] # cost should be at least CG[i][j] - x[j];
    best_cost = -1
    cost = min_cost
    while cost <= range_list[i]:
        x[i] = cost
        rst = DPForWMVC(x, i + 1, sum_costs + x[i], CG, range_list, best_so_far)
        if rst < best_so_far:
            best_so_far = rst
            best_cost = cost
        cost += 1
    if best_cost >= 0:
        x[i] = best_cost
    return best_so_far
