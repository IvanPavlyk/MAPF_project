from queue import Queue
def minimumVertexCoverHelper(CG, num_of_agents):
        rst = 0
        done = [False] * num_of_agents
        i = 0
        while i < num_of_agents:
            if done[i]:
                i+=1
                continue
            indices = []
            Q = Queue()
            Q.put(i)
            done[i] = True
            while not Q.empty():
                j = Q.get()
                indices.append(j)
                k = 0
                while k < num_of_agents:
                    if CG[j * num_of_agents + k] > 0:
                        if not done[k]:
                            Q.put(k)
                            done[k] = True
                    elif CG[k * num_of_agents + j] > 0:
                        if not done[k]:
                            Q.put(k)
                            done[k] = True
                    k += 1
            if len(indices) == 1:
                i+=1
                continue
            elif len(indices) == 2:
                rst += 1
                i+=1
                continue

            subgraph = [0] * (len(indices) * len(indices))
            num_edges = 0
            j = 0
            while j < len(indices):
                k = j + 1
                while k < len(indices):
                    subgraph[j * len(indices) + k] = CG[indices[j] * num_of_agents + indices[k]]
                    subgraph[k * len(indices) + j] = CG[indices[k] * num_of_agents + indices[j]]
                    if subgraph[j * len(indices) + k] > 0:
                        num_edges += 1
                    k += 1
                j += 1
            if len(indices) > 8:
                rst += greedyMatching(subgraph, len(indices))
            else:
                i = 1
                while i < len(indices):
                    if KVertexCover(subgraph, len(indices), num_edges, i, len(indices)):
                        rst += i
                        break
                    i += 1
            i += 1
        return rst

def minimumVertexCover(CG, old_mvc, cols, num_of_CGedges):
    assert old_mvc >= 0
    rst = 0
    if num_of_CGedges < 2:
        return num_of_CGedges
    num_of_CGnodes = 0
    for i in range(0, cols):
        for j in range(0, cols):
            if CG[i * cols + j] > 0:
                num_of_CGnodes += 1
                break
    if num_of_CGnodes > 8:
        return minimumVertexCoverHelper(CG, cols)
    else:
        if KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc - 1, cols):
            rst = old_mvc - 1
        elif KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc, cols):
            rst = old_mvc
        else:
            rst = old_mvc + 1
    return rst

def greedyMatching(CG, cols):
    rst = 0
    used = [False] * cols
    while True:
        maxWeight = 0
        ep1 = None
        ep2 = None
        for i in range(0, cols):
            if used[i]:
                continue
            for j in range(i + 1, cols):
                if used[j]:
                    continue
                elif maxWeight < CG[i * cols + j]:
                    maxWeight = CG[i * cols + j]
                    ep1 = i
                    ep2 = j
        if maxWeight == 0:
            return rst
        rst += maxWeight
        used[ep1] = True
        used[ep2] = True

def KVertexCover(CG, num_of_CGnodes, num_of_CGedges, k, cols):
    if num_of_CGedges == 0:
        return True
    elif num_of_CGedges > k * num_of_CGnodes - k:
        return False
    node = [0 for _ in range(2)]
    flag = True
    i = 0
    while i < cols - 1 and flag:
        j = i + 1
        while j < cols and flag:
            if CG[i * cols + j] > 0:
                node[0] = i
                node[1] = j
                flag = False
            j += 1
        i += 1

    for i in range(0, 2):
        CG_copy = CG.copy()
        num_of_CGedges_copy = num_of_CGedges
        for j in range(0, cols):
            if CG_copy[node[i] * cols + j] > 0:
                CG_copy[node[i] * cols + j] = 0
                CG_copy[j * cols + node[i]] = 0
                num_of_CGedges_copy -= 1
        if KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1, cols):
            return True
    return False