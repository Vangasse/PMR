import matplotlib.pyplot as plt
import networkx as nx
import heapq as hq
import numpy as np
import os

#######################
###### FUNCTIONS ######
#######################


def a_star_PRM(G, start, goal):
    visited = {}
    queue = [(get_distance(start,goal), start, start)]

    hq.heapify(queue)
    while queue:
        node = hq.heappop(queue)
        
        if goal in visited and visited[goal][0] < node[0]:
            break
        
        heuristica = get_distance(start,goal)

        if node[1] in visited:
            if node[0] - heuristica < visited[node[1]][0]:
                visited[node[1]] = (node[0] - heuristica, node[2])
            else:
                pass
        else:
            visited[node[1]] = (node[0] - heuristica, node[2])

        neighbors = list(G[node[1]].keys())

        for n in neighbors:
            if not n in visited:
                heuristica = get_distance(start,goal)
                hq.heappush(queue, (G[node[1]][n]['weight'] + visited[node[1]][0] + heuristica, n, node[1]))

    res = [goal]

    searcher = goal

    while searcher != start:
        res.append(visited.get(searcher)[1])
        searcher = visited.get(searcher)[1]

    res.reverse()

    return res

def a_star(G, start, goal, graph_input):
    visited = {}
    queue = [(get_distance(graph_input[start][0],graph_input[goal][0]), start, start)]

    hq.heapify(queue)
    while queue:
        node = hq.heappop(queue)
        
        if goal in visited and visited[goal][0] < node[0]:
            break
        
        heuristica = get_distance(graph_input[node[1]][0],graph_input[goal][0])

        if node[1] in visited:
            if node[0] - heuristica < visited[node[1]][0]:
                visited[node[1]] = (node[0] - heuristica, node[2])
            else:
                pass
        else:
            visited[node[1]] = (node[0] - heuristica, node[2])

        neighbors = list(G[node[1]].keys())

        for n in neighbors:
            if not n in visited:
                heuristica = get_distance(graph_input[n][0],graph_input[goal][0])
                hq.heappush(queue, (G[node[1]][n]['weight'] + visited[node[1]][0] + heuristica, n, node[1]))

    res = [goal]

    searcher = goal

    while searcher != start:
        res.append(visited.get(searcher)[1])
        searcher = visited.get(searcher)[1]

    res.reverse()

    return res

def get_distance(a,b):
    return np.sqrt((a[0]- b[0])**2 + (a[1]- b[1])**2)

def dict_to_graph(dicto,nodes):

    G = nx.Graph()

    for i in nodes:
        for o in dicto[i][1]:
            weight0 = get_distance(dicto[i][0], dicto[o][0])
            G.add_edge(i, o, weight = round(weight0, 3))

    return G


def discrete_to_continuos(nodes):
    for i in range(len(nodes) - 1):
        path[0] = np.hstack((path[0] , np.linspace(nodes[i][0],nodes[i+1][0],5)))
        path[1] = np.hstack((path[1] , np.linspace(nodes[i][1],nodes[i+1][1],5)))
   
    return path 




#####################
######## PATH #######
#####################

# Dictionary
graph_input = {"start": [[0,-1],["c"]], 
"a": [[0,0],["start","b","d"]], 
"b": [[-1,0],["k"]], 
"c": [[-0.2,-1.5],["e"]], 
"d": [[1,1],["l","m","n"]], 
"e": [[-1,-3],["f"]], 
"f": [[-3,-3.5],["g"]], 
"g": [[-3.5,-3],["goal"]], 
"h": [[-4,0],["j","i"]], 
"i": [[-3.5,1],["j"]], 
"j": [[-3,0],["k"]], 
"k": [[-2,0.5],["b"]], 
"l": [[2,2],["m","n"]], 
"m": [[3,1.5],["o"]], 
"n": [[2,3.5],["o"]], 
"o": [[3.5,2.5],[]], 
"goal": [[-4,-2],["h"]]}

# graph_input = {"o": [[0,0],["a","b","c"]], 
# "a": [[-1,1],["d","e"]], 
# "b": [[1,0.5],["d","f"]], 
# "c": [[0,-1.5],[]], 
# "d": [[0.5,1.5],["e","g"]], 
# "e": [[-1,3],["g"]], 
# "f": [[3,0],["h"]], 
# "g": [[1.2,3.5],["i"]], 
# "h": [[4,-0.4],["j"]], 
# "i": [[3.1,4.2],["l","k"]], 
# "j": [[5,0.3],["k"]], 
# "k": [[5,2],["l"]], 
# "l": [[5.2,4.1],["goal"]], 
# "goal": [[5.8,3.9],[]]}

# graph_input = {"o": [[7,44],["a","b","c"]], 
# "a": [[12,40],["d","e","l"]], 
# "b": [[6,37],["g","h","i"]], 
# "c": [[15,45],["j","k","l"]], 
# "d": [[14,33.4],["goal"]], 
# "e": [[23,27],["goal"]], 
# "f": [[23.4,39],["goal"]], 
# "g": [[6,30],[]], 
# "h": [[8,32],["d"]], 
# "i": [[8,30],[]], 
# "j": [[22,43],["f"]], 
# "k": [[18,44],[]], 
# "l": [[18,41],["f"]], 
# "goal": [[26.6,19.4],[]]}

nodes = graph_input.keys()

G = nx.Graph()

G = dict_to_graph(graph_input,nodes)

path_nodes = a_star(G, "start", "goal",graph_input)
path_discrete = []
# path = [[]]*2

for i in path_nodes:
    path_discrete.append(graph_input[i][0])


np.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path_a_star.npy'), path_discrete)



#####################
###### PLOTING ######
#####################


elarge = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] > 0.5]
esmall = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] <= 0.5]

pos = nx.spring_layout(G, seed=7)  # positions for all nodes - seed for reproducibility

pos = {i: np.array(graph_input[i][0]) for i in graph_input}

nodes_color = []

for node in G:
    if node in path_nodes:
        nodes_color.append('tab:green')
    else:
        nodes_color.append('tab:blue')


# nodes
nx.draw_networkx_nodes(G, pos, node_color= nodes_color, node_size=700)

# edges
nx.draw_networkx_edges(G, pos, edgelist=elarge, width=6)
nx.draw_networkx_edges(
    G, pos, edgelist=esmall, width=6, alpha=0.5, edge_color="k", style="dashed"
)
# node labels
nx.draw_networkx_labels(G, pos, font_size=20, font_family="sans-serif")
# edge weight labels
edge_labels = nx.get_edge_attributes(G, "weight")
nx.draw_networkx_edge_labels(G, pos, edge_labels)

ax = plt.gca()
ax.margins(0.08)
plt.axis("off")
plt.tight_layout()
plt.show()




