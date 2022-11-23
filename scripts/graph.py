import matplotlib.pyplot as plt
import networkx as nx
import heapq as hq
import numpy as np

#######################
###### FUNCTIONS ######
#######################


def a_star(G, start, goal):
    visited = {}
    queue = [(0, start, start)]

    hq.heapify(queue)
    while queue:
        node = hq.heappop(queue)
        
        if node[1] in visited:
            if node[0] < visited[node[1]][0]:
                visited[node[1]] = (node[0], node[2])
            else:
                pass
        else:
            visited[node[1]] = (node[0], node[2])

        neighbors = list(G[node[1]].keys())
        for n in neighbors:
            if not n in visited:
                hq.heappush(queue, (G[node[1]][n]['weight'] + visited[node[1]][0], n, node[1]))

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

nodes = ["o", "goal", "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l"]

# Dictionary
graph_input = {"o": [[7,44],["a","b","c"]], 
"a": [[12,40],["d","e","l"]], 
"b": [[6,37],["g","h","i"]], 
"c": [[15,45],["j","k","l"]], 
"d": [[14,33.4],["goal"]], 
"e": [[23,27],["goal"]], 
"f": [[23.4,39],["goal"]], 
"g": [[6,30],[]], 
"h": [[8,32],["d"]], 
"i": [[8,30],[]], 
"j": [[22,43],["f"]], 
"k": [[18,44],[]], 
"l": [[18,41],["f"]], 
"goal": [[26.6,19.4],[]]}


G = nx.Graph()

G = dict_to_graph(graph_input,nodes)

path_nodes = a_star(G, "o", "goal")
path_discrete = []
path = [[]]*2

for i in path_nodes:
    path_discrete.append(graph_input[i][0])

path = discrete_to_continuos(path_discrete)


with open('path_a_star.txt', 'w') as f:
    for line in path_discrete:
        f.write(str(line[0])+" "+str(line[1]))
        f.write('\n')



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




