import matplotlib.pyplot as plt
import networkx as nx
import heapq as hq


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
                # print(queue, (G[node[1]][n]['weight'], n, node[2]))
                hq.heappush(queue, (G[node[1]][n]['weight'] + visited[node[1]][0], n, node[1]))

    print(visited, queue)


G = nx.Graph()

G.add_edge("o", "a", weight=1)
G.add_edge("o", "c", weight=1)
G.add_edge("o", "b", weight=1)
G.add_edge("a", "d", weight=1)
G.add_edge("a", "e", weight=1)
G.add_edge("a", "f", weight=3)
G.add_edge("e", "goal", weight=3)
G.add_edge("b", "g", weight=4)
G.add_edge("g", "goal", weight=3)
G.add_edge("b", "h", weight=1)
G.add_edge("b", "i", weight=2)
G.add_edge("i", "goal", weight=3)
G.add_edge("c", "j", weight=1)
G.add_edge("c", "k", weight=1)
G.add_edge("k", "goal", weight=2)
G.add_edge("c", "l", weight=1)

#####################
###### PLOTING ######
#####################
elarge = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] > 0.5]
esmall = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] <= 0.5]

pos = nx.spring_layout(G, seed=7)  # positions for all nodes - seed for reproducibility

# nodes
nx.draw_networkx_nodes(G, pos, node_size=700)

# edges
nx.draw_networkx_edges(G, pos, edgelist=elarge, width=6)
nx.draw_networkx_edges(
    G, pos, edgelist=esmall, width=6, alpha=0.5, edge_color="b", style="dashed"
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

# print(list(G['a'].keys()))

a_star(G, "o", "goal")