import os

import numpy as np
from matplotlib import pyplot as plt

import networkx as nx

class RRT():

    def __init__(self, start, goal, map):
        self.start = start
        self.goal = goal

        self.map = map

        self.G = nx.Graph()
        self.createRoot(self.start)
        

    def plotMap(self):
        plt.imshow(self.map,vmin=0,vmax=1),plt.title('Subsampled')
        plt.show()

    def pickFreePoint(self):

        while True:
            x = np.random.randint(self.map.shape[0])
            y = np.random.randint(self.map.shape[1])

            if self.map[x,y] == 1:
                return (x, y)

    def pickPoint(self):
        x = np.random.randint(self.map.shape[0])
        y = np.random.randint(self.map.shape[1])
        return (x, y)

    def checkConnection(self, start, goal, resolution=.25):

        num = int(abs(start[1] - goal[1])/resolution)

        x = np.round(np.linspace(start[1],goal[1],num=num))
        y = np.round(np.linspace(start[0],goal[0],num=num))
        x = x.reshape(len(x), 1)
        y = y.reshape(len(y), 1)
        line = np.concatenate((x,y), axis=1)
        
        for point in line:
            if self.map[int(point[0]), int(point[1])] == 0:
                return 0
        return 1

    def createRoot(self, node):
        self.G.add_node(node)

        elarge = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] > 0.5]
        esmall = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] <= 0.5]

        pos = nx.spring_layout(self.G, seed=7)

        # nodes
        nx.draw_networkx_nodes(self.G, pos, node_size=700)

        # edges
        nx.draw_networkx_edges(self.G, pos, edgelist=elarge, width=6)
        nx.draw_networkx_edges(
            self.G, pos, edgelist=esmall, width=6, alpha=0.5, edge_color="b", style="dashed"
        )

        # node labels
        nx.draw_networkx_labels(self.G, pos, font_size=20, font_family="sans-serif")
        # edge weight labels
        edge_labels = nx.get_edge_attributes(self.G, "weight")
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels)

        ax = plt.gca()
        ax.margins(0.08)
        plt.axis("off")
        plt.tight_layout()
        plt.show()

    def findCloser(self, point):
        queue = [self.start]
        visited = []
        d_min = np.inf
        closer = self.start

        while queue:
            node = queue.pop(0)
            visited.append(node)

            for son in self.G[node]:
                if not son in visited:
                    queue.append(son)

            dx = (node[0] - point[0])**2
            dy = (node[1] - point[1])**2
            d = np.sqrt(dx+dy)

            if d < d_min:
                d_min = d
                closer = node

        return closer, d_min

    def addNode(self, father, node, d):
        self.G.add_edge(father, node, weight = d)

        elarge = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] > 0.5]
        esmall = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] <= 0.5]

        pos = nx.spring_layout(self.G, seed=7)

        # nodes
        nx.draw_networkx_nodes(self.G, pos, node_size=700)

        # edges
        nx.draw_networkx_edges(self.G, pos, edgelist=elarge, width=6)
        nx.draw_networkx_edges(
            self.G, pos, edgelist=esmall, width=6, alpha=0.5, edge_color="b", style="dashed"
        )

        # node labels
        nx.draw_networkx_labels(self.G, pos, font_size=20, font_family="sans-serif")
        # edge weight labels
        edge_labels = nx.get_edge_attributes(self.G, "weight")
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels)

        ax = plt.gca()
        ax.margins(0.08)
        plt.axis("off")
        plt.tight_layout()
        plt.show()
        
    def reach(self, start, end, dp, d=3):

        q = dp/d

        x = int(np.round(start[0] - (start[0] - end[0])/q))
        y = int(np.round(start[1] - (start[1] - end[1])/q))

        dx = (start[0] - x)**2
        dy = (start[1] - y)**2
        d = np.sqrt(dx+dy)

        node = (x,y)

        if self.checkConnection(start, node):
            self.addNode(start, node, d)
            return 0

        return 1

    # def buildTree(self):

    #     point = self.pickPoint()
        




def main():
    img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/subsampled.npy'))

    start = (24, 36)
    goal = (36, 28)

    rrt = RRT(start, goal, img)

    img = np.stack((img,)*3, axis=-1)

    node = rrt.pickFreePoint()

    closer, d = rrt.findCloser(node)

    while not rrt.reach(closer, node, d):
        pass

    # for i in range(100):
    #     point = rrt.pickFreePoint()
    #     img[point[0], point[1]] = (1,0,0)

    # plt.imshow(img,vmin=0,vmax=1),plt.title('Subsampled')
    # plt.show()
    
    

if __name__ == '__main__':
    main()