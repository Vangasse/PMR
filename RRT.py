import os

import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

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

        # Notei o problema em que, se escolhida uma dimensão de referência, caso goal e starte estivessem alinhados, não se criava linha.
        numx = int(abs(start[0] - goal[0])/resolution)
        numy = int(abs(start[1] - goal[1])/resolution)
        if numx > numy:
            num = numx
        else:
            num = numy

        x = np.round(np.linspace(start[0],goal[0],num=num))
        y = np.round(np.linspace(start[1],goal[1],num=num))
        x = x.reshape(len(x), 1)
        y = y.reshape(len(y), 1)
        line = np.concatenate((x,y), axis=1)

        # print(x, y)
        # print(line)
        
        for point in line:
            # print(str(point) + ":" + str(self.map[int(point[0]), int(point[1])]))
            if self.map[int(point[0]), int(point[1])] == 0:
                # img[int(point[0]), int(point[1])] = (1,0,0)
                # plt.imshow(img,vmin=0,vmax=1),plt.title('Subsampled')
                # plt.show()
                return 0
            # img[int(point[0]), int(point[1])] = (0,1,0)
            # plt.imshow(img,vmin=0,vmax=1),plt.title('Subsampled')
            # plt.show()
        return 1

    def createRoot(self, node):
        self.G.add_node(node, antecessor=node)

        # elarge = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] > 0.5]
        # esmall = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] <= 0.5]

        # pos = nx.spring_layout(self.G, seed=7)

        # # nodes
        # nx.draw_networkx_nodes(self.G, pos, node_size=700)

        # # edges
        # nx.draw_networkx_edges(self.G, pos, edgelist=elarge, width=6)
        # nx.draw_networkx_edges(
        #     self.G, pos, edgelist=esmall, width=6, alpha=0.5, edge_color="b", style="dashed"
        # )

        # # node labels
        # nx.draw_networkx_labels(self.G, pos, font_size=20, font_family="sans-serif")
        # # edge weight labels
        # edge_labels = nx.get_edge_attributes(self.G, "weight")
        # nx.draw_networkx_edge_labels(self.G, pos, edge_labels)

        # ax = plt.gca()
        # ax.margins(0.08)
        # plt.axis("off")
        # plt.tight_layout()
        # plt.show()

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

            # dx = (node[0] - point[0])**2
            # dy = (node[1] - point[1])**2
            # d = np.sqrt(dx+dy)

            d = sp.spatial.distance.euclidean(node, point)

            if d < d_min:
                d_min = d
                closer = node

        return closer, d_min

    def addNode(self, father, node, d):
        self.G.add_edge(father, node, weight = d)
        nx.set_node_attributes(self.G, {node: father}, name="antecessor")

        # elarge = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] > 0.5]
        # esmall = [(u, v) for (u, v, d) in self.G.edges(data=True) if d["weight"] <= 0.5]

        # pos = nx.spring_layout(self.G, seed=7)

        # # nodes
        # nx.draw_networkx_nodes(self.G, pos, node_size=700)

        # # edges
        # nx.draw_networkx_edges(self.G, pos, edgelist=elarge, width=6)
        # nx.draw_networkx_edges(
        #     self.G, pos, edgelist=esmall, width=6, alpha=0.5, edge_color="b", style="dashed"
        # )

        # # node labels
        # nx.draw_networkx_labels(self.G, pos, font_size=20, font_family="sans-serif")
        # # edge weight labels
        # edge_labels = nx.get_edge_attributes(self.G, "weight")
        # nx.draw_networkx_edge_labels(self.G, pos, edge_labels)

        # ax = plt.gca()
        # ax.margins(0.08)
        # plt.axis("off")
        # plt.tight_layout()
        # plt.show()
        
    def reach(self, start, end, dp, d=3):

        if dp < d:
            return 0
            if d < 1:
                return 0
            node = end
        else:
            q = dp/d

            x = int(np.round(start[0] - (start[0] - end[0])/q))
            y = int(np.round(start[1] - (start[1] - end[1])/q))

            node = (x,y)

        # dx = (start[0] - x)**2
        # dy = (start[1] - y)**2
        # d = np.sqrt(dx+dy)

        d = sp.spatial.distance.euclidean(node, start)

        # print(start, node)
        if self.checkConnection(start, node):
            self.addNode(start, node, d)
            return node
        # print()
        return 0

    def buildTree(self, img):
        img[self.start[0],self.start[1]] = (0,0,1)

        while True:

            point = self.pickPoint()

            closer, dp = self.findCloser(point)

            last_node = self.reach(closer, point, dp)

            if last_node:
                if sp.spatial.distance.euclidean(last_node, self.goal) < 4:
                    self.addNode(last_node, self.goal, sp.spatial.distance.euclidean(last_node, self.goal))
                    img[self.goal[0],self.goal[1]] = (1,0,0)
                    break

                img[last_node[0],last_node[1]] = (0,1,0)

        path = [self.goal]
        while not path[0] == self.start:
            path.insert(0, self.G.nodes[path[0]]["antecessor"])

        return img, path

    def map2env(self, map_path, env_dim=(10,10)):
        map_size = (self.map.shape[0], self.map.shape[1])

        env_path = []
        for i in map_path:
            env_path.append((
                (env_dim[0]/map_size[0])*i[0] - 5,
                (env_dim[1]/map_size[1])*i[1] - 5
            ))

        return env_path

def main():
    img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/subsampled.npy'))

    start = (25, 22)
    goal = (42, 10)

    rrt = RRT(start, goal, img)

    img = np.stack((img,)*3, axis=-1)

    # rrt.checkConnection(start, goal, img)

    img, path = rrt.buildTree(img)

    path = rrt.map2env(path)
    
    plt.imshow(img,vmin=0,vmax=1),plt.title('Subsampled')
    plt.show()

    np.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/path_RRT.npy'), path)

    # for i in range(100):
    #     point = rrt.pickFreePoint()
    #     img[point[0], point[1]] = (1,0,0)

    # plt.imshow(img,vmin=0,vmax=1),plt.title('Subsampled')
    # plt.show()
    
    

if __name__ == '__main__':
    main()