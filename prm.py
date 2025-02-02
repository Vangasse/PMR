import os
from scipy.spatial import KDTree
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx
import heapq as hq

class PRM():

    def __init__(self, img, n):
        self.im = 0
        self.k = 6                                      # Número de vizinhos próximos
        self.zero = np.zeros(3,dtype=int)
        self.one = np.ones(3,dtype=int)
        self.map = img
        self.G = nx.Graph()
        self.roadmap = self.createPoints(n)
        self.kd_tree = KDTree(self.roadmap)      
        self.createGraph()   
        

    # Método usado para criar o roadmap com n pontos
    def createPoints(self, n):
        points = np.empty((0,2))
        i = 0
      
        #po = np.array([[10,10],[40,40],[20,10],[25,8],[30,5],[20,45],[22,45],[30,30],[25,20],[25,15],[47,30],[47,20],[5,14],[30,40]])
 
        while i < n:
        #while i < po.shape[0]:
            # Gera posições aleatórias (x,y) com base na dimensão do mapa
            p = np.random.randint(low=[0,0], high=[self.map.shape[0],self.map.shape[1]])
            #p = po[i]
         
            if self.checkFreePoint(p):
                points = np.append(points, np.array([p]), axis=0)
                #self.map[p[0],p[1]] = (0,0,255) # descomentar para visualização
                i+=1
        
        return points
    
    # Método usado para verificar se existe colisão do ponto com algum obstáculo 
    def checkFreePoint(self,p):
        return np.array_equal(self.map[p[0],p[1]],self.one)
    
    # Método que retorna os k vizinhos mais próximos de um ponto e suas distâncias
    def nearestNeighbors(self, point):
        dist, id = self.kd_tree.query(point, k=range(1,1+self.k))
        return np.transpose(dist), self.roadmap[id]
    
    # Método usado para criar o grafo
    def createGraph(self):
   
        i = 0
        while i < self.roadmap.shape[0]:
            
            point = self.roadmap[i]
            distance,points = self.nearestNeighbors(point)
            
            for n in range(1, distance.shape[0]):                
                if self.checkConnection(points[0],points[n]):
                    self.addNode((points[0][0],points[0][1]),(points[n][0],points[n][1]),distance[n])     
                    # print((points[0][0],points[0][1]))   
            i+=1
        self.im = self.map

    # Método usado para adicionar nós no grafo e suas arestas
    def addNode(self, node1, node2, d):
        self.G.add_edge(node1, node2, weight = d)
    
    # Método usado para verificar se existe conexão entre dois pontos por uma linha reta
    def checkConnection(self, start, goal, resolution=.25):

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

        # l1 = []
        # l2 = []
        for point in line:
            
            if np.array_equal(self.map[int(point[0]), int(point[1])], self.zero):
                return 0
            
        #     l1.append(int(point[0]))
        #     l2.append(int(point[1]))

        # for i in range(len(l1)):
        #     self.map[int(l1[i]), int(l2[i])] = (255,127,0)
        return 1
    
    # Método usado para adicionar o ponto inicial e final ao grafo
    def addStartOrGoal(self, point, i):
        
        if self.checkFreePoint(point):
            distance,points = self.nearestNeighbors(point)
            
            for n in range(distance.shape[0]-1):              
                if self.checkConnection(point, points[n]):
                    self.addNode((point[0],point[1]),(points[n][0],points[n][1]),distance[n])                      
                    self.map[point[0],point[1]] = (0,255,0) if i == 1 else (255,0,0)                      
                    print('start foi conectado ao grafo') if i == 1 else print('goal foi conectado ao grafo') 
                    return 1
            print('Não foi possível conectar start ao grafo') if i == 1 else print('Não foi possível conectar goal ao grafo')
            return 0
        else:
            print('Posição do start está colidindo com obstáculo') if i == 1 else print('Posição do goal está colidindo com obstáculo')
            return 0

# Método usado para encontrar um caminho no grafo entre a posição inicial e final
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

    res = [list(goal)]

    searcher = goal
    
    while searcher != start:
        res.append(list(visited.get(searcher)[1]))
        searcher = visited.get(searcher)[1]

    res.reverse()
    return res

def get_distance(a,b):
    return np.sqrt((a[0]- b[0])**2 + (a[1]- b[1])**2)

def map2env(map_path, env_dim=(10,10)):
    map_size = (53, 53)

    env_path = []
    for i in map_path:
        env_path.append((
            ((env_dim[0]/map_size[0])*i[0] - 5),
            ((env_dim[1]/map_size[1])*i[1] - 5)
        ))
    return env_path

def main():

    img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/subsampled.npy'))
    img = np.stack((img,)*3, axis=-1)

    start = np.array([40,10]) #np.array([27,25])
    goal = np.array([8, 36]) #np.array([26, 45])

    prm = PRM(img, 150) # Definindo tamanho do espaço e número de nós
    
    prm.addStartOrGoal(start,1)
    prm.addStartOrGoal(goal,2)

    # prm.map[40,10] = (0,255,0) 
    # prm.map[8,36] = (255,0,0) 

    # print(list(prm.G.nodes))

    # plt.imshow(img,vmin=0,vmax=1)
    # plt.show() 
    

    pathNodes = a_star_PRM(prm.G, (start[0],start[1]), (goal[0],goal[1])) #np.array([26, 45]) #np.array([8, 36])
    pathDiscrete = []

    for i in pathNodes:
        pathDiscrete.append([i[0],i[1]])

    pathDiscrete = map2env(pathDiscrete)
    print(pathDiscrete)

    np.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/path_PRM.npy'), pathDiscrete)


    #####################
    ###### PLOTING ######
    #####################


    elarge = [(u, v) for (u, v, d) in prm.G.edges(data=True) if d["weight"] > 0.5]
    esmall = [(u, v) for (u, v, d) in prm.G.edges(data=True) if d["weight"] <= 0.5]

    pos = nx.spring_layout(prm.G, seed=7)  # positions for all nodes - seed for reproducibility

    nodes_color = []

    for node in prm.G:
        if node in pathNodes:
            nodes_color.append('tab:green')
        else:
            nodes_color.append('tab:blue')


    # nodes
    nx.draw_networkx_nodes(prm.G, pos, node_color= nodes_color, node_size=700)

    # edges
    nx.draw_networkx_edges(prm.G, pos, edgelist=elarge, width=6)
    nx.draw_networkx_edges(
        prm.G, pos, edgelist=esmall, width=6, alpha=0.5, edge_color="k", style="dashed"
    )
    # node labels
    nx.draw_networkx_labels(prm.G, pos, font_size=20, font_family="sans-serif")
    # edge weight labels
    edge_labels = nx.get_edge_attributes(prm.G, "weight")
    nx.draw_networkx_edge_labels(prm.G, pos, edge_labels)

    ax = plt.gca()
    ax.margins(0.08)
    plt.axis("off")
    plt.tight_layout()
    plt.show()

    plt.imshow(img,vmin=0,vmax=1)
    plt.show() 


if __name__ == '__main__':
    main()


