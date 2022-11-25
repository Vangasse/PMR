import os
from scipy.spatial import KDTree
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx

class PRM():

    def __init__(self, img, n):
        
        self.k = 6                                      # Número de vizinhos próximos
        self.zero = np.zeros(3,dtype=int)
        self.one = np.ones(3,dtype=int)
        self.map = img
        self.G = nx.Graph()
        self.roadmap = self.createPoints(n)
        self.kd_tree = KDTree(self.roadmap)      
        self.createGraph()
        # print(list(self.G.nodes))
        # print(list(self.G.edges))        
        

    # Método usado para criar o roadmap com n pontos
    def createPoints(self, n):
        points = np.empty((0,2))
        i = 0
        #ones=np.array([1,1,1])
        po = np.array([[10,10],[40,40],[20,10],[20,45],[30,30],[25,20],[47,30],[47,20],[5,14]])
 
        #while i < n:
        while i < po.shape[0]:
            # Gera posições aleatórias (x,y) com base na dimensão do mapa
            #p = np.random.randint(low=[0,0], high=[self.map.shape[0],self.map.shape[1]])
            p = po[i]
         
            if self.checkFreePoint(p):
                points = np.append(points, np.array([p]), axis=0)
                self.map[p[0],p[1]] = (0,0,255)
                i+=1
        
        return points
    
    def checkFreePoint(self,p):
        return np.array_equal(self.map[p[0],p[1]],self.one)
    
    # Método que retorna os k vizinhos mais próximos de um ponto e suas distâncias
    def nearestNeighbors(self, point):
        dist, id = self.kd_tree.query(point, k=range(1,1+self.k))
        return np.transpose(dist), self.roadmap[id]
    
    def createGraph(self):
   
        i = 0
        while i < 1:#self.roadmap.shape[0]:
            point = self.roadmap[i]
            distance,points = self.nearestNeighbors(point)
            
            for n in range(1, distance.shape[0]):                
                if self.checkConnection(points[0],points[n]):
                    self.addNode((points[0][0],points[0][1]),(points[n][0],points[n][1]),distance[n])         
            i+=1

    def addNode(self, node1, node2, d):
        self.G.add_edge(node1, node2, weight = d)
    
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

        for point in line:
            
            if np.array_equal(self.map[int(point[0]), int(point[1])], self.zero):
                return 0

        return 1
    
    def addStartOrGoal(self, point, i):
        
        if self.checkFreePoint(point):
            distance,points = self.nearestNeighbors(point)
            
            for n in range(distance.shape[0]-1):              
                if self.checkConnection(point, points[n]):
                    self.addNode((points[0][0],points[0][1]),(points[n][0],points[n][1]),distance[n])                      
                    self.map[point[0],point[1]] = (0,255,0) if i == 1 else (255,0,0)                      
                    print('start foi conectado ao grafo') if i == 1 else print('goal foi conectado ao grafo') 
                    return 1
            print('Não foi possível conectar start ao grafo') if i == 1 else print('Não foi possível conectar goal ao grafo')
            return 0
        else:
            print('Posição do start está colidindo com obstáculo') if i == 1 else print('Posição do goal está colidindo com obstáculo')
            return 0


def main():

    img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/subsampled.npy'))
    img = np.stack((img,)*3, axis=-1)
  
    start = np.array([40,10])
    goal = np.array([8, 36])

    prm = PRM(img, 100) # Definindo tamanho do espaço e número de nós
    
    prm.addStartOrGoal(start,1)
    prm.addStartOrGoal(goal,2)


    plt.imshow(img,vmin=0,vmax=1)
    plt.show() 
    

if __name__ == '__main__':
    main()


