import os
from scipy.spatial import KDTree
import numpy as np
from matplotlib import pyplot as plt

class PRM():

    def __init__(self, img, n):
        self.map = img
        self.roadmap = self.create_roadmap(n)
        self.kd_tree = KDTree(self.roadmap)      

    # Método usado para criar o roadmap com n pontos
    def create_roadmap(self, n):
        points = np.empty((0,2))
        i = 0
        ones=np.array([1,1,1])
        while i < n:
            # Gera posições aleatórias (x,y) com base na dimensão do mapa
            p = np.random.randint(low=[0,0], high=[self.map.shape[0],self.map.shape[1]])
            if self.check_free_point(p):
                points = np.append(points, np.array([p]), axis=0)
                self.map[p[0],p[1]] = (0,0,255)
                i+=1
        return points
    
    def check_free_point(self,p):
        return np.array_equal(self.map[p[0],p[1]],np.array([1,1,1]))
    
    # Método que retorna os k vizinhos mais próximos de um ponto e suas distâncias
    def nearest_neighbors(self, point, k):
        dist, id = self.kd_tree.query(point, k=range(1,1+k))
        return np.transpose(dist), self.roadmap[id[0]]
    
    def add_start_and_goal(self, start, goal):

        if self.check_free_point(start):
            self.map[start[0],start[1]] = (0,255,0)

        if self.check_free_point(goal):
            self.map[goal[0],goal[1]] = (255,0,0)
    


def main():

    img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/subsampled.npy'))
    img = np.stack((img,)*3, axis=-1)
  
    start = np.array([40,10])
    goal = np.array([8, 36])

    prm = PRM(img, 100) # Definindo tamanho do espaço e número de nós
    
    prm.add_start_and_goal(start,goal)

    

    
    # distance, points = prm.nearest_neighbors(start, 5) # Obtendo a distância e os 5 vizinhos de um ponto  
    # print(distance)
    # print(points)

    plt.imshow(img,vmin=0,vmax=1)
    plt.show() 
    

if __name__ == '__main__':
    main()


