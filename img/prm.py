from scipy.spatial import KDTree
import numpy as np
from matplotlib import pyplot as plt

class PRM():

    def __init__(self, xy_min, xy_max, n):
        self.rm = self.create_roadmap(xy_min, xy_max, n)
        self.kd_tree = KDTree(self.rm)

    # Método usado para criar o roadmap através do tamanho do espaço e da quantidade de pontos
    def create_roadmap(self, xy_min, xy_max, n):
        return np.random.uniform(low=xy_min, high=xy_max, size=(n,2))

    # Método que retorna os k vizinhos mais próximos de um ponto e suas distâncias
    def nearest_neighbors(self, point, k):
        dist, id = self.kd_tree.query(point, k=range(1,1+k))
        return np.transpose(dist), self.rm[id[0]]
    
    def plot_points(self, point, k):
        distance, points = self.nearest_neighbors(point, k)

        plt.plot(self.rm[:,0],self.rm[:,1],'.',color='black')
        plt.plot(points[:,0],points[:,1],'.',color='red')
        plt.plot(point[:,0],point[:,1],'.',color='blue')
        plt.show()



def main():
    prm = PRM([0,0], [53,53], 100) # Definindo tamanho do espaço e número de nós

    point = np.array([[25, 25]]) # ponto usado para encontrar os vizinhos próximos
    distance, points = prm.nearest_neighbors(point, 5) # Obtendo a distância e os 5 vizinhos de um ponto
  
    prm.plot_points(point, 5)   
    

if __name__ == '__main__':
    main()


