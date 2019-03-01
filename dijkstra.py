# Python program for Dijkstra's single  
# source shortest path algorithm. The program is  
# for adjacency matrix representation of the graph 
  
# Library for INT_MAX 
import sys 
  
class Graph(): 
  
    def __init__(self, vertices): 
        self.V = vertices 
        self.graph = [[0 for column in range(vertices)]  
                      for row in range(vertices)] 
  
    def printSolution(self, dist): 
        print ("Vertex tDistance from Source")
        for node in range(self.V): 
            print (node,"t",dist[node]) 
  
    # A utility function to find the vertex with  
    # minimum distance value, from the set of vertices  
    # not yet included in shortest path tree 
    def minDistance(self, dist, sptSet): 
  
        # Initilaize minimum distance for next node 
        min = float("inf") 
  
        # Search not nearest vertex not in the  
        # shortest path tree 
        for v in range(self.V): 
            if dist[v] < min and sptSet[v] == False: 
                min = dist[v] 
                min_index = v 
        return min_index 
  

    # Funtion that implements Dijkstra's single source  
    # shortest path algorithm for a graph represented  
    # using adjacency matrix representation 
    def dijkstra(self, src): 
        
        next_hops = {}

        dist = [float("inf")] * self.V 
        dist[src] = 0
        sptSet = [False] * self.V 
  
        for cout in range(self.V): 
  
            # Pick the minimum distance vertex from  
            # the set of vertices not yet processed.  
            # u is always equal to src in first iteration 
            u = self.minDistance(dist, sptSet) 
  
            # Put the minimum distance vertex in the  
            # shotest path tree 
            sptSet[u] = True
  
            # Update dist value of the adjacent vertices  
            # of the picked vertex only if the current  
            # distance is greater than new distance and 
            # the vertex in not in the shotest path tree 
            for v in range(self.V): 
                if self.graph[u][v] > 0 and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]: 
                        dist[v] = dist[u] + self.graph[u][v]

                        next_hops[(v,src)] = u
  
        self.printSolution(dist)
        print(next_hops)

        route_list = np.array([], [], [], [], [], [])

        for v in range(self.V):
          if next_hops[(v,src)] != src:
            while(next_hops[(next_hops[(v,src)], src)] != src)
            route_list[v, next_hops[(v,src)]]


        return next_hops
  
# Driver program 
# g  = Graph(6) 
# g.graph = [[0,  70,  90,  0,   0,   140], 
#           [70,  0,   100, 150, 0,   0], 
#           [90,  100, 0,   110, 0,   20], 
#           [0,   150, 110, 0,   60,  0], 
#           [0,   0,   0,   60,  0,   90], 
#           [140, 0,   20,  0,   90,  0]]

  
# g.dijkstra(5); 