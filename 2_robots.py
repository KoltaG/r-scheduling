import math
import random

import numpy as np
import matplotlib.pyplot as plt; 
import networkx as nx
from matplotlib.animation import FuncAnimation
import matplotlib.colors as mcolors

# Define the distance function
def dist(u, v):
    if u is None:
        u = [0,0] 
    if v is None:
        v = [0,0]
       
    return math.sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)


# Define the k-robot assignment algorithm
def k_robot_assignment(W, L):
    n = len(W)
    k = len(W[0])
    T = [[] for i in range(k)]
    T_dep = [None for i in range(k)]
    x_dep = [None for i in range(k)]
    for i in range(n):
        w = W[i]
        min_cost = float('inf')
        min_j = None
        for j in range(k):
            cost = dist(w, x_dep[j]) + L
            if cost < min_cost:
                min_cost = cost
                min_j = j
        T[min_j].append(W[i][0])

        if T_dep[min_j] is None or w[1] > W[T_dep[min_j]][1]:
            T_dep[min_j] = len(T[min_j])-1
            x_dep[min_j] = w
    for j in range(k):
        if T_dep[j] is None:
            return False
        T_dep[j] = T[j][T_dep[j]]
    return T, T_dep, x_dep

# Define the single-robot schedule algorithm
def single_robot_schedule(T_weights, T_subtree,T_root):
    #n = len(T_weights)
    n = len(T_subtree)
    L = 0
    print("T_weights", T_weights)
    print("T_subtree", T_subtree) #index of the node in the original graph, not ID! 
    #print("T_root", T_root)
    index_of_t_root = T_subtree.index(T_root)
    #print("indexof T_root", index_of_t_root)
    for i in range(n)[index_of_t_root:]:
        w = T_weights[i][1]
        #print("w in single_robot_schedule", w)
        #L = max(L, dist(T[i][T_dep[i]], T[i][0]) + w)
        #w = T[i][0][1]
        node1 = T_weights[i]
        node2 = T_weights[(i+1) % n]
        #print("node1", node1)
        #print("node2", node2)
        #ITT kéne, hogyha a node1 node2 + w mint L akkor kirajzolni?
        L = max(L, dist(node1, node2) + w)

    return L

    
                 
#Define graph animation function
def draw_graph(self, pos,draw, subtrees ):
    if draw:
        nodes = list(self.nodes())
        print("nodes", nodes)
        node_colors = []
        for node in nodes: 
            #ehelyett bejárni
            if node in subtrees[0]:
                node_colors.append("red")
            elif node in subtrees[1]:
                node_colors.append("green")    
            else:
                node_colors.append("skyblue")

        nx.draw(self, pos, node_size=1500, cmap=mcolors.BASE_COLORS, node_color=node_colors,)

    plt.show()

# Define the main function
def main():
    # Define the weights of the nodes (node, weight)
    # original_weights = [(3, 2), (4, 3), (5, 2),(6, 1), (0, 1), (1, 1), (2, 1), (7, 3),(8, 2)]
    # original_weights = [(3, 2), (4, 3), (5, 2), (0, 5), (1, 1),(6, 1), (2, 1), (7, 3),(8, 2)]
    original_weights = [(1, 3), (2, 2), (3, 1), (4, 2), (0, 3)]
    # original_weights = [(1, 1), (2, 2), (4, 2), (0, 3),(3, 1)]
    # original_weights = []
    # for i in range(9):
    #     original_weights.append((i, random.randint(1,8)))
    # Define the number of robots
    k = 2
    # Compute the smallest and largest weights
    w_min = min(original_weights, key=lambda x: x[1])[1]
    w_max = max(original_weights, key=lambda x: x[1])[1]
    # Compute the value of m
    m = math.log2(w_max/w_min)
    # Round the weights to the nearest dyadic value
    weights = [(original_weight[0], 2**math.ceil(math.log2(original_weight[1]))) for original_weight in original_weights]
    
    #create the graph
    graph = nx.complete_graph(len(weights))
   
    for i in range(len(weights)):
        graph.add_nodes_from(weights[i])

    labels ={}
    for i in range(len(weights)):
        #set the node key as the label key and the label (which is the coord (id+weight)) as its value 
        labels[weights[i][0]] = weights[i]
 
    #pos
    pos = {weights[i][0]:weights[i] for i in range(len(weights))}
    print("pos", pos)
    
    nx.draw(graph, node_size=1500, node_color="skyblue",with_labels=True, pos=pos, labels=labels)
    plt.axis("on")
    plt.savefig("output/original_graph" +  ".png",
                bbox_inches='tight', dpi = 250)

    # Run the k-robot assignment algorithm
    L = 6 # Guess an upper bound for the optimal maximum weighted latency
    T = None
    while T is None:
        T = k_robot_assignment(weights, L)
        if T is None:
            L *= 2
    # Compute the maximum weighted latency of the schedule
    max_latency = 0
    print ("T", T)
    for j in range(k):
        j_subtree = T[0][j]
        #print("j_subtree", j_subtree)
        #print("weights", weights)
        T_j_weights = []
        for i in j_subtree:
            for w in weights:
                if i == w[0]:
                    T_j_weights.append(w)
           
        #print("T_j", T_j_weights)
        T_j_root = T[1][j]
        #print("T_j_root", T_j_root)
        T0_index = None
        for l in range(len(T[0])):
            #print('iteration',l)
            if T[1][j] in T[0][l]:
                T0_index = l
        T_j_subtree = T[0][T0_index]  
        #print("T_j_root 2", T_j_subtree)
       
        
        max_latency = max(max_latency, single_robot_schedule(T_j_weights, T_j_subtree,T_j_root))
    # Print the results
    print('***RESULTS***')
    print('Original weights:', original_weights)
    print('Dyadic weights:', weights)
    print('Subtrees represented by node ids:', T[0])
    print('Node ids and weights:', weights)
    print('Number of robots:', k)
    print('Minimized Maximum weighted latency:', max_latency)

    #save_animations(T[0])
    draw_graph(graph, pos, True, T[0]) 

    

#plt.ion()
    #plt.show()
    
    #draw_graph(graph, True)
    # plt.pause(100)

    # plt.ioff()    






if __name__ == '__main__':
    main()

