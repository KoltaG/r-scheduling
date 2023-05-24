import numpy as np
import matplotlib.pyplot as plt; plt.close('all')
import networkx as nx
from matplotlib.animation import FuncAnimation

def animate_nodes(G, node_colors, pos=None, *args, **kwargs):
  # define graph layout if None given
  if pos is None:
    pos = nx.random_layout(G)

  # draw graph
  nodes = nx.draw_networkx_nodes(G, pos, *args, **kwargs)
  edges = nx.draw_networkx_edges(G, pos, *args, **kwargs)
  
  plt.axis('off')

  def update(ii):
    # nodes are just markers returned by plt.scatter;
    # node color can hence be changed in the same way like marker colors
    nodes.set_array(node_colors[ii])
    return nodes,

  fig = plt.gcf()
  animation = FuncAnimation(fig, update, interval=100, frames=len(node_colors), blit=True)
  return animation

total_nodes = 7

graph = nx.Graph()
nodes = np.arange(total_nodes)
graph.add_nodes_from(nodes)

for i in range(total_nodes):
  graph.add_edge(i, (i+1) % total_nodes)

time_steps = 6

baseline = [1] * total_nodes
baseline[0] = 2
baseline[round(total_nodes / 2)] = 2

line = np.array(baseline)

node_colors = []

for i in range(time_steps):
  line = np.roll(line, 1)
  colors = line.tolist()
  node_colors.append(colors)

animation = animate_nodes(graph, node_colors)
plt.show()
animation.save('demo.gif', writer='imagemagick', savefig_kwargs={'facecolor':'white'}, fps=1)