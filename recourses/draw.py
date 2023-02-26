import sys
import matplotlib.pyplot as plt
import networkx as nx

def draw_graph(G, edgelist=[], nodelist=[], pos=None):
    nx.draw_networkx(G,node_color='w', pos=pos, with_labels=True, edgecolors = 'black')
    nx.draw_networkx_edges(G, edge_color='#66FFFF', edgelist=edgelist, pos=pos)
    nx.draw_networkx_nodes(G, node_color='#66FFFF', nodelist=nodelist, pos=pos)
    plt.show()  

def _set_graph_v111():
    G = nx.Graph()
    G.add_nodes_from(range(0,4))
    G.add_edges_from([
        (0, 1), (0, 3), (1, 2), 
        (1, 3), (1, 4), (2, 4), 
        (3, 4)
    ])
    return G
    
def main():
    G = _set_graph_v111()
    nx.write_adjlist(G, "graph.txt")
    H = nx.read_adjlist("graph.txt", nodetype=int)
    draw_graph(G, pos=nx.shell_layout(H))
    
main()