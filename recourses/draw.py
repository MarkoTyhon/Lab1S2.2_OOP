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
    G.add_nodes_from(range(1,15))
    G.add_edges_from([
        (1,2),(1,3),(2,3),(1,4),(2,4),(3,4),(1,5),(2,5),(4,5),
        (6,7),(6,8),(7,8),(6,9),(7,9),(8,9),
        (10,11),(10,12),(11,12),(10,13),(11,13)
    ])
    return G

def make_img_graph(G):
    draw_graph(G, pos=nx.shell_layout(G))
    plt.savefig("Graph.png")
    
def main():
    G = _set_graph_v111()
    nx.write_adjlist(G, "graph.txt")
    H = nx.read_adjlist("graph.txt", nodetype=int)
    make_img_graph(H)
    
main()