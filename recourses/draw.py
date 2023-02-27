import sys
import matplotlib.pyplot as plt
import networkx as nx


def getEdges():
    return [(int(i.split(":")[0]), int(i.split(":")[1])) for i in sys.argv[2:]]

def getEdgesWithWeight():
    return {(int(i.split(":")[0]), int(i.split(":")[1])) : int(i.split(":")[2]) for i in sys.argv[2:]}    

def draw_graph(G, edgelist=[], nodelist=[], pos=None):
    nx.draw_networkx(G, node_color='w', pos=pos, with_labels=True, edgecolors = 'black')
    nx.draw_networkx_edges(G, edge_color='#66FFFF', edgelist=edgelist, pos=pos)
    nx.draw_networkx_nodes(G, node_color='#66FFFF', nodelist=nodelist, pos=pos)
    nx.draw_networkx_edge_labels(G, pos=pos, edge_labels=getEdgesWithWeight())
                                                
    plt.show()  

def set_graph():
    G = nx.Graph()
    G.add_nodes_from(range(0, int(sys.argv[1])))
    G.add_edges_from(getEdges())
    return G
    
def main():
    G = set_graph()
    nx.write_adjlist(G, "graph.txt")
    H = nx.read_adjlist("graph.txt", nodetype=int)
    draw_graph(G, pos=nx.shell_layout(H))
    
main()