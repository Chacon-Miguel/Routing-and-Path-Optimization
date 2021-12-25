# DO NOT MODIFY
class Node(object):
    def __init__(self, name):
        """Assumes name is a string"""
        self._name = name
    def get_name(self):
        return self._name
    def __str__(self):
        return self._name

# DO NOT MODIFY
class Edge(object):
    def __init__(self, src, dest):
        """Assumes src and dest are nodes"""
        self._src = src
        self._dest = dest
    def get_source(self):
        return self._src
    def get_destination(self):
        return self._dest
    def __str__(self):
        return self._src.get_name() + '->' + self._dest.get_name()
    
# DO NOT MODIFY
class Digraph(object):
    #nodes is a list of the nodes in the graph
    #edges is a dict mapping each node to a list of its children
    def __init__(self):
        self._nodes = []
        self._edges = {}
    def add_node(self, node):
        if node in self._nodes:
            raise ValueError('Duplicate node')
        else:
            self._nodes.append(node)
            self._edges[node] = []
    def add_edge(self, edge):
        src = edge.get_source()
        dest = edge.get_destination()
        if not (src in self._nodes and dest in self._nodes):
            raise ValueError('Node not in graph')
        self._edges[src].append(dest)
    def children_of(self, node):
        """Returns a list containing all nodes that can be reached from node in one hop."""
        return self._edges[node]

# IMPLEMENT THIS
def is_path(graph, start, end):
    """Assumes graph is an ACYLIC Digraph; start and end are different nodes in graph
    Returns True if there is a path from start to end in graph and False otherwise."""
    visited = {}
    path_queue = [[start]]
    visited[start] = True

    while len(path_queue) != 0:
        tmp_path = path_queue.pop(0)
        last_node = tmp_path[-1]
        print(last_node)
        if last_node == end:
            return True
        try:
            for next_node in graph.children_of(last_node):
                if next_node not in visited:
                    new_path = tmp_path + [next_node]
                    visited[next_node] = True
                    path_queue.append(new_path)
        except:
            return False
    return False    

d = Digraph()
d._nodes = [1,2,3,4,5]
d._edges = {1:[2,3], 3:[4]}

ans = is_path(d,1,3)