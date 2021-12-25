# 6.0002 Problem Set 2 Fall 2021
# Graph Optimization
# Name: Miguel
# Collaborators:
# Time: 8 hours

#
# Finding shortest paths to drive from home to work on a road network
#

from graph import DirectedRoad, Node, RoadMap


# PROBLEM 2: Building the Road Network
#
# PROBLEM 2a: Designing your Graph
#
# What do the graph's nodes represent in this problem? What
# do the graph's edges represent? Where are the times
# represented?
#
# Write your answer below as a comment:
# the nodes are road intersections
# the edges are roads
# the times are the weights of each road

# PROBLEM 2b: Implementing load_map
def load_map(map_filename):
    """
    Parses the map file and constructs a road map (graph).

    Travel time and traffic multiplier should be cast to a float.

    Parameters:
        map_filename : String
            name of the map file

    Assumes:
        Each entry in the map file consists of the following format, separated by spaces:
            source_node destination_node travel_time road_type traffic_multiplier

        Note: hill road types always are uphill in the source to destination direction and
              downhill in the destination to the source direction. Downhill travel takes
              half as long as uphill travel. The travel_time represents the time to travel
              from source to destination (uphill).

        e.g.
            N0 N1 10 highway 1
        This entry would become two directed roads; one from 'N0' to 'N1' on a highway with
        a weight of 10.0, and another road from 'N1' to 'N0' on a highway using the same weight.

        e.g.
            N2 N3 7 hill 2
        This entry would become to directed roads; one from 'N2' to 'N3' on a hill road with
        a weight of 7.0, and another road from 'N3' to 'N2' on a hill road with a weight of 3.5.

    Returns:
        a directed road map representing the inputted map
    """
    road_map = RoadMap()

    # get road map data from file
    file = open(map_filename, "r")
    # for every line...
    for line in file:
        # get road data
        dr_data = line.split()

        # convert to actual numbers
        dr_data[2] = float(dr_data[2])
        dr_data[4] = float(dr_data[4])

        # Instantiate source and destination nodes
        dr_data[0] = Node(dr_data[0])
        dr_data[1] = Node(dr_data[1])

        # update set and dictionary of road map by adding node, if needed
        if not road_map.contains_node(dr_data[0]):
            road_map.insert_node(dr_data[0])
        if not road_map.contains_node(dr_data[1]):
            road_map.insert_node(dr_data[1])

        # also get opposite directed road data
        opp_dr_data = dr_data.copy()
        if dr_data[3] == "hill":
            opp_dr_data[2] = dr_data[2] * 0.5

        # switch source and dest nodes for opp directed road
        opp_dr_data[0], opp_dr_data[1] = opp_dr_data[1], opp_dr_data[0]

        # instantiate both roads and add to road map
        directed_road = DirectedRoad(*dr_data)
        opp_directed_road = DirectedRoad(*opp_dr_data)

        road_map.insert_road(directed_road)
        road_map.insert_road(opp_directed_road)
    return road_map
# PROBLEM 2c: Testing load_map
# Include the lines used to test load_map below, but comment them out after testing

road_map = load_map("maps/test_load_map.txt")


# PROBLEM 3: Finding the Shortest Path using Optimized Search Method
# Problem 3a: Objective function
#
# What is the objective function for this problem? What are the constraints?
#
# Answer: 
# The objective function is minimizing the distance traveled to get to a place (i.e. node)
#
#

# PROBLEM 3b: Implement find_optimal_path
def find_optimal_path(roadmap, start, end, restricted_roads, has_traffic=False):
    """
    Finds the shortest path between start and end nodes on the road map,
    without using any restricted roads,
    following traffic conditions.
    Use Dijkstra's algorithm.

    Parameters:
    roadmap - RoadMap
        The graph on which to carry out the search
    start - Node
        node at which to start
    end - Node
        node at which to end
    restricted_roads - list[string]
        Road Types not allowed on path
    has_traffic - boolean
        flag to indicate whether to get shortest path during traffic or not

    Returns:
    A tuple of the form (best_path, best_time).
        The first item is the shortest path from start to end, represented by
        a list of nodes (Nodes).
        The second item is a float, the length (time traveled)
        of the best path.
    """
    # if neither node given is in road map, return None
    if not (roadmap.contains_node(start) and roadmap.contains_node(end)):
        return None
    # if start and end nodes are the same, return the start node and 0 distance
    elif start == end:
        return ([start], 0)

    # mark all nodes as unvisited
    unvisited = list(roadmap.get_all_nodes())
    # mark all nodes as having no predecessor node
    predecessor = {node: None for node in unvisited}
    # mark all nodes as having infinite distance
    dist = {node: float('inf') for node in unvisited}
    # set start node's shortest travel time to 0
    dist[start] = 0
    # initialize current node var to make Python stop bothering me
    curr_node = 0 

    # while there are still nodes to visit...
    while len(unvisited) != 0:
        # set current node to the node with smallest distance
        curr_node = min(unvisited, key= lambda node: dist[node])

        # break if we've reached the end node or if the min distance is infinity
        if curr_node == end:
            break
        elif min(dist.values()) == float('inf'):
            break

        # set current node as visited
        unvisited.remove(curr_node)

        # check the neighboring nodes of the current node
        for road in roadmap.get_reachable_roads_from_node(curr_node, restricted_roads):
            # get the road's end node
            neighbor_node = road.get_destination_node()
            if neighbor_node in unvisited:
                # account for traffic
                if has_traffic:
                    new_dist = road.get_travel_time()*road.get_traffic_multiplier() + dist[curr_node]
                else:
                    new_dist = road.get_travel_time() + dist[curr_node]
                # if the new found distance is smaller than current distance...
                if new_dist < dist[neighbor_node]:
                    # set distance equal to new distance and add the nodes predecessor
                    dist[neighbor_node] = new_dist
                    predecessor[neighbor_node] = curr_node
    # reconstruct path backwards
    path = []
    curr_node = end
    # while we haven't reached the start node
    while predecessor[curr_node] != None:
        # add the current node's predecessor to beginning of list
        path.insert(0, curr_node)
        curr_node = predecessor[curr_node]
    # if an actual path was found, also add the beginning node
    if path != []:
        path.insert(0, curr_node)
        return (path, dist[end])
    # otherwise return nothing
    return None 


# PROBLEM 4a: Implement optimal_path_no_traffic
def find_optimal_path_no_traffic(filename, start, end):
    """
    Finds the shortest path from start to end during conditions of no traffic.

    You must use find_optimal_path and load_map.

    Parameters:
    filename - name of the map file that contains the graph
    start - Node, node object at which to start
    end - Node, node object at which to end

    Returns:
    list of Node objects, the shortest path from start to end in normal traffic.
    If there exists no path, then return None.
    """
    roadmap = load_map(filename)
    return find_optimal_path(roadmap, start, end, [])[0]

# PROBLEM 4b: Implement optimal_path_restricted
def find_optimal_path_restricted(filename, start, end):
    """
    Finds the shortest path from start to end when local roads and hill roads cannot be used.

    You must use find_optimal_path and load_map.

    Parameters:
    filename - name of the map file that contains the graph
    start - Node, node object at which to start
    end - Node, node object at which to end

    Returns:
    list of Node objects, the shortest path from start to end given the aforementioned conditions,
    If there exists no path that satisfies constraints, then return None.
    """
    roadmap = load_map(filename)
    return find_optimal_path(roadmap, start, end, ['local', 'hill'])[0]


# PROBLEM 4c: Implement optimal_path_heavy_traffic
def find_optimal_path_in_traffic_no_toll(filename, start, end):
    """
    Finds the shortest path from start to end when toll roads cannot be used and in traffic,
    i.e. when all roads' travel times are multiplied by their traffic multipliers.

    You must use find_optimal_path and load_map.

    Parameters:
    filename - name of the map file that contains the graph
    start - Node, node object at which to start
    end - Node, node object at which to end; you may assume that start != end

    Returns:
    The shortest path from start to end given the aforementioned conditions,
    represented by a list of nodes (Nodes).

    If there exists no path that satisfies the constraints, then return None.
    """
    roadmap = load_map(filename)
    return find_optimal_path(roadmap, start, end, ['toll'], True)[0]

if __name__ == '__main__':

    # UNCOMMENT THE FOLLOWING LINES TO DEBUG
    rmap = load_map('./maps/small_map.txt')

    start = Node('N0')
    end = Node('N4')
    restricted_roads = []

    print(find_optimal_path(rmap, start, end, restricted_roads))
