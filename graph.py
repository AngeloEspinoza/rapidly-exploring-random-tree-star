import pygame
import random
import math
import numpy as np
import random
import queue

class Graph():
	"""
	A class for the Rapidly-exploring Random Tree Star (RRT*).
	
	Attributes
	----------
	start : tuple
		Initial position of the tree in X and Y respectively.
	goal : tuple
		End position of the tree in X and Y respectively.
	map_dimensions : tuple
		Map width and height in pixels.
	"""

	def __init__(self, start, goal, map_dimensions, epsilon, radius):
		self.x_init = start
		self.x_goal = goal
		self.robot_radius = radius

		self.WIDTH, self.HEIGHT = map_dimensions
		self.MAX_NODES = 100
		self.EPSILON = epsilon

		self.obstacles = None
		self.is_goal_reached = False

		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)
		self.YELLOW = (255, 255, 0)
		self.TURQUOISE = (64, 224, 208)
		self.FUCSIA = (255, 0, 255)

	def is_free(self, point, obstacles):
		"""Checks if a node is colliding with an obstacle.

		When dealing with obstacles it is necessary to check 
		for the collision with them from the generated node.

		Parameters
		----------
		point : tuple
			Point to be checked.
		obstacles : pygame.Rect
			Rectangle obstacle.

		Returns
		-------
		bool
		"""
		for obstacle in obstacles:
			if obstacle.colliderect(point):
				return False

		return True

	def generate_random_node(self):
		"""Generates a random node on the screen.

		The x and y coordinate is generated given an uniform
		distribution of the size of the screen width and height.

		Parameters
		----------
		None

		Returns
		-------
		pygame.Rect
			Rectangle of the random node. 
		"""
		x, y = random.uniform(0, self.WIDTH), random.uniform(0, self.HEIGHT)
		x_rand = int(x), int(y) # To use within the class

		# Rectangle generated around the random node
		left = x_rand[0] - self.robot_radius
		top = x_rand[1] - self.robot_radius
		width = 2*self.robot_radius
		height = width
		self.x_rand = pygame.Rect(left, top, width, height)

		return self.x_rand

	def euclidean_distance(self, p1, p2):
		"""Euclidean distance between two points.

		Parameters
		----------
		p1 : int
			Start point.
		p2 : int 
			End point.

		Returns
		-------
		float
			Euclidean distance metric.
		"""
		if not isinstance(p1, tuple):
			p1 = p1.center

		if not isinstance(p2, tuple):
			p2 = p2.center


		return int(math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2))

	def nearest_neighbor(self, tree, x_rand):
		"""Returns the index of the nearest neighbor.
		
		The nearest neighbor from all the nodes in the tree
		to the randomly generated node.

		Parameters
		----------
		tree : list
			Tree containing all the coordinate nodes.
		x_rand : tuple 
			Coordinate of the random node generated.

		Returns
		-------
		tuple
			Nearest node to the random node generated.	
		"""
		distances = []

		for state in tree:
			distance = self.euclidean_distance(state, x_rand.center)
			distances.append(distance)
			
		# Index of the minimum distance to the generated random node
		self.min_distance = np.argmin(distances) 
		x_near = tree[self.min_distance]

		return x_near

	def nearest_neighbors(self, tree, x_new, radius):
		neighbors = []
		distances = []

		for node in tree:
			distance = self.euclidean_distance(node, x_new)
			distances.append(distance)
			if distance <= radius:
				neighbors.append(node)

		return neighbors

	def new_state(self, x_rand, x_near, x_goal):
		"""Advances a small step (self.EPSILON) towards the random node.
		
		Takes small step (self.EPSILON) from the nearest node to the
		random node, if the distance is greater than the small step.
		Otherwise, takes the smallest distance computed by the metric
		distance.

		Parameters
		----------
		x_rand : tuple
			Coordinate of the random node generated.
		x_near : tuple 
			Coordinate of the nearest neighbor node.
		x_goal : tuple
			Coordinate of the goal node.

		Returns
		-------
		tuple
			Coordinate of the new node generated between the nearest
			and random nodes.	
		"""
		if self.euclidean_distance(x_near, x_rand) < self.EPSILON:
			# Check if goal is reached
			if (abs(x_rand[0] - x_goal[0]) < self.EPSILON and 
				abs(x_rand[1] - x_goal[1]) < self.EPSILON): 
				self.is_goal_reached = True
				self.goal_configuration = self.number_of_nodes

			# Rectangle generated around the generated new node
			left = x_rand[0] - self.robot_radius
			top = x_rand[1] - self.robot_radius
			width = 2*self.robot_radius
			height = width
			self.x_rand = pygame.Rect(left, top, width, height)

			# Keep that shortest distance from x_near to x_rand
			return self.x_rand
		else:
			px, py = x_rand[0] - x_near[0], x_rand[1] - x_near[1]
			theta = math.atan2(py, px)
			x_new = (x_near[0] + self.EPSILON*math.cos(theta), 
			         x_near[1] + self.EPSILON*math.sin(theta))

			# Check if goal is reached
			if (abs(x_new[0] - x_goal[0]) < self.EPSILON and
			    abs(x_new[1] - x_goal[1]) < self.EPSILON): 
				self.is_goal_reached = True
				self.goal_configuration = self.number_of_nodes

			# Rectangle around the generated new node
			left = x_new[0] - self.robot_radius
			top = x_new[1] - self.robot_radius
			width = 2*self.robot_radius
			height = width
			self.x_new = pygame.Rect(left, top, width, height)

			return self.x_new

	def generate_parents(self, values, parent):
		"""Generates a list of parents and their children.
		
		Sets up a list of the parents and its corresponding
		children of the tree given a value and the value 
		of the nearest neighbor.

		Parameters
		----------
		values : list
			Collection of values of the assigned x_new node.
		parent : list
			Collection of parents to be fulfilled given its
			correspondant x_near value.

		Returns
		-------
		list
			Ordered collection of the parents.
		"""
		# Value nearest node
		parent_value = values[self.min_distance] 
		# Used to be the index of the parent list
		parent_index = len(parent) 
		parent.insert(parent_index, parent_value)

		if self.is_goal_reached:
			# Insert in the very last index the last value recorded
			# plus one
			parent.insert(parent_index+1, values[-1]+1)

		return parent

	def path_to_goal(self):
		"""Collects the parents of each node.
		
		Given the x_goal node, it searches the next parent
		continously until it reaches the x_init node.

		Parameters
		----------
		None

		Returns
		-------
		None
		"""
		if self.is_goal_reached:
			self.path = []
			self.path.append(self.goal_configuration)
			 # Parent of the x_goal node
			new_configuration = self.parent[self.goal_configuration]

			while new_configuration != 0:
				# Append the parent of the parent and 
				# update the configuration
				self.path.append(new_configuration)
				new_configuration = self.parent[new_configuration]

			# Append the parent 0 (correspondant to the x_init node)
			self.path.append(0)

	def get_path_coordinates(self):
		"""Collects the correspondant coordinates.

		Given a list of the nodes it searches the correspondant
		coordinates in the tree.

		Parameters
		----------
		None

		Returns
		-------
		None
		"""
		self.path_coordinates = []

		for node in self.path:
			x, y = self.tree[node]
			self.path_coordinates.append((x, y))

		return self.path_coordinates

	def find_best_neighbor_and_shortest_path(self, tree, parents, 
	                                         nearest_neighbors, x_rand):
	    """Finds the best neighbor and the shortest path to 
	    the start node.

	    Searches through the nearest neighbors of x_rand, selects the
	    one that minimizes the cost to reach the start node, and
	    returns the shortest path through that neighbor.

	    Parameters
	    ----------
	    tree : list
	        List of nodes in the tree.
	    parents : list
	        List of parent indices for each node in the tree.
	    nearest_neighbors : list
	        List of nodes that are near x_rand.
	    x_rand : tuple
	        The random node for which we are finding the best
	        connection.
	    distance_function : function
	        Function to compute the distance between two nodes.

	    Returns
	    -------
	    best_neighbor : tuple
	        The node in nearest_neighbors that minimizes the cost to
	        reach x_rand.
	    best_cost : float
	        The minimum cost to reach x_rand via best_neighbor.
	    shortest_path : list
	        The shortest path (list of nodes) from the start node
	        to x_rand.
	    """

	    best_neighbor = None
	    best_cost = float('inf')
	    shortest_path = []
	    
	    for neighbor in nearest_neighbors:
	        neighbor_index = tree.index(neighbor)
	        
	        # Compute cost from the start to this neighbor
	        cost_to_neighbor = 0
	        current_index = neighbor_index
	        path_to_neighbor = []
	        
	        while current_index != 0:
	        	parent_index = parents[current_index]
	        	# Break if a node points to itself, preventing
	        	# infinite loops
	        	if current_index == parent_index:
	        		return None, None, None

	        	cost_to_neighbor += self.euclidean_distance(
					p1=tree[current_index],
					p2=tree[parent_index])
	        	path_to_neighbor.append(tree[current_index])
	        	current_index = parent_index

	        # Add the start node to the path
	        path_to_neighbor.append(tree[0])

	        # Reverse to get the correct order (from start to neighbor)
	        path_to_neighbor.reverse()

	        # Add cost from neighbor to x_rand
	        total_cost = (cost_to_neighbor
	                      + self.euclidean_distance(p1=neighbor, p2=x_rand))

	        # Check if this is the best (minimum) cost
	        if total_cost < best_cost:
	            best_cost = total_cost
	            best_neighbor = neighbor
	            shortest_path = path_to_neighbor + [x_rand]

	    return best_neighbor, best_cost, shortest_path

	def find_shortest_path_to_start(self, tree, parents,
	                                start_index, node_index):
	    """
	    Finds the shortest path from a specific node to the start node 
	    in the tree.

	    This function traces back from the specified node to the start 
	    node using the parent links stored in the parents list, thereby 
	    constructing the shortest path to the start node.

	    Parameters
	    ----------
	    tree : list
	        List of nodes in the tree.
	    parents : list
	        List of parent indices for each node in the tree.
	    start_index : int
	        Index of the start node in the tree.
	    node_index : int
	        Index of the target node from which the path to the start 
	        node is found.

	    Returns
	    -------
	    shortest_path : list
	        List of nodes representing the shortest path from the start
	        node to the specified node.
	    """

	    path = []
	    visited = set()  # To avoid cycles

	    current_index = node_index
	    while current_index != start_index:
	        # Add the current node to the path and mark it as visited
	        path.append(tree[current_index])
	        visited.add(current_index)

	        # Move to the parent of the current node
	        current_index = parents[current_index]

	        # Check for a cycle
	        if current_index in visited:
	            return []  # Return an empty path if a cycle is found

	        # If index goes out of bounds or points to itself, break
	        if (current_index >= len(tree) or
	            parents[current_index] == current_index):
	            return []

	    # Append the start node at the end of the path
	    path.append(tree[start_index])
	    path.reverse()  # Reverse to get the path from start to node
	    return path

	def compute_cost_to_start(self, tree, parents, node_index):
	    """
	    Computes the cost (distance) from the start node to a specified
	    node.

	    Parameters
	    ----------
	    tree : list
	        List of nodes in the tree.
	    parents : list
	        List of parent indices for each node in the tree.
	    node_index : int
	        The index of the target node.

	    Returns
	    -------
	    cost : float
	        The total cost to reach the target node from the start.
	    """
	    cost = 0
	    current_index = node_index
	    while parents[current_index] != current_index:
	        cost += self.euclidean_distance(
				p1=tree[current_index].center,
				p2=tree[parents[current_index]].center)
	        current_index = parents[current_index]
	    return cost

	def draw_random_node(self, map_):
		"""Draws the x_rand node."""
		pygame.draw.circle(surface=map_,
		                   color=self.GREEN,
		                   center=self.x_rand.center,
		                   radius=self.robot_radius, width=0)

	def draw_new_node(self, map_, x_new):
		"""Draws the x_near node."""
		if type(x_new) is not tuple:
			x_new = x_new.center

		pygame.draw.circle(surface=map_,
		                   color=self.BROWN,
		                   center=x_new,
		                   radius=self.robot_radius, width=0)

	def draw_initial_node(self, map_):
		"""Draws the x_init node."""
		return pygame.draw.circle(surface=map_,
		                          color=self.BLUE,
		                          center=self.x_init,
			radius=self.robot_radius)

	def draw_goal_node(self, map_):
		"""Draws the x_goal node."""
		return pygame.draw.circle(surface=map_,
		                          color=self.RED,
		                          center=self.x_goal,
		                          radius=self.robot_radius)

	def draw_local_planner(self, p1, p2, map_, color):
		"""Draws the local planner from node to node."""
		pygame.draw.line(surface=map_,
		                 color=color,
		                 start_pos=p1,
		                 end_pos=p2)

	def draw_path_to_goal(self, map_):
		"""Draws the path from the x_goal node to the x_init node."""
		for i in range(len(self.path_coordinates)-1):
			pygame.draw.line(surface=map_,
			                 color=self.RED,
			                 start_pos=self.path_coordinates[i],
			                 end_pos=self.path_coordinates[i+1], width=4)

	def draw_shortest_neighbor_path(self, path, map_, color):
		"""Draws the shortest path found given a neighborhood
		around a node.

		"""
		for i in range(len(path)-1):
			self.draw_local_planner(p1=path[i].center,
			                        p2=path[i+1].center,
			                        map_=map_,
			                        color=color)

	def move_robot(self, position, map_):
		"""Draws the robot moving at the given position."""
		pygame.draw.circle(surface=map_,
		                   color=(0, 0, 255),
		                   center=position,
		                   radius=self.robot_radius)

	def draw_tree(self, tree, parent, environment, should_draw_obstacles):
	    """Draws the entire tree to avoid issues with edge erasing."""
	    
	    # Clear the map
	    environment.map.fill(environment.WHITE)

	    # Draw initial and goal nodes
	    self.draw_initial_node(environment.map)
	    self.draw_goal_node(environment.map)

	    # Draw obstacles again
	    if should_draw_obstacles:
		    environment.draw_obstacles()

	    # Draw all nodes and edges
	    for i in range(1, len(tree)):
	        node = tree[i]
	        parent_node = tree[parent[i]]
	        self.draw_local_planner(
	            p1=parent_node.center, 
	            p2=node.center, 
	            map_=environment.map, 
	            color=self.BLACK
	        )

	def draw_trajectory(self, nears, news, environment,
	                    obstacles, keep_tree):
		"""Draws the robot moving in the map."""
		for i in range(len(self.path_coordinates)-1):
			robot_position = self.path_coordinates[::-1][i]

			if obstacles != []:
				environment.draw_obstacles()

			# Draw inital and final robot configuration
			self.draw_initial_node(map_=environment.map)
			self.draw_goal_node(map_=environment.map)

			# Draw path to goal, and the robot movement
			self.draw_path_to_goal(map_=environment.map)		
			self.move_robot(position=robot_position, map_=environment.map)

			if keep_tree:
				self.draw_tree(nears=nears, news=news, map_=environment.map)				

			# Refresh the screen
			pygame.display.update()
			pygame.time.delay(20)
			environment.map.fill(self.WHITE)