import pygame
import environment
import graph 
import argparse
import sys

# Initialization 
pygame.init()

# Constants
MAP_DIMENSIONS = 640, 480

# Initial and final position of the robot
x_init = (50, 50)
x_goal = (540, 380)

# Instantiating the environment and the graph
environment_ = environment.Environment(map_dimensions=MAP_DIMENSIONS)
graph_ = graph.Graph(start=x_init, goal=x_goal, map_dimensions=MAP_DIMENSIONS, epsilon=40.0, radius=8)

def main():
	run = True
	clock = pygame.time.Clock()
	initial = graph_.draw_initial_node(map_=environment_.map)
	goal = graph_.draw_goal_node(map_=environment_.map)
	tree = [] # Tree list containing all the nodes/vertices
	parent = [] # Parent list of each each node/vertex
	values = [] # Values list of each node/vertex
	configurations = []
	tree.append(initial) # Append initial node
	parent.append(0) # Append initial parent
	configurations.append(initial)
	configurations.append(goal)
	environment_.make_obstacles()
	obstacles = environment_.draw_obstacles()
	graph_.obstacles = obstacles


	k = 0
	node_value = 0
	iteration = 0
	nodes_to_values = {}
	parents_nodes = {}
	# bias_percentage = 10 - args.bias_percentage//10 if args.bias_percentage != 100 else 1

	nears = [] # To store the generated x_near nodes

	while run:
		# Make sure the loop runs at 60 FPS
		clock.tick(environment_.FPS)  
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		# Sample free space and check x_rand node collision
		x_rand = graph_.generate_random_node() # Random node 
		rand_collision_free = graph_.is_free(point=x_rand, obstacles=obstacles)
	
		if rand_collision_free:
			x_near = graph_.nearest_neighbor(tree, x_rand) # Nearest neighbor to the random node
			
			nears.append(x_near) # Append nearest neighbor of x_new
			graph_.draw_random_node(map_=environment_.map)

			graph_.number_of_nodes = len(tree)
			node_value += 1 # Increment the value for the next randomly generated node

		pygame.display.update()
	pygame.quit()
	sys.exit()

if __name__ == '__main__':
		main()