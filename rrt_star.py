import pygame
import environment
import graph 
import argparse
import sys

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the RRT* algorithm \
                                 for path planning.')
parser.add_argument('-o',
                    '--obstacles',
                    type=bool,
                    action=argparse.BooleanOptionalAction,
                    metavar='',
                    required=False,
                    help='Obstacles on the map')
parser.add_argument('-n',
                    '--nodes',
                    type=int,
                    metavar='',
                    required=False,
                    default=5000,
					help='Maximum number of nodes')
parser.add_argument('-e',
                    '--epsilon',
                    type=float,
                    metavar='',
                    required=False,
                    default=20.0,
                    help='Step size')
parser.add_argument('-init',
                    '--x_init',
                    nargs='+',
                    type=int,
                    metavar='',
                    required=False,
                    default=(50, 50),
                    help='Initial node position in X and Y, respectively')
parser.add_argument('-goal',
                    '--x_goal',
                    nargs='+',
                    type=int,
                    metavar='',
                    required=False,
                    default=(540, 380),
                    help='Goal node position in X and Y, respectively')
parser.add_argument('-srn',
                    '--show_random_nodes',
                    type=bool,
                    action=argparse.BooleanOptionalAction,
                    metavar='',
                    required=False,
                    help='Show random nodes on screen')
parser.add_argument('-snn',
                    '--show_new_nodes',
                    type=bool,
                    action=argparse.BooleanOptionalAction,
                    metavar='',
                    required=False,
                    help='Show new nodes on screen')
args = parser.parse_args()

# Initialization 
pygame.init()

# Constants
MAP_DIMENSIONS = 640, 480

# Initial and final position of the robot
x_init = args.x_init
x_goal = args.x_goal	

# Instantiating the environment and the graph
environment_ = environment.Environment(map_dimensions=MAP_DIMENSIONS)
graph_ = graph.Graph(start=x_init,
                     goal=x_goal,
                     map_dimensions=MAP_DIMENSIONS,
                     epsilon=args.epsilon,
                     radius=8)

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
	obstacles = environment_.draw_obstacles() if args.obstacles else []
	graph_.obstacles = obstacles
	nodes_counter = 0

	while run:
		# Make sure the loop runs at 60 FPS
		clock.tick(environment_.FPS)  
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		if nodes_counter <= args.nodes:
			# Sample free space and check x_rand node collision
			x_rand = graph_.generate_random_node()	
			rand_collision_free = graph_.is_free(point=x_rand,
			                                     obstacles=obstacles)
			nodes_counter += 1
		
			if rand_collision_free:
				x_near = graph_.nearest_neighbor(tree, x_rand)
				x_new = graph_.new_state(x_rand, x_near, x_goal)
				new_collision_free = graph_.is_free(point=x_new,
				                                    obstacles=obstacles)

				if new_collision_free:
					# Find nearest neighbors around the new node
					nearest_neighbors = graph_.nearest_neighbors(tree=tree,
					                                             x_new=x_new,
					                                             radius=40)

					best_neighbor, best_cost, shortest_path = (
						graph_.find_best_neighbor_and_shortest_path(
	                        tree=tree,
	                        parents=parent,
	                        nearest_neighbors=nearest_neighbors,
	                        x_new=x_new
						)	
					)

					if best_neighbor == None:
						continue

					if args.show_random_nodes:
						graph_.draw_random_node(map_=environment_.map)	

					if args.show_new_nodes:
						graph_.draw_new_node(map_=environment_.map,
						                     x_new=x_new)

					# Draw the x_new node shortest path to goal
					graph_.draw_shortest_neighbor_path(
						path=shortest_path,
						map_=environment_.map,
						color=graph_.BLACK)
					tree.append(x_new)

					# Add the best_neighbor as the parent of x_new
					parent.append(tree.index(best_neighbor))

					# Check and potentially update connections for 
					# each neighbor
					for neighbor in nearest_neighbors:
						neighbor_index = tree.index(neighbor)
						current_cost = graph_.compute_cost_to_start(
							tree=tree,
							parents=parent,
							node_index=neighbor_index)

						# Calculate new cost if x_new is set as 
						# the parent
						new_cost = (best_cost
						            + graph_.euclidean_distance(
	                                    p1=x_new,
	         							p2=neighbor.center))

						# Update parent if the new path is less
						# expensive
						if new_cost < current_cost:
							# Draw over the old connection with 
							# background color to "delete" it
							old_parent_index = parent[neighbor_index]
							old_parent_node = tree[old_parent_index]
							graph_.draw_local_planner(
								p1=old_parent_node.center,
								p2=neighbor.center,
								map_=environment_.map,
								color=environment_.WHITE)

							# Update parent
							parent[neighbor_index] = tree.index(x_new)
							
							# Redraw the entire tree to avoid erasing
							# other edges
							graph_.draw_tree(
								tree=tree,
								parent=parent,
								environment=environment_,
								should_draw_obstacles=args.obstacles)

							# Draw the new connection
							graph_.draw_local_planner(
								p1=x_new.center,
								p2=neighbor.center,
								map_=environment_.map,
								color=graph_.BLACK)


							# Draw the full path from start to the
							# neighbor through the new connection
							path = graph_.find_shortest_path_to_start(
								tree=tree,
	                        	parents=parent,
	                        	start_index=tree.index(initial),
	                        	node_index=neighbor_index)
							graph_.draw_shortest_neighbor_path(
								path=path,
								map_=environment_.map,
								color=graph_.BLACK)

						graph_.number_of_nodes = len(tree)
									
			pygame.display.update()
	pygame.quit()
	sys.exit()

if __name__ == '__main__':
		main()