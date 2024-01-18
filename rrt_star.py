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
graph_ = graph.Graph(start=x_init, goal=x_goal, map_dimensions=MAP_DIMENSIONS, epsilon=7.0, radius=5)

def main():
	run = True
	clock = pygame.time.Clock()
	initial = graph_.draw_initial_node(map_=environment_.map)
	goal = graph_.draw_goal_node(map_=environment_.map)
	environment_.make_obstacles()
	obstacles = environment_.draw_obstacles()

	while run:
		clock.tick(environment_.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		pygame.display.update()

	pygame.quit()
	sys.exit()

if __name__ == '__main__':
		main()