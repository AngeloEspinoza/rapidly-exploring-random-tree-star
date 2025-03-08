# Optimal Rapidly-exploring Random Tree (RRT*)

<p align="center">
  <img src="https://github.com/user-attachments/assets/564860a4-4a8b-4972-b38a-580dd96322af" />
</p>

## Description
A 2D simulation in the framework Pygame of the paper [Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/pdf/1105.1186).
The environment has 2 non-convex obstacles that can be used or not. It shows different elements of the tree while building it, like the $\mathbf{x_{\mathit{new}}}$ or $\mathbf{x_{\mathit{rand}}}$ nodes. Also,
the $\varepsilon$ and the maximum allowed nodes variables and be adjusted.

## Usage
```
usage: rrt_star.py [-h] [-o | --obstacles | --no-obstacles] [-n] [-e]
                   [-init  [...]] [-goal  [...]]
                   [-srn | --show_random_nodes | --no-show_random_nodes]
                   [-snn | --show_new_nodes | --no-show_new_nodes] [-bp]
                   [-ptg | --path_to_goal | --no-path_to_goal]
                   [-mr | --move_robot | --no-move_robot]
                   [-kt | --keep_tree | --no-keep_tree]

Implements the RRT* algorithm for path planning.

options:
  -h, --help            show this help message and exit
  -o, --obstacles, --no-obstacles
                        Obstacles on the map
  -n , --nodes          Maximum number of nodes
  -e , --epsilon        Step size
  -init  [ ...], --x_init  [ ...]
                        Initial node position in X and Y, respectively
  -goal  [ ...], --x_goal  [ ...]
                        Goal node position in X and Y, respectively
  -srn, --show_random_nodes, --no-show_random_nodes
                        Show random nodes on screen
  -snn, --show_new_nodes, --no-show_new_nodes
                        Show new nodes on screen
  -bp , --bias_percentage 
                        Amount of bias the RRT from 1 to 100
  -ptg, --path_to_goal, --no-path_to_goal
                        Draws a red line indicating the path to goal
  -mr, --move_robot, --no-move_robot
                        Shows the movements of the robot from the start to
                        the end
  -kt, --keep_tree, --no-keep_tree
                        Keeps the tree while the robot is moving towards the
                        goal

```

## Examples
Generate obstacles in the map, sample 2500 nodes, show the final path to goal in red, and animate the trajectory of the robot 

```python3 rrt_star.py --obstacles --nodes 2500 -ptg -mr```

No obstacles, show the new nodes $\mathbf{x}_{new}$, and show the path to goal once achieved

```python3 rrt_star.py --no-obstacles --show_new_nodes -ptg```

## License 
 MIT License

Copyright (c) [2025] [Angelo Espinoza]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
